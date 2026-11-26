
#include "lio/super_lio.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>


using namespace BASIC;

namespace LI2Sup{

// 平面拟合：先减去近邻点的质心再做 PCA，取最小特征值方向为法向（几何最小二乘）。
// 原先解 A·n = -1 最小化的是代数残差，残差会被"平面到原点的距离"缩放：离原点越远，
// 越偏向法向指向远处的错误平面。走廊里这些错误平面在走廊方向形成伪约束，把位姿锁在原地。
// 3x3 闭式特征分解比原来 5x3 的列主元 QR 还便宜。
inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (int i = 0; i < N; ++i) centroid += points[i].cast<double>();
  centroid /= N;
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d d = points[i].cast<double>() - centroid;
    cov.noalias() += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.computeDirect(cov);
  if (es.eigenvalues()(1) < 1e-8) return false;  // 近邻重合或共线，法向不确定
  const Eigen::Vector3d normvec = es.eigenvectors().col(0);
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  abcd[3] = -normvec.dot(centroid);

  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > g_plane_fit_tol) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.reserve(20000);
  effect_knn_idxs_.reserve(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    V3 up;
    if (g_init_level_by_ground && EstimateGroundNormal(up)) kf_->SetVerticalFromGround(up);
    if (g_auto_mount_tilt) kf_->CalibrateMountTilt();
    std::vector<Eigen::Vector3d>().swap(ground_cand_);
    kf_->SetImuAnchor(measures_.imu_end);
    kf_->init_ = true;
    data_wrapper_->replayForwardImu();
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  // 雷达接近倒装时两向量几乎反向，单精度下 FromTwoVectors 的结果不是单位四元数，先归一化
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).normalized().toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


// 起飞前静止时，从当前帧（世界系）里找地面平面。加速度计零偏 b 会让世界系歪 b/g，静止时
// IMU 自己分不开；起飞点周围的地面可以直接给出真实竖直方向。只在初始化时算一次。
// 找不到可靠地面（点少、范围小、倾角过大）就返回 false，保持纯重力对齐。
bool SuperLIO::EstimateGroundNormal(V3& up) const {
  constexpr double kInlier = 0.03, kMaxTiltCos = 0.9962 /* cos 5° */, kMinSpread = 1.0;
  constexpr std::size_t kMinInliers = 300;
  const auto& cand = ground_cand_;   // map_init 各帧累积的候选点（相对起飞点，世界系）
  const auto skip = [&](std::size_t inliers, double spread) {
    LOG(WARNING) << "Ground plane not found, keeping accelerometer leveling (candidates " << cand.size()
                 << ", inliers " << inliers << ", spread " << spread << " m)";
    return false;
  };
  if (cand.size() < kMinInliers) return skip(0, 0.0);

  // RANSAC：法向限制在竖直 ±5° 内
  std::mt19937 rng(20261125);
  std::uniform_int_distribution<std::size_t> pick(0, cand.size() - 1);
  Eigen::Vector3d best_n = Eigen::Vector3d::UnitZ();
  double best_d = 0.0;
  std::size_t best = 0;
  long best_score = 0;
  // 候选点里地面通常只占两三成，三点全落在地面上的概率约 1~3%，采样要足够多
  for (int it = 0; it < 600; ++it) {
    const Eigen::Vector3d &a = cand[pick(rng)], &b = cand[pick(rng)], &c = cand[pick(rng)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    const double len = n.norm();
    if (len < 1e-3) continue;
    n /= len;
    if (n.z() < 0.0) n = -n;
    if (n.z() < kMaxTiltCos) continue;
    const double d = -n.dot(a);
    // 评分 = 内点数 − 2×平面下方的点数。地面是最低的面，下方不该有点；略微倾斜、同时蹭到地面和
    // 墙根的平面内点数可能相近，但会把一侧的真实地面点压在下面。
    long count = 0, under = 0;
    for (const auto& q : cand) {
      const double e = n.dot(q) + d;
      count += std::abs(e) < kInlier;
      under += e < -0.05;
    }
    const long score = count - 2 * under;
    if (score > best_score) { best_score = score; best = static_cast<std::size_t>(count); best_n = n; best_d = d; }
  }
  std::size_t below = 0;
  for (const auto& q : cand) below += best_n.dot(q) + best_d < -0.05;
  if (best < kMinInliers || below * 5 > best) return skip(best, 0.0);

  // 内点上做 PCA 精修（重新选内点 → 重新拟合，迭代 3 轮），并检查地面覆盖范围
  Eigen::Vector3d n = best_n;
  double d = best_d;
  std::size_t n_in = 0;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  for (int round = 0; round < 3; ++round) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    n_in = 0;
    for (const auto& q : cand) if (std::abs(n.dot(q) + d) < kInlier) { mean += q; ++n_in; }
    if (n_in < kMinInliers) return skip(n_in, 0.0);
    mean /= static_cast<double>(n_in);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& q : cand) {
      if (std::abs(n.dot(q) + d) < kInlier) cov.noalias() += (q - mean) * (q - mean).transpose();
    }
    es.computeDirect(cov / static_cast<double>(n_in));
    Eigen::Vector3d refined = es.eigenvectors().col(0);
    if (refined.z() < 0.0) refined = -refined;
    n = refined;
    d = -n.dot(mean);
  }
  const double spread = std::sqrt(es.eigenvalues()(1));
  if (spread < kMinSpread || n.z() < kMaxTiltCos) return skip(n_in, spread);

  up = n.cast<scalar>();
  LOG(INFO) << GREEN << " ---> [SuperLIO]: ground plane found (" << n_in << " points), world tilt "
            << std::acos(std::min(1.0, n.z())) * 180.0 / M_PI << " deg corrected" << RESET;
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);

  if (g_init_level_by_ground) {   // 累积地平线附近及以下的点，供起飞前的地面拟合使用
    const Eigen::Vector3d origin = sys_init_pose_.t_.cast<double>();
    for (const auto& p : points_world_v3_) {
      const Eigen::Vector3d d = p.cast<double>() - origin;
      const double r = std::hypot(d.x(), d.y());
      if (r > 1.0 && r < 15.0 && d.z() < 0.05 + 0.05 * r) ground_cand_.push_back(d);
    }
  }

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    bool accepted = false;
    time_record_.Evaluate([this, &accepted]() { accepted = Observe(); }, "[Observe]");
    if (!accepted) return;
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    if (!Observe()) return;
    UpdateMap();
  }
  if (data_wrapper_->scanFresh(measures_.lidar.end_time)) Output();
  caceData();
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (const auto& imu : measures_.imu) {
    if (kf_->Predict(imu)) {
      propagate_states_.emplace_back(kf_->GetDynamicState());
    }
  }
  if (kf_->GetTime() < measures_.imu_end.secs && kf_->Predict(measures_.imu_end)) {
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }
  kf_->InflateForImuGap(measures_.imu_gap);
  std::vector<Quat> rotations;
  rotations.reserve(propagate_states_.size());
  for (const auto& state : propagate_states_) rotations.emplace_back(state.R);

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      const double query_time = start_time + pt.offset_time;
      V3 eigen_point = TLI_R * V3(pt.x, pt.y, pt.z) + TLI_t;
      if (query_time < propagate_states_.back().time && propagate_states_.size() > 1) {
        const auto tail = std::lower_bound(propagate_states_.begin() + 1, propagate_states_.end(),
            query_time, [](const DynamicState& state, double time) { return state.time < time; });
        const auto& head = *std::prev(tail);
        const double tau = std::max(0.0, query_time - head.time);
        const double s = tau / (tail->time - head.time);
        const auto i = tail - propagate_states_.begin();
        const M3 R_i = rotations[i - 1].slerp(s, rotations[i]).toRotationMatrix();
        const V3 p_i = head.p + head.v * tau + 0.5 * tail->a * tau * tau;
        eigen_point = R_inv * (R_i * eigen_point + (p_i - T_end_t));
      }
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  std::size_t inliers = 0;
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


bool SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  const double huber_delta = g_plane_huber_delta;
  // 匹配质量分级（min_match_ratio=0 时与旧版一致：不检查）：
  //   内点 < min_inliers        → 没有可用观测，本帧不更新；
  //   内点 < ratio × 点数       → 照常更新，但信息按 内点/期望内点 缩小（低重叠帧更信 IMU 预测），
  //                               进入新区域时仍有约束、地图能继续生长，不会因硬拒而锁死。
  const bool guard = g_min_match_ratio > 0.0;
  const std::size_t min_inliers = guard ? static_cast<std::size_t>(g_min_inliers) : 1;
  const std::size_t full_inliers = guard
      ? std::max<std::size_t>(min_inliers, std::ceil(g_min_match_ratio * ptsize)) : 1;
  const double inlier_distance = guard ? g_inlier_distance : std::numeric_limits<double>::infinity();

  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  if (ptsize > effect_knn_idxs_.size()) {
    effect_knn_idxs_.resize(ptsize);
    effect_mask_.resize(ptsize);
    effect_knn_mask_.resize(ptsize);
    abcd_vec_.resize(ptsize);
  }

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  const bool accepted = kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
      
            constexpr double information = 1000.0;
            const double distance = std::abs(static_cast<double>(error));
            local_acc.inliers += distance <= inlier_distance;
            const double weight = huber_delta > 0.0 && distance > huber_delta
                ? information * huber_delta / distance : information;
            local_acc.HTVH += (weight * J) * J.transpose();
            local_acc.HTVr -= (weight * error) * J;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    std::size_t inliers = 0;
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
      inliers += local_acc.inliers;
    }
    if (inliers < min_inliers) {
      HTVH.setZero();
      HTVr.setZero();
      return;
    }
    if (inliers < full_inliers) {
      const double scale = static_cast<double>(inliers) / static_cast<double>(full_inliers);
      sum_HTVH *= scale;
      sum_HTVr *= scale;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  frame_num_++;
  if (!accepted) {
    const bool no_obs = kf_->LastUpdateResult() == ESKF::UpdateResult::kNoObservation;
    LOG_EVERY_N(WARNING, 20) << "Skipping LiDAR scan without a reliable pose update ("
                             << (no_obs ? "too few matches" : "non-finite update") << ")";
    return false;
  }
  data_wrapper_->replayForwardImu();
  return true;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetOutputState();
  data_wrapper_->pub_odom(state);
  if (!g_visual_map) return;

  static int count = -1;
  if (++count % g_pub_step != 0) return;
  count = 0;

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  const auto& cloud = g_visual_dense ? scan_undistort_full_ : ds_undistort_;
  pcl::transformPointCloud(*cloud, *ds_world_, transformation);
  data_wrapper_->pub_cloud_world(ds_world_, state.timestamp);
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.