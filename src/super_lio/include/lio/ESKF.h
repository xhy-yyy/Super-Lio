#ifndef ESKF_HPP_
#define ESKF_HPP_

#include <deque>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "params.h"


namespace LI2Sup{

class ESKF {
public:
  using Ptr   = std::shared_ptr<ESKF>;
                                                  //         0 3 6 9 12 15
  using STATE = Eigen::Matrix<BASIC::scalar, 18, 1>;     //Flatten: R p v bg ba g. 
  using STATE_DOF = Eigen::Matrix<BASIC::scalar, 17, 1>; //Flatten: R p v bg ba g_2.
  using NOISE = Eigen::Matrix<BASIC::scalar, 12, 12>;
  using COV   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_X   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_W   = Eigen::Matrix<BASIC::scalar, 18, 12>;


  struct Options {
    Options(){}
    int num_iterations_ = 3;
    double quit_eps_ = 1e-6;

    double gyro_var_ = 1e-5;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-6;
    double bias_acce_var_ = 1e-4;
  };


  struct KFState{
    bool need_converge = true;
    BASIC::SE3  pose;
  };


  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  ESKF(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8))
      : options_(options) {
    BuildNoise(options);
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
  }

  void SetInitialConditions(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const float imu_scale = 1.0,
                            const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8));

  bool Predict(const IMUData& imu);

  using ObsFunc = std::function<void(const KFState& kf_state, BASIC::M6& HT_Vinv_H, BASIC::V6& HT_Vinv_r)>;
  bool UpdateObserve(ObsFunc obs);

  // kNoObservation: 观测不足（匹配点太少）；kNumerical: 观测或解出现非有限值
  enum class UpdateResult { kAccepted, kNoObservation, kNumerical };
  UpdateResult LastUpdateResult() const { return last_update_result_; }

  double GetTime() const { return current_time_; }

  SysState GetSysState() const { return SysState(current_time_, R_, p_, v_, bg_, ba_); }

  NavState GetNavState() const { return NavState(current_time_, R_, p_, v_); }

  // 调平后的输出位姿。初始化时世界系按加速度均值对齐重力，加速度计零偏会让世界系歪 b/g
  // （30 mg ≈ 1.7°），飞远后 Z 随水平距离线性漂。滤波器在飞行中会估出真实重力方向 g_，
  // 这里用它把输出绕起飞点转平；内部状态和地图不变。
  NavState GetOutputState() const;
  void LevelPose(BASIC::M3& R, BASIC::V3& p, BASIC::V3& v) const;
  // 静止初始化时用地面法向（世界系、朝上）作为真实竖直方向：同时改重力和加速度计零偏，
  // 保持静止时"比力 + 重力 = 0"成立；输出调平立即到位。
  void SetVerticalFromGround(const BASIC::V3& up);
  // 自动标定安装倾角：假定静止初始化时机体水平、航向为 0，此刻雷达相对水平面的姿态就是安装姿态。
  // 之后机体姿态 = 调平后的雷达姿态 × 安装姿态的逆，雷达以任意俯仰/横滚角安装都不用改外参。
  void CalibrateMountTilt();

  DynamicState GetDynamicState() const { return DynamicState(current_time_, R_.R_, p_, v_, body_omega_, global_acc_); }

  KFState GetKFState() const { return KFState{need_converge_, GetSE3()}; }

  Pose_t   GetPoseT() const { return Pose_t(current_time_, R_, p_); }

  COV GetCov() const { return P_; }

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R_, p_); }

  void SetObsTime(const double obs_time) { current_obs_time_ = obs_time; }
  void SetLastObsTime(const double obs_time) { last_obs_time_ = obs_time; }
  void SetImuAnchor(const IMUData& imu);
  void RestartPropagation(const IMUData& imu);
  void InflateForImuGap(double missing);
  void ReplayForward(const std::deque<IMUData>& samples);

  void SetX(const SysState& x);

  void SetCov(const COV& cov){ P_ = cov; }

  BASIC::V3 GetGravity() const { return g_; }

  bool init_ = false;
  bool Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot);

private:
  bool AdvanceForward(const IMUData& imu, BASIC::V3& acc, BASIC::V3& gyr);
  void BuildNoise(const Options& options);
  void Update();
  void UpdateLevel(bool snap = false);

  Eigen::Matrix3d level_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d level_phi_ = Eigen::Vector3d::Zero();     // 调平旋转向量
  Eigen::Vector3d level_origin_ = Eigen::Vector3d::Zero();  // 起飞点，调平绕它旋转
  double level_time_ = -1.0;
  bool mount_calibrated_ = false;
  Eigen::Matrix3d mount_R0_ = Eigen::Matrix3d::Identity();  // 初始化时刻的雷达姿态（内部世界系）
  void ApplyMountTilt();

  UpdateResult last_update_result_ = UpdateResult::kAccepted;

  bool  need_converge_  = true;
  float imu_scale_ = 1.0;
  IMUData last_imu_;
  double current_time_ = 0.0;
  double last_imu_time_ = -1.0;
  double last_obs_time_ = 0.0;
  double current_obs_time_ = 0.0;

  // nominal state
  BASIC::SO3 R_;
  BASIC::V3 p_ = BASIC::V3::Zero();
  BASIC::V3 v_ = BASIC::V3::Zero();
  BASIC::V3 bg_ = BASIC::V3::Zero();
  BASIC::V3 ba_ = BASIC::V3::Zero();
  BASIC::V3 g_{0, 0, - (BASIC::scalar)g_gravity_norm};
  BASIC::V3 global_acc_  = BASIC::V3::Zero();
  BASIC::V3 body_omega_  = BASIC::V3::Zero();

  STATE dx_ = STATE::Zero();

  COV P_ = COV::Identity();

  NOISE Q_ = NOISE::Zero();

  Options options_;

  double  forward_time_ = -1;
  double last_lidar_fix_time_ = -1;
  IMUData forward_last_imu_;
  BASIC::SO3 fw_R_;
  BASIC::V3 fw_p_ = BASIC::V3::Zero();
  BASIC::V3 fw_v_ = BASIC::V3::Zero();
};


}


#endif