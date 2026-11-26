# Super-LIO

Super-LIO 是一套面向实时定位与建图的紧耦合激光惯性里程计。本仓库提供面向 Livox MID360 的 ROS 2 部署配置，融合雷达点云和内置 IMU 数据，输出六自由度位姿、轨迹和世界坐标系下的去畸变点云。

![Super-LIO 系统架构](docs/system_overview.png)

## 算法原理与优势

- **紧凑的体素地图（OctVox）**。每个体素划分为 8 个子体素，每个子体素维护一个融合后的代表点。增量更新控制了局部地图的点密度，并利用重复观测抑制点云噪声，为点到平面配准保留局部几何结构。
- **利用空间局部性的近邻搜索（HKNN）**。根据查询点在体素内的位置，按预设顺序分组检索候选子体素。当前近邻满足该搜索阶段的距离条件时即可提前结束，减少点云配准中的候选查询与距离计算。
- **迭代误差状态卡尔曼滤波（IESKF）**。IMU 提供帧间状态传播和扫描期间的运动估计，用于点云去畸变。滤波器利用点到局部平面的残差迭代校正状态，再用对齐后的点云更新地图。节点同时提供按雷达帧更新的融合位姿和按 IMU 数据更新的前向传播位姿。

地图表示和查询机制直接作用于每帧配准的主要开销，适合算力和内存预算有限的移动平台。算法细节见 [Super-LIO 原论文](https://arxiv.org/abs/2509.05723)。

## MID360 状态估计

- **扫描时序与数据连续性**。用原始点云的逐点时间偏移确定帧尾，并在帧尾插值 IMU。出现 IMU 断档时，滤波器用两侧样本进行传播，并按缺失时长增大预测不确定度，使后续有效的雷达观测能够校正状态。
- **按匹配质量分配观测权重**。结合近邻平面一致性检查、Huber 权重和内点数量评估激光观测。大残差点与低重叠扫描对滤波更新的影响更小。
- **重力对齐与安装姿态补偿**。初始化时可利用可靠的地面法向确定竖直方向，减轻加速度计零偏引入的姿态倾斜。机体水平静止时可估计雷达的安装俯仰和横滚，并在机体位姿输出中补偿。

## 安装与编译

以下命令以 Ubuntu 22.04、ROS 2 Humble 和 C++20 为例。先编译并加载 Livox 驱动工作区，使 ROS 2 能找到 `livox_ros_driver2`。[ws_livox 驱动与构建说明](https://github.com/xhy-yyy/ws_livox)

```bash
sudo apt install build-essential python3-colcon-common-extensions \
  libeigen3-dev libpcl-dev libgoogle-glog-dev libtbb-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions

source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

mkdir -p ~/ws_super/src
git clone --branch ros2 https://github.com/xhy-yyy/Super-Lio.git ~/ws_super/src/Super-LIO
cd ~/ws_super
colcon build --executor sequential --packages-up-to super_lio
source install/setup.bash
```

## 启动 MID360

在一个终端启动 MID360 驱动，使用 `msg_MID360_launch.py` 发布带逐点时间偏移的 `CustomMsg`，并按实际网卡和雷达地址设置驱动配置。初始化期间保持传感器静止，以估计重力方向和陀螺仪零偏。

```bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

在另一个终端启动里程计。需要 RViz 时，将 `rviz` 设为 `true`。启用安装倾角标定时，初始化期间将机体水平放置。启用地面对齐时，让雷达看到起始位置附近的水平地面。

```bash
source ~/ws_super/install/setup.bash
ros2 launch super_lio Livox_mid360.py rviz:=false
```

### ROS 2 话题

| 方向 | 话题 | 数据 |
| --- | --- | --- |
| 输入 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg`，包含逐点时间偏移 |
| 输入 | `/livox/imu` | `sensor_msgs/msg/Imu`，来自 MID360 内置 IMU |
| 输出 | `/lio/odom` | `world` 坐标系下按雷达帧更新的 IMU 融合里程计 |
| 输出 | `/lio/imu/odom` | `world` 坐标系下的 IMU 前向传播里程计 |
| 输出 | `/lio/robo/odom` | 应用机体外参后的里程计 |
| 输出 | `/lio/path` | 里程计轨迹 |
| 输出 | `/lio/cloud_world` | `world` 坐标系下的去畸变点云 |
| 输出 | `/mavros/vision_pose/pose` | `lio.output.robot` 开启时发布的机体位姿 |

### MID360 参数

| 参数 | 当前默认值 | 用途 |
| --- | --- | --- |
| `lio.sensor.blind` | `0.5` | 近距离过滤阈值，单位为米。距离小于或等于阈值的点会被丢弃 |
| `lio.sensor.maxrange` | `60.0` | 最大保留距离，单位为米 |
| `lio.output.log_pose` | `true` | 有里程计输出时，约每秒记录一次 `x/y/z/yaw`。位置单位为米，航向单位为度 |
| `lio.kf.init_level_by_ground` | `true` | 初始化时使用可靠的地面法向确定竖直方向 |
| `lio.extrinsic.auto_mount_tilt` | `true` | 静止初始化时估计雷达的安装俯仰和横滚 |
| `lio.output.level_by_gravity` | `true` | 使用滤波器估计的重力方向对齐位姿和点云输出 |
| `lio.output.map` | `true` | 发布世界坐标系下的点云 |
| `lio.output.robot` | `true` | 额外发布 MAVROS 视觉位姿 |

修改源码配置后，重新构建以同步安装目录中的 YAML，随后重启节点。[MID360 配置](src/super_lio/config/livox_360.yaml)

## 论文与许可证

本仓库基于 Liansheng Wang 等人的 Super-LIO ROS 2 实现。项目采用 GPL-3.0 许可证。[上游仓库](https://github.com/Liansheng-Wang/Super-LIO/tree/ros2) · [IEEE 论文](https://doi.org/10.1109/LRA.2026.3653372) · [许可证](LICENSE)

```bibtex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```
