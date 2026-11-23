# Super-Lio

面向 Livox MID360 的 ROS 2 激光惯性里程计。基于上游 Super-LIO 的 `ros2` 分支，使用 MID360 点云和内置 IMU 估计位姿、输出去畸变点云。适用于希望在有限算力下运行定位和建图的设备。

## 改进内容

- **以原始点包确定扫描结束时刻。** 使用点包中最大的逐点时间偏移，避免抽样、盲区过滤和距离过滤改变扫描的结束时间。
- **把 IMU 状态传播到扫描末端。** 用帧末两侧的 IMU 样本插值，将传播和去畸变对齐到同一时刻。融合位姿与世界系点云使用同一个时间戳。
- **降低大残差点对配准的影响。** MID360 配置默认启用 Huber 降权。点到平面距离超过 **0.05 m** 后，观测权重随距离增大而降低，减轻墙角、柱边等处不可靠匹配对位姿的拉偏。

上述改动沿用 Super-LIO 的 OctVoxMap、点云抽样和迭代滤波框架。

## 安装与编译

以下命令适用于 **Ubuntu 22.04 / ROS 2 Humble**。先按配套驱动仓库的说明编译并加载 `ws_livox`，使当前终端能够找到 `livox_ros_driver2` 消息包。[ws_livox 安装与构建](https://github.com/xhy-yyy/ws_livox/blob/main/README.md#安装与构建)

```bash
sudo apt install build-essential python3-colcon-common-extensions \
  libeigen3-dev libpcl-dev libgoogle-glog-dev libtbb-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions

source /opt/ros/humble/setup.bash
# 路径按 Livox 驱动工作区的位置调整
source ~/ws_livox/install/setup.bash

mkdir -p ~/ws_super/src
git clone git@github.com:xhy-yyy/Super-Lio.git ~/ws_super/src/Super-Lio
cd ~/ws_super
colcon build --executor sequential --packages-up-to super_lio
source install/setup.bash
```

在 Jetson Orin NX 上部署时，在目标设备上编译源码。首次初始化期间保持雷达静止，让内置 IMU 估计重力方向和陀螺零偏。

## 启动 MID360

在单独的终端启动 Livox 驱动。使用 `msg_MID360_launch.py`，配置 `xfer_format=1`，并根据实际网卡和雷达地址填写驱动的 `MID360_config.json`。

```bash
# Super-Lio 终端
source ~/ws_super/install/setup.bash
ros2 launch super_lio Livox_mid360.py rviz:=false
```

需要 RViz 时将 `rviz` 改为 `true`。仅回放或单独运行里程计时，可用下面的入口关闭额外的 MAVROS 视觉位姿发布。

```bash
source ~/ws_super/install/setup.bash
super_config="$(ros2 pkg prefix --share super_lio)/config/livox_360.yaml"
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$super_config" -p lio.output.robot:=false
```

### 话题

| 方向 | 话题 | 用途 |
| --- | --- | --- |
| 输入 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg`，包含逐点时间偏移 |
| 输入 | `/livox/imu` | `sensor_msgs/msg/Imu`，来自 MID360 内置 IMU |
| 输出 | `/lio/odom` | 世界系下的 IMU 位姿，按点云帧更新 |
| 输出 | `/lio/imu/odom` | 世界系下的 IMU 前向传播位姿，按有效 IMU 样本更新 |
| 输出 | `/lio/robo/odom` | 通过 `lio.extrinsic.odom_robo` 转换的机器人位姿 |
| 输出 | `/lio/cloud_world` | 已去畸变、已变换到 `world` 坐标系的点云 |

### 常用配置

在 [`livox_360.yaml`](src/super_lio/config/livox_360.yaml) 中调整参数。修改源码目录中的 YAML 后，重新构建以更新安装目录中的启动配置。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `lio.kf.plane_huber_delta` | `0.05` | 点面残差开始降权的距离，单位为米。设为 `0.0` 可关闭 |
| `lio.ros.lidar_topic` / `lio.ros.imu_topic` | `/livox/lidar` / `/livox/imu` | 驱动输出话题 |
| `lio.output.robot` | `true` | 向 `/mavros/vision_pose/pose` 额外发布机器人位姿 |
| `lio.output.map` | `true` | 发布世界系点云 |

## 上游与许可证

本项目基于 Liansheng Wang 等人的 Super-LIO ROS 2 实现，基线提交为 `f89f48d`。算法和 OctVoxMap 的引用见原论文 *Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy*，IEEE Robotics and Automation Letters, 2026。[上游源码](https://github.com/Liansheng-Wang/Super-LIO/tree/f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2) · [论文](https://doi.org/10.1109/LRA.2026.3653372) · [GPL-3.0 许可证](LICENSE)
