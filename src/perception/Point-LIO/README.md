# 🛰️ Point-LIO ROS 2

> 面向 Livox MID-360 与高速旋转哨兵场景的高频激光惯性里程计组件，提供去畸变定位、稠密世界系点云和低开销进程内感知链路。

[返回项目主页](../../../README.md) · [Livox Driver](../livox_ros_driver2/README.md) · [ROGMap](../rog_map/README.md)

## ✨ 本仓库版本亮点

- ROS 2 Component 化，可与 Livox Driver、Planner/ROGMap 部署在同一多线程容器。
- `CustomMsg::UniquePtr` 输入与 `PointCloud2::UniquePtr` 输出，为进程内零拷贝/少拷贝提供必要条件。
- 原始点云、滤波定位点云与 `/cloud_registered_full` 稠密点云职责分离。
- 稠密点云按点时间稳定排序，结合状态快照进行去畸变并变换到世界系。
- “最新完整帧”发布采用 SensorData QoS `keep_last(1)`，减少旧大消息排队。
- 支持双雷达融合输入，由驱动层完成时间窗聚合与外参变换。
- 增加 `blind_center`，可将近场盲区中心对齐到实际底盘中心。
- 独立工作线程分离 ROS 回调与主 LIO 处理，降低高频调度互相阻塞。

> `UniquePtr + intra-process` 能显著减少同进程消息拷贝，但是否达到完全零拷贝还取决于 ROS 2 中间件、发布/订阅类型和容器部署方式。

## 🔄 数据链路

```mermaid
flowchart LR
  D[Livox CustomMsg] --> Q[回调队列]
  I[Livox IMU] --> Q
  Q --> P[预处理 / 时间排序]
  P --> U[IMU 传播与点云去畸变]
  U --> E[ESIKF 状态更新]
  E --> O["/aft_mapped_to_init"]
  E --> C["/cloud_registered"]
  U --> F[稠密帧状态快照与世界系变换]
  F --> CF["/cloud_registered_full"]
  CF --> R[ROGMap]
```

定位主链路可以使用降采样点云控制计算量；`/cloud_registered_full` 则尽量保留当前扫描的完整有效点，为 ROGMap 的 raycast、占据更新与 ESDF 提供更充分的观测。

## 📡 ROS 接口

### 输入

| Topic | 类型 | 说明 |
|---|---|---|
| `livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 单雷达或驱动层融合后的点云 |
| `livox/imu_192_168_1_135` | `sensor_msgs/msg/Imu` | 当前主雷达 IMU，名称取决于雷达 IP |

### 输出

| Topic | Frame | 说明 |
|---|---|---|
| `/aft_mapped_to_init` | `camera_init` → `body` | 高频 LIO 里程计 |
| `/cloud_registered` | `camera_init` | 定位/可视化注册点云 |
| `/cloud_registered_full` | `camera_init` | 稠密、去畸变、世界系完整帧，ROGMap 主输入 |
| `/cloud_registered_body` | `body` | 机体系注册点云 |
| `/Laser_map` | `camera_init` | 累积地图 |
| `/path` | `camera_init` | 里程计路径 |

启用 TF 发布后，模块发布 `camera_init → body`，并按项目约定提供 `camera_init → base_link`、`camera_init → slambase`。这些变换含项目特定安装补偿，改变雷达位置时必须核对源码与下游 frame 约定。

## ⚙️ 关键配置

主配置：`config/mid360.yaml`。

### 输入与 IMU

```yaml
common:
  lid_topic: "livox/lidar"
  imu_topic: "livox/imu_192_168_1_135"
  use_imu_as_input: false
  prop_at_freq_of_imu: true
```

雷达 IP 改变后，驱动生成的逐雷达 IMU topic 也会改变；必须同步修改 `imu_topic`。

### 🎯 Blind 中心

```yaml
preprocess:
  blind: 0.45
  blind_center_enabled: true
  blind_center: [0.0, 0.20, 0.0]
```

传统盲区以雷达原点为中心。本项目允许把盲区中心平移到底盘中心附近，避免安装偏置导致车体近场点在不同方向被不一致地保留。`blind_center` 是预处理距离判断中心，不是 TF，也不会替代正确外参。

### 📐 Lidar–IMU 外参

```yaml
mapping:
  extrinsic_est_en: false
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
```

比赛配置关闭在线外参估计，依赖离线标定值。平移单位为米，旋转矩阵需保持正交。错误外参会表现为旋转时地图重影、墙面弯曲和 odom 漂移。

### 发布与地图

| 参数 | 作用 |
|---|---|
| `publish.tf_send_en` | 发布 LIO TF |
| `publish.scan_publish_en` | 发布注册点云 |
| `publish.dense_publish_en` | 保留更稠密的输出 |
| `publish.scan_bodyframe_pub_en` | 发布机体系点云 |
| `publish_accumulated_map` | 发布累积地图 |
| `publish.accumulated_map_publish_hz` | 限制大地图发布频率 |
| `pcd_save.pcd_save_en` | 保存 PCD；会产生磁盘开销 |

## 🚀 推荐启动

单/双 MID-360 与 Point-LIO 的推荐组合入口位于 Point-LIO launch 目录：

```bash
ros2 launch point_lio mixed_livox_pointlio_intra_process.launch.py
```

该 launch 将驱动与 Point-LIO 放入同一 `component_container_mt` 并开启 intra-process。若拆分到不同进程，功能仍可运行，但大点云将经过 DDS 序列化链路。

启动后检查：

```bash
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
ros2 topic info /cloud_registered_full --verbose
ros2 run tf2_ros tf2_echo camera_init body
```

## ⚡ 稠密点云性能设计

`/cloud_registered_full` 的处理重点不是简单“多发一份点云”，而是保证下游可用性：

1. 对帧内点按相对时间稳定排序。
2. 保存与点云对应的滤波器状态快照。
3. 将各点补偿到统一时刻并转换到 `camera_init`。
4. 丢弃过旧、乱序和非有限点，避免污染地图。
5. 以 `UniquePtr` 发布，QoS 仅保留最新帧。

这条链路针对高速小陀螺运动尤其重要：点时间、去畸变和姿态传播任何一项错误，都会把静态墙面拉成动态障碍。

## 🛠️ 常见问题

| 现象 | 优先检查 |
|---|---|
| 收不到点云 | 雷达 IP、主机网卡、驱动 JSON、topic remap |
| 收到雷达但无 IMU | `imu_topic` 是否与主雷达 IP 生成的 topic 一致 |
| 原地转动地图重影 | 时间同步、IMU 外参、点时间单位与去畸变 |
| `/cloud_registered_full` 延迟增长 | 是否跨进程、QoS 队列、下游订阅是否阻塞 |
| 地图中心与车体错位 | 区分 `blind_center`、TF、Planner/Controller 杆臂参数 |
| 地面/高障碍进入 ROGMap 异常 | Point-LIO frame 与 ROGMap 投影 Z 范围 |
| CPU 占用过高 | 发布开关、降采样、累积地图/PCD、可视化订阅 |

## 🗂️ 关键源码

- `src/laserMapping.cpp`：LIO 主流程、odom/TF 与点云发布。
- `src/preprocess.cpp`：Livox 点云预处理、盲区与时间字段。
- `src/IMU_Processing.hpp`：IMU 传播和运动补偿。
- `config/mid360.yaml`：项目主参数。
- `launch/`：Point-LIO 独立或组合启动入口。

## 📚 延伸阅读

驱动网络、双雷达融合与组件部署见 [Livox Driver 文档](../livox_ros_driver2/README.md)；点云如何进入占据地图和 ESDF 见 [ROGMap 文档](../rog_map/README.md)。
