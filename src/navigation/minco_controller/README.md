# 🎮 MincoMpcController

> 面向全向哨兵底盘的 Nav2 Controller 插件：以 MINCO 轨迹为参考，使用凝聚式 QP MPC 在 SE(2) 状态空间输出全局坐标系速度指令。

[返回项目主页](../../../README.md) · [MINCO Planner](../minco_planner/README.md)

## ✨ 模块定位

`minco_controller::MincoMpcController` 实现 `nav2_core::Controller`，订阅规划器生成的 `/opt_path` 与 Point-LIO 里程计，在 Nav2 Controller Server 的控制周期内求解 MPC。当前主配置控制频率为 **100 Hz**。

主要特性：

- 状态 `x = [px, py, yaw]`，控制 `u = [vx, vy, wz]`。
- qpOASES 求解凝聚后的有约束二次规划。
- 单调推进参考索引，减少高速运动时参考点回跳。
- 速度、角速度及相邻控制量差分约束。
- 控制时延、雷达到底盘参考点杆臂及车体 roll 补偿。
- 支持固定角速度小陀螺模式与 yaw 优化轨迹。
- 发布预测轨迹，提供求解耗时与频率统计。

## 🧠 控制链路

```mermaid
flowchart LR
  T["/opt_path"] --> R[参考序列构造]
  O["/aft_mapped_to_init"] --> S[位姿与全局速度提取]
  S --> L[杆臂 / roll / 时延补偿]
  R --> Q[凝聚式 QP]
  L --> Q
  Q --> C[速度与加速度约束]
  C --> U["/cmd_vel_mpc"]
  Q --> V["/mpc_predict_path"]
```

离散模型以固定 `dt` 预测有限时域状态。代价函数主要由状态跟踪误差 `Q` 与控制输入代价 `R` 构成，并通过上下界约束限制底盘速度及控制变化率。

## 🌐 坐标系与输出约定

控制器从 odom 四元数提取 yaw，将雷达参考点速度转换并补偿到底盘控制参考点。输出的 `vx/vy` 保持为**全局坐标系速度分量**，用于当前下位机通信约定；不要默认把它当作 `base_link` 局部速度。

杆臂速度关系可概括为：

```text
v_base = v_lidar + ω × r
```

其中 `r` 的符号取决于配置中“雷达参考点相对底盘参考点”的定义。修改安装位置后，Planner 与 Controller 必须同步校核。

## 📡 ROS 接口

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/opt_path` | `ros_interfaces/msg/MpcPositionCommand` | MINCO 位置、速度、朝向参考 |
| 输入 | `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 当前位姿、速度与角速度 |
| 输出 | `/cmd_vel_mpc` | `geometry_msgs/msg/TwistStamped` | 提供给上层通信/选择器的控制量 |
| 输出 | `/mpc_predict_path` | `nav_msgs/msg/Path` | MPC 预测轨迹可视化 |

Nav2 标准接口 `setPlan()`、`computeVelocityCommands()` 和 `setSpeedLimit()` 仍由插件实现；项目主链路的高阶轨迹信息来自 `/opt_path`。

## ⚙️ 关键参数

配置位于 `src/navigation/navi2_bringup/params/sentry1.yaml` 的 `controller_server.ros__parameters.FollowPath`。

| 参数 | 当前典型值 | 说明 |
|---|---:|---|
| `controller_frequency` | `100.0` | Controller Server 调用频率 |
| `dt` | `0.05` | MPC 预测离散步长，不等同于控制周期 |
| `lookahead_time` | `0.5` | 参考轨迹前视时间 |
| `control_delay_compensation` | `0.15` | 控制链路时延预测 |
| `Q` | `[3, 3, 2]` | x、y、yaw 跟踪权重 |
| `R` | `[1.5, 1.5, 1]` | vx、vy、wz 输入权重 |
| `vx/vy_min/max` | `±3.0` | 全局平移速度约束 |
| `omega_min/max` | `±5.0` | 角速度约束 |
| `enable_acc_constraints` | `true` | 启用相邻控制量差分约束 |
| `ax/ay_min/max` | `±2.0` | 平移加速度约束 |
| `alpha_min/max` | `±5.0` | 角加速度约束 |
| `use_small_gyro_mode` | `false` | 固定角速度小陀螺模式 |
| `fixed_wz` | `4.18` | 小陀螺目标角速度 |

### 📐 杆臂与姿态补偿

```yaml
lidar_offset_x: 0.0
lidar_offset_y: -0.20
roll_angle: 0.1745
```

- `lidar_offset_x/y`：里程计参考点与底盘控制参考点的平面偏移。
- `roll_angle`：安装姿态导致的固定横滚补偿。
- 三者均是实体标定量，不应仅凭轨迹观感随意调整。
- 修改后应低速验证原地旋转、纯 x/y 平移和组合运动，确认补偿方向正确。

## 🚀 启动与检查

控制器随 Nav2 完整启动：

```bash
ros2 launch navi2 navigation2.launch.py
```

运行时检查：

```bash
ros2 topic hz /opt_path
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cmd_vel_mpc
ros2 topic echo /cmd_vel_mpc --once
```

在 RViz 中叠加 `/opt_path_vis` 与 `/mpc_predict_path`，可快速判断误差来自参考轨迹还是控制器预测。

## 📈 性能观测

详细性能 CSV 默认路径：

```text
/tmp/mpc_perf_detailed.csv
```

重点关注：控制回调实际频率、QP 求解 P95/P99、超时/不可行次数和参考轨迹年龄。100 Hz 控制要求日志与可视化不能阻塞主回调。

## 🛠️ 常见问题

| 现象 | 优先检查 |
|---|---|
| `/cmd_vel_mpc` 无输出 | Nav2 lifecycle、是否收到 `/opt_path` 与 odom |
| 速度方向与车体直觉不一致 | 输出为全局速度；核对下位机坐标约定 |
| 原地旋转伴随平移 | 杆臂偏移符号、雷达安装位置和角速度单位 |
| 高频振荡 | `Q/R` 比例、时延补偿、参考密度、加速度约束 |
| 跟踪明显滞后 | odom/轨迹时间戳、控制延迟、前视时间和下位机延迟 |
| QP 不可行 | 初始状态偏差、速度/加速度边界和参考突变 |
| 小陀螺退出异常 | 上层模式切换是否恢复参数与固定角速度配置 |

## 🗂️ 关键源码

- `src/minco_mpc_controller.cpp`：插件生命周期、参考构造、补偿、QP 与输出。
- `include/minco_controller/minco_mpc_controller.hpp`：状态、参数与接口定义。
- `minco_controller.xml`：pluginlib 注册信息。
- `src/navigation/navi2_bringup/params/sentry1.yaml`：比赛主参数。

## 📚 延伸阅读

轨迹如何生成见 [MincoPlanner 文档](../minco_planner/README.md)，系统启动和通信链路见[项目主 README](../../../README.md)。
