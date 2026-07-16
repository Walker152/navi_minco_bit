# 🧭 MincoPlanner

> 面向 RoboMaster 哨兵机器人的 Nav2 全局规划插件：以离散搜索提供拓扑引导，以 ROGMap / ESDF 提供局部环境约束，以 MINCO 生成可实时执行的平滑轨迹。

[返回项目主页](../../../README.md) · [MPC 控制器](../minco_controller/README.md) · [ROGMap](../../perception/rog_map/README.md)

## ✨ 模块定位

`minco_planner::MincoPlanner` 实现 `nav2_core::GlobalPlanner`。Nav2 调用 `createPlan()` 时，插件完成坐标系归一化并更新待处理目标；真正的搜索、优化、重规划与安全检查由内部 FSM 周期执行。规划结果通过自定义轨迹消息发送给 MPC，而不是仅输出离散 `nav_msgs/Path`。

核心能力：

- `PRIORMAP`：基于 Nav2 全局代价地图进行全局搜索，适合先验地图导航。
- `EXPLORATION`：基于 ROGMap 搜索局部可达区域及边界候选。
- 路径裁剪、稀疏化与时间分配，降低优化维度。
- MINCO 多项式轨迹优化，同时约束速度、加速度、障碍距离与终端状态。
- 轨迹热启动、在线重规划、连续安全检查与受控恢复。
- 直接查询 ROGMap / ESDF，避免为规划查询重复序列化大地图。

## 🧠 算法链路

```mermaid
flowchart LR
  G[Nav2 Goal] --> N[坐标系归一化]
  N --> S{planner_mode}
  S -->|PRIORMAP| C[Nav2 Costmap / SMAC 2D]
  S -->|EXPLORATION| R[ROGMap 局部搜索]
  C --> P[路径裁剪与稀疏化]
  R --> P
  P --> T[时间分配 / 热启动]
  T --> M[MINCO 轨迹优化]
  M --> E[ESDF 与动力学安全检查]
  E --> O["/opt_path"]
  E -->|失败或阻塞| F[重规划 / Recovery]
  F --> P
```

MINCO 将轨迹表示为分段多项式，通过有限数量的控制变量联合优化轨迹形状与时间。相较直接逐点跟踪搜索路径，该表示连续、紧凑，并便于对速度、加速度和障碍代价求导。

## 🔄 状态机

主要状态为：

| 状态 | 职责 |
|---|---|
| `INIT` | 等待里程计、地图查询接口等必要条件 |
| `WAIT_GOAL` | 等待新目标；已有目标更新时进入规划 |
| `GENERATE_TRAJ` | 搜索路径、构造初值并执行轨迹优化 |
| `FOLLOW_TRAJ` | 发布并监测当前轨迹，按条件触发重规划 |
| `RECOVERING` | 执行局部脱困并在成功后重新规划 |

> 状态名不等于所有分支均处于当前主路径。判断实际行为时应以 `minco_fsm.cpp` 中未注释的状态转移为准。

## 📡 ROS 接口

### 输入

| 类型 | 名称 | 说明 |
|---|---|---|
| Nav2 API | `createPlan(start, goal)` | 接收规划请求并更新内部目标 |
| `nav_msgs/msg/Odometry` | `/aft_mapped_to_init` | 位姿、速度与轨迹起始状态 |
| 内部查询 | `MapQueryInterface` | ROGMap 占据、投影层与 ESDF 查询 |
| Nav2 Costmap | `planner_server` costmap | `PRIORMAP` 模式的全局搜索依据 |

### 输出与可视化

| Topic | 类型 / 用途 |
|---|---|
| `/opt_path` | `ros_interfaces/msg/MpcPositionCommand`，MPC 主参考轨迹 |
| `/backup_path` | 兼容性轨迹通道；当前文档不将其作为主规划能力 |
| `/opt_path_vis` | 优化轨迹可视化 |
| `/astar_path_vis` | 离散搜索路径可视化 |
| `/minco_control_points_vis` | MINCO 控制点可视化 |
| `/recover_path`, `/recover_goal` | 恢复过程诊断 |

## ⚙️ 关键配置

主配置位于 `src/navigation/navi2_bringup/params/sentry1.yaml` 的 `planner_server.ros__parameters.MincoPlanner`。

### 模式与坐标系

| 参数 | 当前典型值 | 含义 |
|---|---:|---|
| `planner_mode` | `PRIORMAP` | `PRIORMAP` / `EXPLORATION` |
| `odom_topic` | `/aft_mapped_to_init` | Point-LIO 里程计 |
| `map_frame` | `map` | 先验地图规划坐标系 |
| `rog_frame` | `camera_init` | ROGMap 与局部轨迹坐标系 |
| `use_smac` | `true` | 先验地图模式使用 SMAC 2D 搜索 |

### 轨迹与安全

| 参数组 | 作用 | 调参影响 |
|---|---|---|
| `optimizer.max_vel`, `max_acc` | 线速度、线加速度上限 | 过高会增加跟踪压力，过低会降低机动性 |
| `optimizer.safe_distance` | 优化障碍安全距离 | 应与车体外形、地图膨胀共同确定 |
| `optimizer.collision_distance` | 碰撞判定阈值 | 不应大于安全距离 |
| `optimizer.lookahead_distance` | 局部优化前视长度 | 越大越平滑，但计算量与局部地图依赖增加 |
| `optimizer.integral_resolution` | 代价积分采样密度 | 越高越精细，也越耗时 |
| `optimizer.enable_yaw_opt` | 是否联合优化朝向 | 隧道、侧移和高速转向需结合控制器验证 |
| `recovery.*` | 连续失败阈值、冷却与脱困参数 | 影响阻塞后的恢复激进程度 |

### 📐 Planner 杆臂补偿

`lidar_offset_x/y` 描述雷达/里程计参考点相对底盘控制参考点的平面偏移。当前典型配置为：

```yaml
lidar_offset_x: 0.0
lidar_offset_y: -0.20
```

该补偿会影响规划初始状态。它必须与 Point-LIO 发布的参考点、控制器中的同名偏移及实体安装方向一致。不要通过修改该参数修补错误的 TF；先确认坐标轴方向和偏移定义。

## 🚀 启动与检查

本模块通常随完整导航启动，不建议独立启动插件：

```bash
ros2 launch navi2 navigation2.launch.py
```

启动后可检查：

```bash
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /opt_path
ros2 topic echo /opt_path --once
ros2 param get /planner_server MincoPlanner.planner_mode
```

推荐在 RViz 同时观察 `/astar_path_vis`、`/opt_path_vis`、ROGMap 占据层和 ESDF，区分“搜索不可达”“优化失败”和“地图输入异常”。

## 📈 性能观测

性能监视器可记录里程计回调频率、规划阶段耗时和优化统计，详细 CSV 默认写入：

```text
/tmp/minco_perf_detailed.csv
```

在线调参时优先关注 P95/P99 耗时而非单次最快值，并同时观察轨迹可执行性。高频日志会扰动实时线程，比赛配置中应保持克制。

## 🛠️ 常见问题

| 现象 | 优先检查 |
|---|---|
| 一直停留在 `INIT` | 里程计、ROGMap 创建、坐标系与生命周期状态 |
| 有搜索路径但无 `/opt_path` | 时间分配、初值、ESDF、安全距离和优化器返回状态 |
| 轨迹贴障或穿障 | 点云时延、ROGMap 投影高度、ESDF 更新、安全/碰撞距离 |
| 频繁重规划 | 地图抖动、目标更新频率、重规划条件、速度/加速度约束 |
| 起步方向或速度异常 | Planner 与 Controller 杆臂参数、odom 参考点和 yaw 定义 |
| `EXPLORATION` 无结果 | 起点是否位于 ROGMap、局部边界是否存在可达候选 |

## 🗂️ 关键源码

- `src/minco_core/minco_planner.cpp`：插件配置、目标接入、地图创建与参数更新。
- `src/minco_core/minco_fsm.cpp`：规划状态机与重规划逻辑。
- `src/minco_core/components/global_path_searcher.cpp`：两种模式的全局/局部搜索。
- `src/traj_opt/minco_optimizer.cpp`：MINCO 优化入口。
- `src/minco_core/components/trajectory_safety_checker.cpp`：轨迹安全查询。
- `include/minco_core/`：插件、FSM 与组件接口。

## 📚 延伸阅读

系统级安装、启动顺序与完整参数关系见[项目主 README](../../../README.md)。地图内部结构见 [ROGMap 文档](../../perception/rog_map/README.md)，轨迹执行见 [MPC 控制器文档](../minco_controller/README.md)。
