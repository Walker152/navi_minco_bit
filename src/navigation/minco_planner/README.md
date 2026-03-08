# BIT minco_planner
## DreamChaser
`minco_planner` 是一个面向 RoboMaster 高动态地面平台的 **Nav2 GlobalPlanner 插件**：
- 前端全局引导支持 `SmacPlanner2DSimple`（Costmap + ESDF 势场偏置）与 A* 回退路径。
- 中层以固定频率执行局部前瞻裁剪、稀疏化、视线碰撞修复（角点插入）与走廊约束构建。
- 后端调用 MINCO 优化器输出动态可行轨迹，并持续生成备份刹停/缓停轨迹。
- 运行期由独立 FSM 管控重规划、急停、恢复与超时退避，适配对抗环境下的突发遮挡与路径失效。

> 插件导出：`minco_planner::MincoPlanner`（见 `global_planner_plugin.xml`）。

---

## 1. 功能包简介

在整个导航系统中，`minco_planner` 的角色可以概括为：

1) **全局路径生成（离散引导）**
- 默认链路是 `MincoPlanner::makePlan()` -> A* 离散引导；当 `use_smac` 打开时，前端切换为 `SmacPlanner2DSimple::createPath()`。
- `SmacPlanner2DSimple` 将 **Costmap 栅格代价** 与 **ESDF 势场代价** 融合：
  - 栅格代价项：`tentative_g += step_cost * (1 + cost_penalty * normalized_cost)`；
  - ESDF 势场项：`tentative_g += getESDFPotentialCost(nx, ny)`，采用指数衰减势函数抑制贴障通行。
- 工程层面保留“死锁逃离”策略位：在狭窄膨胀层通道中可放宽可通行阈值，实车模式下可配置为仅将 `LETHAL_OBSTACLE=254` 作为硬阻挡（用于脱困），而常规模式仍维持保守阈值。
- 输出：`latest_global_path_`（离散全局引导）并同步可视化。

2) **局部前瞻段优化（可执行轨迹）**
- 局部段由 `lookahead_dist` 裁剪，随后按弧长构造梯形/三角速度时间分配，避免采样点在急弯处过稀导致拟合失真。
- 稀疏化采用 Greedy Line-of-Sight（`isLineFree`）逐段验证：
  - 若当前直连段碰撞，触发 **Corner Insertion**，在冲突区间插入最大偏离角点并继续验证；
  - 直到形成可直连的稀疏控制点序列，作为 MINCO 优化输入。
- 走廊约束来自 `HybridESDFMap`：在线构建轴向对齐安全包围盒并转换为 `PolyhedronH` 半空间约束。
  - 走廊半尺寸会严格扣除 `robot_radius_` 与 `extra_margin`；
  - 对退化场景设置几何下界 `kMinHalfSize=0.05m`，防止矩阵奇异或空走廊导致优化器数值崩溃。

3) **有限状态机 FSM（运行鲁棒性核心）**

```mermaid
stateDiagram-v2
  [*] --> INIT
  INIT --> WAIT_GOAL: 有里程计/位姿
  WAIT_GOAL --> GENERATE_TRAJ: consumePendingGoal
  GENERATE_TRAJ --> FOLLOW_TRAJ: PlanGlobalPath && ReplanLocal 成功
  GENERATE_TRAJ --> EMER_STOP: 全局或局部重规划失败

  FOLLOW_TRAJ --> GENERATE_TRAJ: 轨迹超时/不安全/10Hz强制刷新且重规划失败
  FOLLOW_TRAJ --> WAIT_GOAL: 目标到达且速度<0.2 或 目标取消
  FOLLOW_TRAJ --> EMER_STOP: 安全条件破坏（由上游状态切转）

  EMER_STOP --> FOLLOW_TRAJ: EMER_RECOVER(ReplanLocal 成功)
  EMER_STOP --> WAIT_GOAL: 急停超时(>5s)保护/底盘速度 < 0.1 m/s
```

FSM 关键触发条件（实车重点）：
- `INIT -> WAIT_GOAL`：仅在底层位姿可用时解锁系统。
- `WAIT_GOAL -> GENERATE_TRAJ`：`createPlan()` 只写入 pending goal，真正规划由 FSM 消费触发。
- `FOLLOW_TRAJ` 强制刷新：每 `0.1s`（10Hz）触发一次重规划，避免轨迹“卡死”但仍被持续执行。
- `EMER_STOP` 阶段：
  - 首次进入立刻发布独立刹停轨迹；
  - 若可恢复则 `ReplanLocal` 成功后回 `FOLLOW_TRAJ`；
  - 若不可恢复则阻塞等待底盘降速，**只有实际速度 `< 0.1 m/s` 才可进入 `WAIT_GOAL`**；

4) **运行期输出**
- 控制输出：`/opt_path`（主轨迹）与 `/backup_path`（备份刹停/缓停）。
- 调试输出：A* / SMAC 引导路径、控制点、优化轨迹、备份轨迹、ESDF 点云。
- 所有输出遵循“主轨迹优先、备份轨迹兜底”的实车安全策略。


---

## 2. 算法与数据流

```mermaid
flowchart TD
  subgraph S["Search"]
    A["Nav2 createPlan(start, goal)"] --> 
    B["前端选路：SMAC(可选) / A* 回退"]
    B --> C["代价构建：Costmap + ESDF Potential"]
    C --> D["离散引导路径（latest_global_path_）"]
end
  subgraph O["Optimize"]
    H["按 lookahead_dist 裁剪局部段"]
    H --> J["梯形/三角速度时间分配<br/>s(t) 采样"]
    J --> K["Line-of-Sight 检测<br/>Corner Insertion 修复"]
    K --> K2["HybridESDFMap 走廊构建<br/>PolyhedronH + 安全余量"]
    K --> L["确定规划状态<br/>COLD / HOT / EMERGENCY"]
    L --> M["生成备份轨迹<br/>ESDF 安全盒 + 前向减速五次多项式"]
    K2 --> N["MINCO 优化<br/>变量：时间 T + 中间控制点"]
    L --> N
    F --> H
  end
```

---

## 3. 算法工作流程

本节按“从一次规划请求到运行期持续优化”的顺序描述。

### 3.1 A* 离散路径搜索（全局引导）

触发点：Nav2 调用 `createPlan(start, goal)`。

1) **坐标与合法性检查**
- 将 start/goal 从 world 坐标映射到 costmap 栅格坐标；
- 当 `tolerance==0` 且 goal 落入 LETHAL 代价时拒绝；
- 清空机器人所在栅格（避免起点被自身占用代价影响）。

2) **SMAC+ESDF / A* 双前端实现**
- 默认路径：`Astar` 生成稳定离散引导；
- 可选路径：`SmacPlanner2DSimple`（`smac_search/smac_planner_2d_simple.cpp`）进行 8 邻接 A* 风格搜索，并在 `g` 代价中叠加 `getESDFPotentialCost()`。
- `getESDFPotentialCost()` 关键点：
  - 将栅格中心映射到 world 坐标；
  - 查询 `HybridESDFMap::evaluate()` 获取距离；
  - 指数衰减势场 `w*exp(-dist/decay)`，靠障碍处代价更大。
- 可通行判定与死锁逃离：
  - 常规模式依据 costmap 阈值拒绝高风险障碍单元；
  - 实车“脱困策略”可放宽到仅 `LETHAL_OBSTACLE=254` 阻挡，允许在膨胀边界内搜索可行逃逸路径。

3) **势场回溯得到离散路径**
- 从 start 位置沿势场做 8 邻接“梯度下降”，每次选择 `potarr` 更小的邻居；
- 写入 `pathx/pathy` 缓冲并最终转换到 world 坐标写入 `nav_msgs::Path`；
- 同时缓存到 `latest_global_path_`，供运行期局部优化使用。

> 说明：当前包提供 `use_smac` 参数用于切换前端；A* 与 SMAC 均输出离散引导路径，供后端局部优化使用。

### 3.2 基于前瞻距离的局部裁剪（运行期）

触发点：`MincoFSM` 的 20Hz 主循环在 `FOLLOW_TRAJ` 中按需触发 `MincoPlanner::ReplanLocal()`。
（异步安全检测：`MincoPlanner::safetyTimerCallback()` 以 20Hz 检查当前已提交轨迹是否与 costmap 冲突。）

1) 获取当前机器人位姿（`costmap_ros_->getRobotPose`）。
2) 在 `latest_global_path_` 上搜索距当前位置最近的点作为起点索引。
3) 从该索引向前累积距离，直到达到 `lookahead_dist`，得到局部“稠密路径段”。

### 3.3 梯形加减速采样 + 贪心射线检测碰撞 + 插值修复（稀疏化）

目标：把局部稠密路径转成较少但“可直连/更适合优化”的稀疏节点序列。

1) **弧长累计**
- 计算局部路径点的累计弧长 `accumulated_dist`，得到总长度 `L`。

2) **梯形/三角速度模型（整体思路）**
- 依据参考速度 `v_ref = 0.8 * max_vel` 与参考加速度 `a_ref = max_acc`：
  - 若 `L` 足够长：采用梯形速度曲线（加速-匀速-减速）；
  - 否则：退化为三角速度曲线（加速-减速）。
- 根据总时间 `t_total`，以固定时间间隔对 `t` 采样，计算 `s(t)`（弧长位置），再映射回原始路径索引。

3) **贪心射线检测（isLineFree / Line-of-Sight）**
- 对每一段候选“直连边”做 costmap 直线离散采样：
  - 以 costmap 分辨率为步长采样线段；
  - 任一点落入 `INSCRIBED_INFLATED_OBSTACLE`（及以上）即判定不可直连。

4) **碰撞修复（插入角点 / Corner Insertion）**
- 若当前安全点到目标点不可直连：
  - 在区间内寻找“偏离直线最大”的点作为角点（近似提取转角/绕障节点）；
  - 插入角点后继续尝试直连，直到修复成功或达到迭代上限。

最终输出：稀疏节点 `sparse_path`（含起点与终点）。

### 3.3.1 走廊生成（PolyhedronH）

为了把几何可行域显式注入优化器，系统会在稀疏路径附近构建局部安全走廊：

1) 以 `HybridESDFMap` 查询局部空间可用距离，生成轴向对齐包围盒（AABB）。
2) 扣除机器人几何尺寸：
  - 半径项 `robot_radius_`；
  - 额外鲁棒余量 `extra_margin`（应对建图噪声与定位漂移）。
3) 将包围盒转换为 `PolyhedronH` 半空间表达，作为优化约束输入。
4) 对退化盒施加数值下界 `kMinHalfSize=0.05m`，防止约束矩阵病态导致优化发散。

### 3.4 MINCO 轨迹优化（核心求解）

1) **规划状态机（HOT/COLD/EMERGENCY）**
- 若无历史轨迹：COLD_START（速度/加速度置 0）；
- 若有历史轨迹：
  - 时间 t 合法（在上一次轨迹持续时间内）；
  - 位置误差不超过阈值（>0.5m 触发 EMERGENCY_STOP）；
  - 速度方向与新路径初段夹角不过大（点积 <0.9 则退回 COLD_START）。
- HOT_START：用上一次轨迹在 t 时刻的 P/V/A 作为新的起始状态。

2) **终端状态设置（靠近全局目标则收敛停车）**
- 计算稀疏终点与全局目标距离：
  - 若距离 > 1m：终端速度沿末段切向给定 `0.8*max_vel`（鼓励持续前进）；
  - 否则：终端速度与加速度置 0（停车/收敛）。

3) **MINCO 优化变量与求解器**
- 优化器：`MincoOptimizer`（内部使用 `lbfgs`）。
- 变量：
  - 时间变量（通过 `tau -> T` 的指数映射保证 `T>0`）；
  - 中间控制点（稀疏路径中除首尾外的点）。

4) **代价项（costFunctional）概述**
总代价由两部分组成：

- **MINCO 内部能量项（平滑度）**
  - 由 `MINCO_S3NU` 提供 `getEnergy()` 与对系数/时间的偏导；

- **约束/惩罚项（constraintsFunctional）**（按分段、按采样积分）
  - `Pos`：基于 ESDF 的安全距离惩罚（距离小于 `safe_dist` 时增长，使用 `smooth_eps` 平滑）；
  - `Vel`：速度上界惩罚（超过 `max_vel` 时增长）；
  - `Acc`：加速度上界惩罚（超过 `max_acc` 时增长）；
  - `Attract`：轨迹对“引导节点/航点”的吸引项（鼓励贴合稀疏路径）。

- **时间正则项**
  - `rho * sum(T)`：鼓励更短的总时长（避免无限慢）。

5) **输出**
- 优化得到的轨迹会以固定步长采样为 `ros_interfaces::msg::MpcPositionCommand`，发布到 `/opt_path`。

### 3.4.1 优化器工程实现细节（源码对应）

- 入口：`MincoOptimizer::optimize()`。
- 变量：`x=[tau, xi]`，通过 `forwardMapTauToT()` 保证分段时间 `T>0`。
- 代价函数：`costFunctional()` = 能量项 + 约束项 + 时间正则 + 运动学时间屏障。
- 约束项计算：`constraintsFunctional()` 对每段按 `integral_res` 采样积分，分别累积
  `Pos/Vel/Acc/Attract` 四项罚函数及梯度。
- 时间屏障：`computeTimeBarrier()` 在末段和高动态段抬升最小时长下界，避免时间被优化器压缩到不可实现区间。

### 3.5 备份轨迹（安全兜底）

即使主优化成功，也会每周期先构造并发布一条安全备份轨迹：

1) 从 ESDF 在当前位置估计安全距离，构造轴对齐“安全盒”（SFC）。
2) 在安全盒内，沿当前速度方向生成一条“前向减速到停”的五次多项式轨迹候选（多组 T 候选）。
3) 若采样点全部落在安全盒内则接受；否则回退为原地短停轨迹。
4) 发布到 `/backup_path`（command_flag = BLOCK）。

### 3.6 FSM 运行机制（与实车恢复逻辑）

`MincoFsm::callMainFsmOnce()` 以主循环驱动状态切换，关键行为与源码一致：

- `GENERATE_TRAJ`：必须先 `PlanGlobalPath` 再 `ReplanLocal`，任一失败直接切 `EMER_STOP`。
- `FOLLOW_TRAJ`：
  - 判定轨迹超时 / 不安全即触发重规划；
  - 另有 10Hz 强制刷新（`>0.1s`）机制，持续上线新轨迹。
- `EMER_STOP`：
  - 首次进入发布独立刹停轨迹；
  - 支持“恢复优先”：若可重规划成功则回 `FOLLOW_TRAJ`；
  - 支持“超时保护”：超过 5s 直接回 `WAIT_GOAL` 避免死锁；
  - 支持“速度门控恢复”：仅当真实底盘速度 `<0.1m/s` 才可通过 `EMER_SAFE` 回到可接单状态。

---

## 4. Visualizer 简介（可视化模块）

`Visualizer` 负责把规划过程中的关键中间量以 RViz 友好的形式发布出来，便于调参和定位问题。

已提供话题（默认）：
- `/astar_path_vis`：A* 离散路径（nav_msgs/Path，transient_local）。
- `/opt_path_vis`：优化轨迹采样成的 Path（nav_msgs/Path）。
- `/backup_path_vis`：备份轨迹采样成的 Path（nav_msgs/Path）。
- `/minco_control_points_vis`：控制点/稀疏节点（Marker，SPHERE_LIST，transient_local）。
- `/esdf_cloud`：ESDF 点云（PointCloud2，仅在成功加载静态 ESDF 且订阅者存在时，按 1Hz 发布）。

实现要点：
- 高频路径可视化与指令发布解耦，避免 RViz 订阅阻塞控制链路；
- ESDF 点云采用订阅触发发布，降低无效算力消耗。

---

## 5. 参数配置说明

所有参数均以 Nav2 插件名称为前缀（`<planner_name>.*`）。下面以 `<planner_name> = minco_planner` 举例。

### 5.1 全局规划/运行期

| 参数 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `tolerance` | double | 0.5 | Nav2 目标容忍半径（当前实现主要用于 goal 校验逻辑）。 |
| `use_astar` | bool | true | 预留开关（当前 `makePlan()` 仍固定走 A*）。 |
| `allow_unknown` | bool | true | A* 是否允许 UNKNOWN 栅格（255）。 |
| `use_smac` | bool | false | 是否启用 SmacPlanner2DSimple 前端（Costmap+ESDF 双代价）。 |
| `minco_optimizer.opt_freq` | double | 20.0 | 运行期优化定时器频率（Hz）。 |
| `minco_optimizer.lookahead_dist` | double | 5.0 | 从机器人当前位置向前截取的前瞻距离（米）。 |
| `minco_optimizer.traj_goal_tolerance` | double | 0.5 | 局部轨迹目标收敛容差。 |

### 5.2 静态 ESDF(static_esdf)

| 参数 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `esdf_pcd_path` | string | `src/utils/pcd2esdf/maps/2026_esdf.pcd` | 静态 ESDF PCD 文件路径。 |
| `esdf_resolution` | double | 0.05 | ESDF 栅格分辨率（米）。 |

### 5.3 MINCO 优化器(minco_optimizer)

| 参数 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `.safe_dist` | double | 0.3 | ESDF 安全距离阈值（米）。 |
| `max_vel` | double | 2.0 | 速度上界（m/s）。 |
| `max_acc` | double | 4.0 | 加速度上界（m/s²）。 |
| `time_allocation_iters` | int | 15 | 预留：时间分配迭代次数（当前实现中以 L-BFGS 优化为主）。 |
| `penalty_weight_time` | double | 0.01 | 时间正则权重 `rho`（越大越倾向更短时长）。 |
| `smooth_eps` | double | 0.01 | ESDF 惩罚平滑参数（避免不可导）。 |
| `integral_res` | int | 16 | 每段轨迹的积分采样分辨率（越大越精细但更耗时）。 |
| `opt_accuracy` | double | 1e-4 | L-BFGS 梯度收敛阈值。 |
| `print_optimizer_log` | bool | true | 输出优化过程日志（代价分解、迭代次数等）。 |
| `penalty_weight_pos` | double | 1000.0 | 位置/安全距离惩罚权重。 |
| `penalty_weight_vel` | double | 1000.0 | 速度超限惩罚权重。 |
| `penalty_weight_acc` | double | 10000.0 | 加速度超限惩罚权重。 |
| `penalty_weight_att` | double | 1000.0 | 吸引项（贴合引导路径）权重。 |

### 5.4 SMAC 前端（可选）

| 参数 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `smac_2d.use_esdf_cost` | bool | false | 是否启用 ESDF 势场代价偏置。 |
| `smac_2d.esdf_weight` | double | 1.0 | ESDF 势场权重。 |
| `smac_2d.esdf_decay` | double | 0.5 | 势场距离衰减系数。 |
| `smac_2d.esdf_max_cost` | double | 5.0 | 单栅格 ESDF 附加代价上限。 |

### 5.5 走廊参数（corridor）

| 参数 | 类型 | 默认值 | 说明 |
|---|---:|---:|---|
| `corridor.robot_radius` | double | 0.4 | 机器人半径，构建可行走廊时扣除。 |
| `corridor.extra_margin` | double | 0.15 | 额外安全余量，提升实车鲁棒性。 |

---

## 6. 依赖与安装

### 6.1 主要依赖

- ROS 2 + Nav2：`nav2_core`, `nav2_costmap_2d`, `nav2_util`, `pluginlib`, `tf2_ros` 等。
- 点云库：PCL（用于包内相关模块链接）。
- Eigen3。
- 优化与搜索：`lbfgs`、SMAC 2D 简化实现、Hybrid ESDF（静态层+动态层融合）。

此外，本包在 `include/` 内部包含（或封装）了一些工具库/头文件（如 fmt/cereal/lbfgs 等），减少外部依赖。

### 6.2 编译

在工作区根目录使用 colcon 构建：

```bash
colcon build --packages-select minco_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

运行前加载环境：

```bash
source install/setup.bash
```

### 6.3 Nav2 配置示例（片段）

> 以下为示意，具体请根据你的 Nav2 配置文件/启动文件调整插件名与命名空间。

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "minco_planner/MincoPlanner"
      tolerance: 0.5
      allow_unknown: true
      minco_optimizer:
        opt_freq: 20.0
        lookahead_dist: 5.0
        safe_dist: 0.3
        max_velocity: 2.0
        max_acceleration: 4.0
        penalty_weight_time: 0.01
        penalty_weight_pos: 1000.0
        penalty_weight_vel: 1000.0
        penalty_weight_acc: 10000.0
        penalty_weight_att: 1000.0
      static_esdf:
        esdf_pcd_path: "src/utils/pcd2esdf/maps/2026_esdf.pcd"
        esdf_resolution: 0.1
```


---

## 7. 参考文献

- [MINCO](https://arxiv.org/abs/2103.00190)-Geometrically Constrained Trajectory Optimization for Multicopters
- [中国科学技术大学（USTC）2025 哨兵技术报告](https://bbs.robomaster.com/article/803727?source=4)
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) – A valuable resource that efficiently performs differentiable trajectory optimization and serves as the foundation of our trajectory optimization method.
- [SUPER](https://github.com/hku-mars/SUPER): Safety-assured High-speed Navigation for MAVs

---

## 8. 版权与交流方式

- 版权/许可：请以仓库根目录 LICENSE 为准；本包内 `include/` 可能包含第三方库的 LICENSE 文件，请一并遵循。
- 交流与反馈：
  - Maintainer: alioth
  - Email: 15207309998@163.com
  - QQ:2914335251

使用注意：
- 本包默认输出 `/opt_path` 与 `/backup_path` 的 `ros_interfaces::msg::MpcPositionCommand`，请确保下游控制器/桥接节点订阅并理解字段含义。
- ESDF 静态地图加载失败时仍可运行，但安全相关能力会受限（例如备份轨迹安全盒、SMAC ESDF 势场偏置与可视化）。
- 实车建议开启里程计与速度监测冗余链路，确保 `EMER_STOP` 的速度门控条件可靠触发。
