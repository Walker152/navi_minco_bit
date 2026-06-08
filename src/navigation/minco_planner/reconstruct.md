# Codex 任务：重写 MincoPlanner 双模式接口架构，不改变算法内部原理

## 0. 总目标

对当前 `MincoPlanner` 的 planner 架构做一次接口级重写，支持两个互斥规划模式：

```cpp
enum class PlannerMode {
  PRIORMAP,
  EXPLORATION
};
```

重点要求：

1. 不修改 Minco 优化器、A*/Smac 搜索、轨迹生成、恢复策略等算法内部原理。
2. 只修改 planner 的模式初始化、地图接口传入、坐标系转换、路径输出 frame、ROGMap 查询方式。
3. 不做过度封装。允许增加少量必要的轻量 adapter / helper，但不要重构成复杂框架。
4. 模式基本不会运行时切换，因此在 `configure()` 阶段读取参数并完成 mode init，之后主流程尽量使用初始化好的变量、函数指针或接口对象，不要在每个阶段到处散落 `if (mode == ...)`。
5. `PRIORMAP` 模式必须保持原有有先验地图导航语义，不允许破坏现有 Nav2 / BT / controller 适配。
6. `EXPLORATION` 模式完全使用 `camera_init` / ROGMap 语义，不依赖 PGM / map / 静态 ESDF。

---

## 1. 模式语义定义

### 1.1 PRIORMAP 模式

这是有先验 PGM / 固定比赛地图的模式。

语义必须保持：

```text
planning_frame = map
output_frame   = map
odom source    = /aft_mapped_to_init, but pose must be transformed to map before planning
global search  = Nav2 global costmap / existing static map
path sparsify  = existing map-frame logic
control points = existing map-frame logic
static ESDF    = enabled, existing logic
ROGMap frame   = camera_init
ROGMap role    = dynamic gradient provider + safety grid only
/opt_path      = map frame
```

特别强调：

* `PRIORMAP` 下，ROGMap 不能参与全局搜索。
* `PRIORMAP` 下，ROGMap 不能改变 BT、Nav2、controller 的 map 语义。
* `PRIORMAP` 下，全局搜索、路径稀疏、控制点选择应尽量保持当前有 PGM 时的逻辑。
* ROGMap 只允许用于：

  1. Minco 优化中的动态障碍距离场 / 梯度查询；
  2. 轨迹安全检测中的动态栅格 / ESDF 查询；
  3. 必要时作为局部优化窗口边界裁剪依据。
* 不允许再出现：把 `map` frame 的 start/goal 直接传入 raw ROGMap 的 `worldToMap()`。

### 1.2 EXPLORATION 模式

这是无先验 PGM、自主探索导航模式。

语义为：

```text
planning_frame = camera_init
output_frame   = camera_init
odom source    = /aft_mapped_to_init, used directly
global search  = ROGMap A*/Dijkstra to reachable boundary
path sparsify  = camera_init frame
control points = camera_init frame
static ESDF    = disabled
ROGMap frame   = camera_init
ROGMap role    = search map + optimization dynamic map + safety grid
/opt_path      = camera_init frame
```

特别强调：

* `EXPLORATION` 下不要强行使用 `map`。
* `EXPLORATION` 下不要加载或使用静态 ESDF PCD。
* `EXPLORATION` 下当前订阅 odom 本来就在 `camera_init`，`getRobotPose()` 应直接返回 odom pose，并保证 header frame 是 `camera_init`。
* `EXPLORATION` 下搜索目标如果在 ROGMap 内，可以搜索到目标；如果目标在 ROGMap 外，不要直接把目标投影到边界后当作终点，因为这个边界点可能不可达。
* 正确策略是：从当前位置出发搜索，终止条件是到达“可达的边界候选点”。目标方向只作为边界候选点评分或 A* heuristic 的偏置。

---

## 2. 参数设计

在 `MincoPlanner` 参数下新增：

```yaml
MincoPlanner:
  planner_mode: PRIORMAP   # PRIORMAP | EXPLORATION

  frames:
    map_frame: map
    rog_frame: camera_init

  priormap:
    use_nav2_global_search: true
    clip_seed_by_rog_boundary: true
    rog_boundary_margin: 0.8
    rog_boundary_sample_step: 0.1

  exploration:
    boundary_margin: 0.8
    boundary_sample_step: 0.1
    unknown_as_occupied: true
    prefer_goal_direction: true
```

要求：

* `planner_mode` 只在 `configure()` 中读取。
* `planner_mode` 不允许 hot reload。若 `onSetParameters()` 收到该参数变化，直接 reject，并提示需要重启节点。
* 默认模式必须是 `PRIORMAP`，防止破坏现有比赛导航系统。
* `map_frame` 默认 `map`。
* `rog_frame` 默认 `camera_init`。

---

## 3. configure() / modeInit() 改造

新增一个轻量结构保存模式初始化结果：

```cpp
struct PlannerRuntimeModeConfig {
  PlannerMode mode;
  std::string planning_frame;
  std::string output_frame;
  std::string map_frame;
  std::string rog_frame;

  bool use_nav2_global_search;
  bool use_rog_global_search;
  bool use_static_esdf;
  bool use_frame_aware_rog_query;
  bool direct_odom_pose;
};
```

在 `configure()` 中按如下顺序做：

1. 读取 `planner_mode`。
2. 读取 `frames.map_frame` 和 `frames.rog_frame`。
3. 创建 ROGMap，保存 raw ROGMap 查询接口。
4. 调用 `initPlannerMode()`，一次性设置：

   * `planning_frame_`
   * `output_frame_`
   * `global_frame_`
   * `use_static_esdf_`
   * `global_search_source_`
   * 用于全局搜索的 map/query 对象
   * 用于动态约束的 map/query 对象
   * `getRobotPose()` 的主语义
5. 初始化 global planner / astar / smac / optimizer / corridor 时，传入 mode init 后确定好的对象，而不是默认全部传 raw ROGMap。

注意：

* 当前 `setMap(map_)` 会把同一个 ROGMap 注入 smac、astar、optimizer、corridor。这个行为需要拆开。
* 不要再用一个 `map_` 同时表示“全局搜索地图”和“动态障碍查询地图”。
* 可以保留 `map_` 名称作为 dynamic query map，但要明确 raw ROGMap 与 frame-aware ROGMap 的区别。
* 建议新增：

  * `rog_query_raw_`
  * `global_search_query_`
  * `dynamic_query_`
* 若为了减少改动量，可让 `dynamic_query_` 继续使用 `std::shared_ptr<rog_map::MapQueryInterface>` 类型，但 PRIORMAP 下必须是 frame-aware wrapper，不是 raw ROGMap。

---

## 4. 轻量 adapter / helper 要求

允许新增少量必要 helper，但不要过度工程化。

### 4.1 FrameAwareRogQuery

PRIORMAP 模式需要一个轻量 wrapper：

```text
输入坐标：map
内部转换：map -> camera_init
查询对象：raw ROGMap
输出距离：保持距离标量
输出梯度：camera_init 梯度旋转回 map
```

它用于：

* `minco_optimizer_->setMap(...)`
* `corridor_gen_->setMap(...)`
* `checkCollision()`
* `getEsdfDistance()`
* recovery 的距离查询

注意：

* 不改 Minco 优化器内部数学。
* 不改轨迹优化代价函数原理。
* 只让 optimizer 看到一个“planning_frame 下可查询的动态地图接口”。

### 4.2 DirectRogQuery

EXPLORATION 模式下可直接使用 raw ROGMap，不需要 frame 转换。

### 4.3 Nav2Costmap Search Adapter

PRIORMAP 模式下全局搜索必须使用 Nav2 global costmap / 现有地图，不使用 ROGMap。

可以选择以下两种最小改造方式之一：

方案 A：为现有 A*/Smac 搜索增加一个轻量 Nav2 costmap adapter，使其访问 `nav2_costmap_2d::Costmap2D`。

方案 B：新增 `makePlanOnNav2Costmap()`，直接使用 costmap 的 `worldToMap() / getCost()` 做搜索，不修改搜索算法原理，只替换地图读取来源。

不要改 A*/Smac 的启发函数、代价累计、邻接扩展等算法本质。

---

## 5. PlanGlobalPath() 改造

将 `PlanGlobalPath()` 改成调用 configure 时绑定好的全局搜索实现。

推荐方式：

```cpp
using PlanGlobalFn = std::function<bool(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)>;

PlanGlobalFn plan_global_fn_;
```

在 `initPlannerMode()` 里：

```cpp
if (mode == PlannerMode::PRIORMAP) {
  plan_global_fn_ = [this](auto & start, auto & goal) {
    return planGlobalPathPriorMap(start, goal);
  };
} else {
  plan_global_fn_ = [this](auto & start, auto & goal) {
    return planGlobalPathExploration(start, goal);
  };
}
```

然后：

```cpp
bool MincoPlanner::PlanGlobalPath(...) {
  return plan_global_fn_(start, goal);
}
```

这样主流程不需要到处判断 mode。

### 5.1 PRIORMAP: planGlobalPathPriorMap()

要求：

1. start 和 goal 都转换到 `map_frame_`。
2. 使用 Nav2 global costmap / 原先 map 搜索逻辑。
3. 不使用 raw ROGMap 的 `worldToMap()`。
4. 输出 `latest_global_path_`，其 `header.frame_id = map`。
5. 若 goal frame 为空，按 `map` 处理并打 WARN。
6. 若 transform 失败，返回 false。
7. 若 costmap worldToMap 失败，打印明确日志：是 Nav2 costmap 坐标转换失败，而不是 ROGMap 失败。

### 5.2 EXPLORATION: planGlobalPathExploration()

要求：

1. start 和 goal 都转换或归一到 `camera_init`。
2. 若 goal 在 ROGMap 边界内且 cell 可通行，则搜索到 goal。
3. 若 goal 不在 ROGMap 边界内，则从 start 出发在 ROGMap 上搜索到“可达边界候选点”。
4. 不要直接把 goal 投影到边界后当终点，因为投影点可能不可达。
5. 搜索到边界的策略：

   * 从 start cell 出发做 A*/Dijkstra。
   * 对扩展到的 cell 判断是否为边界候选：

     * 距离 ROGMap 边界小于 `exploration.boundary_margin`；
     * cell 非 lethal；
     * 根据参数决定 unknown 是否可通过；
     * cell 属于从 start 可达的连通域。
   * 若有外部 goal，则边界候选评分优先选择“朝向 goal 方向最一致、路径代价合理”的候选。
   * 若没有明确 goal，则选择路径代价合理且靠近边界的候选。
6. 重建路径，输出 `latest_global_path_`，其 `header.frame_id = camera_init`。
7. 如果起点不在 ROGMap 内，返回 false 并打印明确错误。
8. 如果没有可达边界候选，返回 false 触发 recovery。

---

## 6. getRobotPose() 改造

将 `getRobotPose()` 的语义也在 mode init 中确定，避免主流程到处判断。

### PRIORMAP

* 优先使用 `costmap_ros_->getRobotPose(pose)`，保持现有 Nav2 语义。
* 若失败，使用 latest odom，经 TF 转换到 `map_frame_`。
* 输出 pose 的 `header.frame_id` 必须是 `map`。

### EXPLORATION

* 不使用 costmap pose。
* 直接读取 `latest_odom_`。
* 输出 pose 的 `header.frame_id` 必须是 `camera_init`。
* 若 odom header frame 为空，按 `camera_init` 处理并打 WARN。
* 不要把 odom 转到 `map`。

---

## 7. ReplanLocal() 改造

`ReplanLocal()` 尽量保持原算法流程，只把输入路径 frame 和地图查询接口改正确。

统一语义：

```text
ReplanLocal 输入 current_pose 已经在 planning_frame。
latest_global_path_ 已经在 planning_frame。
局部路径截取、稀疏、控制点选择都在 planning_frame。
Minco 优化也在 planning_frame。
输出轨迹 header 使用 output_frame。
```

### PRIORMAP

* planning_frame = map。
* 从 map frame 的 `latest_global_path_` 截取局部段。
* 原有路径稀疏 / 控制点选择逻辑尽量不动。
* 如果启用 `priormap.clip_seed_by_rog_boundary`，则只做边界裁剪：

  * 将候选路径点从 map 转到 camera_init。
  * 检查是否在 ROGMap 窗口内，并保留 `rog_boundary_margin`。
  * 超出边界则截断 local seed。
* 不要使用 ROGMap 做第二次局部离散搜索。
* 不要使用 ROGMap 改变原有 Nav2 全局路径语义。
* ROGMap 只通过 `dynamic_query_` 影响后续优化和安全检测。

### EXPLORATION

* planning_frame = camera_init。
* 从 camera_init frame 的 `latest_global_path_` 截取局部段。
* 可以同样做 ROGMap 边界裁剪，但此时是 identity 查询。
* Minco 优化在 camera_init。
* 输出 `/opt_path` 为 camera_init。

---

## 8. checkCollision() / getEsdfDistance() / recovery 查询改造

当前安全检测不能直接用 raw ROGMap 查询规划坐标。

要求：

1. `checkCollision()` 统一使用 `dynamic_query_`。
2. `checkCollision(const Trajectory &)` 统一使用 `dynamic_query_`。
3. `getEsdfDistance()` 统一使用 `dynamic_query_`。
4. recovery_server 回调中的 ESDF 查询使用 `getEsdfDistance()` 即可。
5. 若 `dynamic_query_` 不可用，返回 unsafe，并打印限频错误。
6. PRIORMAP 下，dynamic_query 内部负责 map -> camera_init 转换。
7. EXPLORATION 下，dynamic_query 直接查询 raw ROGMap。

安全检测逻辑本身不要改：

```text
采样轨迹点
worldToMap
读取栅格 cost
evaluate ESDF
距离小于阈值判 unsafe
```

只改查询接口和坐标语义。

---

## 9. 静态 ESDF 加载改造

当前静态 ESDF 加载逻辑只允许 PRIORMAP 使用。

要求：

### PRIORMAP

* 保留现有 static ESDF 加载。
* visualizer 继续使用 map frame。
* optimizer 保持 static ESDF + dynamic ROGMap 的组合。

### EXPLORATION

* 不加载 static ESDF PCD。
* `esdf_map_` 可以置空。
* visualizer 不应依赖 static ESDF。
* optimizer 只使用 ROGMap dynamic query。
* 如果某些模块必须要 `esdf_map_`，请做空指针保护，不要引入假地图。

---

## 10. 输出 frame 改造

所有轨迹发布必须使用 `output_frame_`：

* `/opt_path`
* `/backup_path`
* escape command
* emergency stop / backup trajectory
* debug path if relevant

要求：

```text
PRIORMAP:
  output_frame = map

EXPLORATION:
  output_frame = camera_init
```

不要继续散落使用 `global_frame_` 来代表输出 frame，除非 `global_frame_` 已经在 mode init 中被明确设为 `output_frame_`。

---

## 11. createPlan() 保持兼容

`createPlan()` 当前只缓存 goal 并返回 minimal path 的设计可以保留。

要求：

* PRIORMAP 下，goal 语义保持 map，不要改变 BT / Nav2 行为。
* EXPLORATION 下，goal 需要归一到 camera_init；如果 goal frame 为空，按 camera_init 处理并 WARN。
* 返回的 minimal path header 使用 `output_frame_`。
* 不要让 createPlan 直接访问 raw ROGMap 做全局 start/goal 转换。

---

## 12. FSM 尽量不动

`minco_fsm.cpp` 主流程尽量不改：

```text
consumePendingGoal
getRobotPose
PlanGlobalPath
ReplanLocal
FOLLOW_TRAJ
RECOVERING
```

只要 `MincoPlanner` 内部保证：

* `getRobotPose()` 返回 mode 对应 frame；
* `PlanGlobalPath()` 接受并输出 mode 对应 frame；
* `ReplanLocal()` 使用 mode 对应 frame；
* `getEsdfDistance()` 使用 dynamic_query；

则 FSM 不需要知道 PRIORMAP / EXPLORATION 细节。

---

## 13. 禁止事项

绝对不要做以下改动：

1. 不要改变 Minco 优化器核心数学公式。
2. 不要改变轨迹时间分配、惩罚项、动力学约束原理。
3. 不要改变 A*/Smac 的搜索算法原理。
4. 不要让 PRIORMAP 模式使用 ROGMap 做全局搜索。
5. 不要让 PRIORMAP 模式输出 camera_init frame 的 `/opt_path`。
6. 不要让 EXPLORATION 模式依赖 map / PGM / static ESDF。
7. 不要在主流程各处散落大量 `if (mode == ...)`。
8. 不要为了“统一”而破坏当前有先验地图导航系统的语义。
9. 不要把外部目标直接投影到 ROGMap 边界并假设可达。
10. 不要删除 recovery / safety timer / FSM 现有状态机逻辑。

---

## 14. 需要修改/检查的文件

重点检查：

```text
minco_core/minco_planner.hpp
minco_core/minco_planner.cpp
minco_core/minco_fsm.cpp
minco_core/minco_utils.cpp
smac_search/smac_planner_2d_simple.hpp/.cpp
相关 CMakeLists/package.xml
sentry1.yaml 或对应 planner yaml
```

如果为了 lightweight adapter 需要新增文件，建议最多新增：

```text
minco_core/planner_mode_utils.hpp
minco_core/planner_mode_utils.cpp
```

或直接把小 helper 写在 `minco_planner.cpp` 内部匿名 namespace，优先减少文件扩散。

---

## 15. 日志要求

启动时打印一次模式摘要：

```text
[MincoPlanner] planner_mode=PRIORMAP
[MincoPlanner] planning_frame=map output_frame=map rog_frame=camera_init
[MincoPlanner] global_search=Nav2Costmap dynamic_query=FrameAwareRogQuery static_esdf=enabled
```

或：

```text
[MincoPlanner] planner_mode=EXPLORATION
[MincoPlanner] planning_frame=camera_init output_frame=camera_init rog_frame=camera_init
[MincoPlanner] global_search=ROGMapBoundaryAstar dynamic_query=DirectRogQuery static_esdf=disabled
```

错误日志必须明确区分：

* Nav2 costmap worldToMap 失败；
* ROGMap boundary check 失败；
* TF map -> camera_init 失败；
* ROGMap dynamic query 不可用；
* EXPLORATION 找不到可达边界。

---

## 16. 验证要求

至少完成以下静态验证：

1. `colcon build` 通过。
2. `grep` 确认 PRIORMAP 全局搜索不调用 raw ROGMap `worldToMap(start/goal)`。
3. `grep` 确认 `checkCollision()` 和 `getEsdfDistance()` 不直接查 raw ROGMap，而是查 mode init 后的 dynamic query。
4. `grep` 确认 `/opt_path` header 使用 `output_frame_`。
5. `planner_mode` hot reload 被拒绝。

运行验证建议输出 `planner_mode_validation.md`，包含：

### PRIORMAP 验证步骤

```bash
ros2 param get /planner_server MincoPlanner.planner_mode
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /opt_path --once
```

预期：

```text
planner_mode = PRIORMAP
/opt_path.header.frame_id = map
全局搜索使用 Nav2 costmap
ROGMap 只在优化和安全检测中被查询
不会再出现 failed to convert start world coordinates 到 ROGMap map coordinates
```

### EXPLORATION 验证步骤

```bash
ros2 param get /planner_server MincoPlanner.planner_mode
ros2 topic echo /aft_mapped_to_init --once
ros2 topic echo /opt_path --once
```

预期：

```text
planner_mode = EXPLORATION
/opt_path.header.frame_id = camera_init
getRobotPose 直接使用 odom
不加载 static ESDF
全局搜索在 ROGMap 内从当前位置搜索到可达边界
```

---

## 17. 交付物

完成后输出：

1. 修改过的文件列表。
2. 每个文件的修改摘要。
3. PRIORMAP 模式下语义保持说明。
4. EXPLORATION 模式下边界搜索说明。
5. 编译结果。
6. 仍需实机验证的项目。
7. `planner_mode_validation.md`。
