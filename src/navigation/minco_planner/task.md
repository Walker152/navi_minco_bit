# Codex 任务：重构 MincoPlanner 代码组织，抽离 Query / Mode / Search / Safety 职责，但不要编译

## 0. 重要限制

本任务只做代码结构重构，不做算法重写。

严禁执行：

```bash
colcon build
colcon test
ros2 launch
ros2 run
```

不要编译，不要运行实机相关命令。
只允许做静态代码修改、静态自查、grep 检查、生成说明文档。

如果需要验证，只输出“建议编译命令”，不要实际执行。

---

## 1. 当前问题

当前 `minco_planner.cpp` 过于臃肿，承担了大量不属于 `MincoPlanner` 本体的职责，包括但不限于：

```text
1. Nav2CostmapQuery / FrameAwareRogQuery 等 query adapter 定义；
2. TF 点变换、向量旋转、cost 判断等工具函数；
3. PRIORMAP / EXPLORATION 模式下 query 创建与绑定；
4. 全局搜索实现；
5. EXPLORATION reachable boundary search；
6. 局部路径截取、ROGMap 边界裁剪、路径稀疏；
7. 轨迹安全检测、ESDF 距离查询；
8. 将各类 query 注入 Smac / Astar / Optimizer / Corridor。
```

重构目标是让 `MincoPlanner` 回到高层调度器职责：

```text
MincoPlanner 只负责：
1. Nav2 planner plugin 生命周期；
2. 参数加载；
3. PRIORMAP / EXPLORATION 模式适配；
4. 调度全局搜索、局部路径处理、Minco 优化、安全检测；
5. 发布 /opt_path / backup / recovery command；
6. 与 FSM、Nav2、ROGMapROS 的连接。
```

不要让 `MincoPlanner` 继续直接实现 query adapter、搜索细节、边界搜索细节、安全检测细节。

---

## 2. 核心原则

必须保持现有行为不变。

尤其保持双模式语义：

### PRIORMAP

```text
planning_frame = map
output_frame   = map
global search  = Nav2 costmap / prior map
sparsify       = Nav2 costmap / prior map
dynamic query  = FrameAwareRogQuery(raw ROGMap, map -> camera_init)
static ESDF    = enabled
/opt_path      = map
```

ROGMap 在 PRIORMAP 下只允许用于：

```text
1. Minco 优化动态梯度；
2. 轨迹安全检测；
3. 可选 seed path 边界裁剪。
```

禁止让 ROGMap 参与 PRIORMAP 的全局搜索、路径稀疏、控制点选择。

### EXPLORATION

```text
planning_frame = camera_init
output_frame   = camera_init
global search  = ROGMap reachable boundary search
sparsify       = ROGMap
dynamic query  = raw ROGMap
static ESDF    = disabled
/opt_path      = camera_init
```

EXPLORATION 下搜索边界点必须是从当前位置出发搜索到的 reachable boundary candidate，不允许直接把目标投影到边界后假定可达。

---

## 3. 禁止修改算法内部原理

不要修改以下内容的算法原理：

```text
1. MincoOptimizer 代价函数、时间分配、约束公式；
2. A* / Smac 的搜索启发、代价累计、邻接扩展；
3. FSM 状态机主流程；
4. recovery 决策逻辑；
5. yaw optimizer / backup optimizer；
6. controller 输出消息结构。
```

允许的修改仅限于：

```text
1. 类和文件拆分；
2. query adapter 抽离；
3. 模式上下文抽离；
4. 将已有逻辑搬到更合适的类中；
5. 统一接口命名；
6. 减少 MincoPlanner 主文件职责；
7. 改善内存管理，但不改变算法行为。
```

---

## 4. 推荐新增文件

根据当前包结构，优先新增以下文件：

```text
minco_core/map_query_adapters.hpp
minco_core/map_query_adapters.cpp

minco_core/planner_mode_context.hpp
minco_core/planner_mode_context.cpp

minco_core/global_path_searcher.hpp
minco_core/global_path_searcher.cpp

minco_core/local_path_processor.hpp
minco_core/local_path_processor.cpp

minco_core/trajectory_safety_checker.hpp
minco_core/trajectory_safety_checker.cpp
```

如果项目已有更合适的 include/source 目录，请按现有风格放置。

同时更新：

```text
CMakeLists.txt
minco_core/minco_planner.hpp
minco_core/minco_planner.cpp
minco_core/minco_utils.hpp
minco_core/minco_utils.cpp
```

---

## 5. 第一阶段：抽离 map_query_adapters

从 `minco_planner.cpp` 中移走：

```cpp
transformPoint(...)
rotateVector(...)
isLethalCost(...)
class Nav2CostmapQuery
class FrameAwareRogQuery
```

放入：

```text
minco_core/map_query_adapters.hpp
minco_core/map_query_adapters.cpp
```

要求：

```cpp
namespace minco_planner {

class Nav2CostmapQuery : public rog_map::MapQueryInterface { ... };

class FrameAwareRogQuery : public rog_map::MapQueryInterface { ... };

}
```

`Nav2CostmapQuery` 职责：

```text
1. 适配 nav2_costmap_2d::Costmap2D；
2. 实现 rog_map::MapQueryInterface；
3. worldToMap / mapToWorld / size / origin / resolution / values / copyValues / isFree / evaluate；
4. 不持有 costmap 所有权，只保存裸指针或弱引用；
5. 不在高频函数中分配大对象。
```

`FrameAwareRogQuery` 职责：

```text
1. 输入 planning_frame 坐标；
2. 内部 TF 到 rog_frame；
3. 查询 raw ROGMap；
4. evaluate() 后将梯度从 rog_frame 旋转回 planning_frame；
5. PRIORMAP 下用于 dynamic query；
6. EXPLORATION 下不需要使用。
```

注意：

```text
1. 不要把 Nav2CostmapQuery 放进 rog_map 包，避免 ROGMap 反向依赖 Nav2。
2. FrameAwareRogQuery 可以保留现有每次 lookupTransform 的行为，本轮不强制做 TF cache。
3. 如果做 TF cache，必须保证行为等价，并说明刷新时机；否则不要做。
```

---

## 6. 第二阶段：抽离 PlannerModeContext

新增：

```text
minco_core/planner_mode_context.hpp
minco_core/planner_mode_context.cpp
```

建议定义：

```cpp
enum class PlannerMode {
  PRIORMAP,
  EXPLORATION
};

struct PlannerModeParams {
  std::string planner_mode{"PRIORMAP"};
  std::string map_frame{"map"};
  std::string rog_frame{"camera_init"};

  bool priormap_use_nav2_global_search{true};
  bool priormap_clip_seed_by_rog_boundary{true};
  double priormap_rog_boundary_margin{0.8};
  double priormap_rog_boundary_sample_step{0.1};

  double exploration_boundary_margin{0.8};
  double exploration_boundary_sample_step{0.1};
  bool exploration_unknown_as_occupied{true};
  bool exploration_prefer_goal_direction{true};
};

class PlannerModeContext {
public:
  void configure(
    const PlannerModeParams & params,
    const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger);

  void rebuildQueries(
    const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger);

  PlannerMode mode() const;
  const std::string & planningFrame() const;
  const std::string & outputFrame() const;
  const std::string & mapFrame() const;
  const std::string & rogFrame() const;

  bool useStaticEsdf() const;
  bool directOdomPose() const;

  bool clipSeedByRogBoundary() const;
  double rogBoundaryMargin() const;
  double rogBoundarySampleStep() const;

  double explorationBoundaryMargin() const;
  double explorationBoundarySampleStep() const;
  bool explorationUnknownAsOccupied() const;
  bool explorationPreferGoalDirection() const;

  std::shared_ptr<rog_map::MapQueryInterface> globalQuery() const;
  std::shared_ptr<rog_map::MapQueryInterface> dynamicQuery() const;
  std::shared_ptr<rog_map::MapQueryInterface> sparsifyQuery() const;
};
```

内部逻辑必须等价于当前 `initPlannerMode()` + `rebuildModeDependentQueries()`。

模式绑定要求：

```text
PRIORMAP:
  planningFrame = map_frame
  outputFrame   = map_frame
  useStaticEsdf = true
  directOdomPose = false
  globalQuery   = Nav2CostmapQuery
  dynamicQuery  = FrameAwareRogQuery(raw_rog_query)
  sparsifyQuery = globalQuery

EXPLORATION:
  planningFrame = rog_frame
  outputFrame   = rog_frame
  useStaticEsdf = false
  directOdomPose = true
  globalQuery   = raw_rog_query
  dynamicQuery  = raw_rog_query
  sparsifyQuery = raw_rog_query
```

`MincoPlanner` 中不再直接维护这些 query 的创建细节，只调用：

```cpp
mode_context_->globalQuery()
mode_context_->dynamicQuery()
mode_context_->sparsifyQuery()
```

---

## 7. 第三阶段：抽离 GlobalPathSearcher

新增：

```text
minco_core/global_path_searcher.hpp
minco_core/global_path_searcher.cpp
```

迁移当前 `MincoPlanner` 中的全局搜索相关逻辑，包括：

```text
1. makePlanOnQuery()
2. planGlobalPathPriorMap()
3. planGlobalPathExploration()
4. reachable boundary search
5. projectStartToFreeCell()
6. start/goal worldToMap 检查
7. SMAC / Astar 选择
8. path reconstruction / mapToWorld 输出
```

建议接口：

```cpp
class GlobalPathSearcher {
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::unique_ptr<Astar> astar,
    std::unique_ptr<smac::SmacPlanner2DSimple> smac,
    bool use_smac,
    bool allow_unknown,
    double tolerance);

  void setQueries(
    const std::shared_ptr<rog_map::MapQueryInterface> & global_query);

  bool plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    nav_msgs::msg::Path & out_path);
};
```

或者如果移动 `astar_ / smac_` 所有权太大，也可以先不移动所有权，只让 `GlobalPathSearcher` 持有非 owning 指针。优先保证重构安全。

要求：

```text
1. PRIORMAP 下使用 mode_context.globalQuery()，即 Nav2CostmapQuery；
2. EXPLORATION 下使用 mode_context.globalQuery()，即 raw ROGMap；
3. EXPLORATION 下 goal 在窗口内但不可达时，fallback 到 reachable boundary search；
4. 不改变搜索算法内部原理；
5. 不改变 allow_unknown / use_smac / tolerance 等原参数语义。
```

`MincoPlanner::PlanGlobalPath()` 最终应简化为：

```cpp
bool MincoPlanner::PlanGlobalPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!global_path_searcher_) {
    return false;
  }
  return global_path_searcher_->plan(start, goal, *mode_context_, latest_global_path_);
}
```

---

## 8. 第四阶段：抽离 LocalPathProcessor

新增：

```text
minco_core/local_path_processor.hpp
minco_core/local_path_processor.cpp
```

迁移当前 `MincoPlanner` 中的局部路径处理逻辑，包括：

```text
1. extractLocalPath()
2. clipLocalPathByRogBoundary()
3. ROGMap boundary margin 检查
4. path length / nearest index / lookahead 截取
5. getSparseWaypoints 调用
6. local waypoint/control point 生成前的路径预处理
```

建议接口：

```cpp
struct LocalPathSeed {
  bool valid{false};
  bool local_end_is_goal{false};
  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> sparse_waypoints;
  std::vector<double> local_magnitudes;
};

class LocalPathProcessor {
public:
  void configure(
    double lookahead_dist,
    double max_vel,
    double max_acc,
    rclcpp::Logger logger);

  LocalPathSeed buildSeed(
    const nav_msgs::msg::Path & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context);
};
```

要求：

```text
PRIORMAP:
  1. 截取 map frame global path；
  2. 稀疏使用 mode_context.sparsifyQuery()，也就是 Nav2CostmapQuery；
  3. 可选 ROGMap 边界裁剪使用 mode_context.dynamicQuery()；
  4. 不让 ROGMap 参与稀疏和控制点选择。

EXPLORATION:
  1. 截取 camera_init frame global path；
  2. 稀疏使用 ROGMap；
  3. 边界裁剪使用 ROGMap。
```

`MincoPlanner::ReplanLocal()` 中只保留高层逻辑：

```text
1. 调用 local_path_processor_->buildSeed(...)
2. 准备 start/end state
3. 调用 MincoOptimizer
4. 发布结果
```

不要在 `ReplanLocal()` 内继续堆积路径截取和裁剪细节。

---

## 9. 第五阶段：抽离 TrajectorySafetyChecker

新增：

```text
minco_core/trajectory_safety_checker.hpp
minco_core/trajectory_safety_checker.cpp
```

迁移当前 `MincoPlanner` 中的安全查询逻辑：

```text
1. checkCollision(pos)
2. checkCollision(traj)
3. getEsdfDistance(pos)
4. isTrajSafe() 中与地图 query 相关的部分
```

建议接口：

```cpp
class TrajectorySafetyChecker {
public:
  void configure(
    double safe_dist,
    double sample_dt,
    rclcpp::Logger logger);

  void setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query);

  bool checkPoint(const Eigen::Vector3d & pos) const;
  bool checkTrajectory(const traj_opt::Trajectory & traj) const;
  double getDistance(const Eigen::Vector3d & pos) const;
};
```

要求：

```text
1. 只使用 mode_context.dynamicQuery()；
2. PRIORMAP 下 dynamicQuery 是 FrameAwareRogQuery；
3. EXPLORATION 下 dynamicQuery 是 raw ROGMap；
4. 不改变原安全检测判定阈值和采样逻辑；
5. dynamicQuery 不可用时返回 unsafe，并输出限频日志。
```

`MincoPlanner` 对外保留兼容接口：

```cpp
double MincoPlanner::getEsdfDistance(const Eigen::Vector3d & p)
{
  return safety_checker_->getDistance(p);
}

bool MincoPlanner::isTrajSafe()
{
  return safety_checker_->checkTrajectory(current_traj_);
}
```

---

## 10. minco_utils 通用化要求

检查 `minco_utils.hpp/cpp`，避免重复实现工具函数。

适合放入或复用 `minco_utils` 的内容：

```text
1. quaternionToYaw()
2. transformPoint()
3. rotateVector()
4. interpolateByArcLength()
5. pathLength()
6. findNearestPathIndex()
7. clampValue()
8. getDistFromTrapezoid()
```

不适合放入 `minco_utils` 的内容：

```text
1. Nav2CostmapQuery
2. FrameAwareRogQuery
3. PlannerModeContext
4. GlobalPathSearcher
5. LocalPathProcessor
6. TrajectorySafetyChecker
```

原因：

```text
minco_utils 只放无状态数学/几何工具；
query adapter 和 searcher 是有状态模块，不能塞进 utils。
```

---

## 11. 内存和指针要求

本轮重构要顺带检查，但不要过度优化。

### 11.1 所有权建议

使用：

```text
unique_ptr:
  PlannerModeContext
  GlobalPathSearcher
  LocalPathProcessor
  TrajectorySafetyChecker
  MincoOptimizer
  BackupTrajOpt
  YawTrajOpt
  Visualizer
  MincoFsm

shared_ptr:
  rog_map::MapQueryInterface
  ROGMapROS query interface
  Nav2CostmapQuery
  FrameAwareRogQuery
  objects shared by searcher / optimizer / safety checker / corridor
```

不要用裸 `new/delete` 创建新增模块。

### 11.2 不要高频创建 query adapter

以下对象只允许低频创建或重建：

```text
Nav2CostmapQuery
FrameAwareRogQuery
PlannerModeContext query binding
```

只在：

```text
configure()
setMap(raw_rog_map)
costmap/raw map 更新
mode context rebuild
```

中创建，不要在 `PlanGlobalPath()` / `ReplanLocal()` 高频路径中创建。

### 11.3 Astar 内存管理

如果本轮改动触及 `astar.cpp/hpp`，可以做 RAII 化，但不是必须。

允许将裸数组：

```cpp
new[] / delete[]
```

改成：

```cpp
std::vector
```

要求：

```text
1. 不改变 A* 搜索算法；
2. 不改变 cost 计算；
3. buffer 在 setSize 或 configure 时分配；
4. plan 高频路径中不要反复大分配；
5. 如果风险较高，先不做 Astar RAII，只在说明文档中列为后续优化。
```

---

## 12. MincoPlanner 最终目标形态

重构后 `MincoPlanner` 应该主要保留：

```text
1. configure()
2. activate/deactivate/cleanup
3. createPlan()
4. consumePendingGoal()
5. getRobotPose()
6. PlanGlobalPath()
7. ReplanLocal()
8. publishOptimizedTrajectory / backup / emergency / escape 调度
9. 参数回调
10. FSM 和 timer 初始化
```

不应该再直接包含：

```text
1. Nav2CostmapQuery 类定义；
2. FrameAwareRogQuery 类定义；
3. EXPLORATION boundary search 大段实现；
4. ROGMap boundary clipping 细节；
5. line-free sampling 细节；
6. trajectory safety map query 细节。
```

---

## 13. CMake 更新

新增 `.cpp` 文件后更新 `CMakeLists.txt`。

必须保证这些文件被加入对应 target：

```text
map_query_adapters.cpp
planner_mode_context.cpp
global_path_searcher.cpp
local_path_processor.cpp
trajectory_safety_checker.cpp
```

不要执行编译，只更新配置。

---

## 14. 静态检查要求，不要编译

完成后只做静态检查，不要编译。

允许执行或人工检查：

```bash
grep -R "class Nav2CostmapQuery" -n .
grep -R "class FrameAwareRogQuery" -n .
grep -R "planGlobalPathExploration" -n .
grep -R "clipLocalPathByRogBoundary" -n .
grep -R "checkCollision" -n .
grep -R "colcon build" -n .
```

不要执行：

```bash
colcon build
colcon test
```

输出一个文档：

```text
minco_refactor_static_report.md
```

内容包括：

```text
1. 新增文件列表；
2. 修改文件列表；
3. 从 minco_planner.cpp 搬出了哪些类/函数；
4. MincoPlanner 当前剩余职责；
5. PRIORMAP 模式语义是否保持；
6. EXPLORATION 模式语义是否保持；
7. 哪些地方只做了静态检查，未编译；
8. 后续需要用户手动执行的编译命令；
9. 可能的风险点。
```

文档中明确写：

```text
本次任务按用户要求未执行 colcon build，编译需要用户后续手动验证。
```

---

## 15. 验收标准

静态结构上必须满足：

```text
1. minco_planner.cpp 中不再定义 Nav2CostmapQuery；
2. minco_planner.cpp 中不再定义 FrameAwareRogQuery；
3. minco_planner.cpp 中不再包含大段 EXPLORATION boundary search 实现；
4. minco_planner.cpp 中不再直接实现轨迹安全地图查询细节；
5. PRIORMAP:
   - global query = Nav2CostmapQuery
   - sparsify query = Nav2CostmapQuery
   - dynamic query = FrameAwareRogQuery
   - output frame = map
6. EXPLORATION:
   - global query = raw ROGMap
   - sparsify query = raw ROGMap
   - dynamic query = raw ROGMap
   - output frame = camera_init
7. 不改变 MincoOptimizer / Smac / Astar 的算法内部行为。
8. 不执行任何编译或运行命令。
```

---

## 16. 最终输出

完成后回复：

```text
1. 已完成的重构阶段；
2. 新增文件；
3. 修改文件；
4. 未执行编译的确认；
5. 用户下一步应执行的手动验证命令；
6. minco_refactor_static_report.md 的内容摘要。
```

再次强调：**不要编译，不要运行 colcon build。**
