# ROGMap + MincoPlanner 双模式改造与重构对话总结

## 1. 最初问题定位

当前 ROGMap 已经能够正常工作，但 planner 在打点时出现：

```text
failed to convert start world coordinates(x,y) to map coordinates
```

后来定位到，这不是 ROGMap 本身坏了，也不是 TF transform 报错，而是 planner 侧把 **map 系 start / goal** 直接拿去查 **camera_init 系、局部滑窗 ROGMap**，导致 `worldToMap()` 失败。

核心判断：

```text
PRIORMAP / 有先验地图模式：
  map/Nav2 应负责全局搜索、路径稀疏、控制点选择。
  ROGMap 只负责动态感知、优化梯度、安全检测、可选边界裁剪。

EXPLORATION / 无先验地图模式：
  全流程切到 camera_init。
  ROGMap 负责搜索、稀疏、优化、安全检测。
```

---

## 2. 双模式架构设计

最终确定两个模式名称：

```cpp
PRIORMAP
EXPLORATION
```

### 2.1 PRIORMAP 模式

保持旧导航系统语义不变：

```text
planning_frame = map
output_frame   = map
global search  = Nav2 global costmap / prior map
sparsify       = Nav2 global costmap / prior map
dynamic query  = FrameAwareRogQuery(raw ROGMap, map -> camera_init)
static ESDF    = enabled
/opt_path      = map
```

关键要求：

```text
ROGMap 不参与 PRIORMAP 下的全局搜索。
ROGMap 不参与 PRIORMAP 下的路径稀疏和控制点选择。
ROGMap 只作为动态约束源：
  1. Minco 优化动态梯度；
  2. 轨迹安全检测；
  3. 可选 seed path 边界裁剪。
```

### 2.2 EXPLORATION 模式

无先验地图，完全使用 ROGMap / camera_init：

```text
planning_frame = camera_init
output_frame   = camera_init
global search  = ROGMap reachable boundary search
sparsify       = ROGMap
dynamic query  = raw ROGMap
static ESDF    = disabled
/opt_path      = camera_init
```

边界搜索不能简单把目标投影到边界，因为投影点可能不可达。正确策略是：

```text
从当前位置 start 出发，在 ROGMap 上搜索；
边界候选必须来自 start 可达连通域；
goal 方向只作为候选评分或启发偏置；
goal 在窗口内但不可达时 fallback 到 reachable boundary search。
```

---

## 3. 第一次 Codex 改造任务

第一版任务是：重写 planner 双模式接口架构，但不改算法内部原理。

主要目标：

```text
1. 新增 planner_mode: PRIORMAP | EXPLORATION。
2. configure 阶段完成 mode init。
3. 不要每个阶段到处 if 判断，而是在模式初始化时绑定：
   - planning_frame
   - output_frame
   - global_search_query
   - dynamic_query
   - sparsify_query
   - plan_global_fn
4. PRIORMAP 全局搜索使用 Nav2 costmap。
5. EXPLORATION 全局搜索使用 ROGMap reachable boundary。
6. ROGMap 查询通过 FrameAwareRogQuery 适配 map -> camera_init。
7. /opt_path frame 按模式输出：
   - PRIORMAP: map
   - EXPLORATION: camera_init
```

之后检查发现主架构基本正确，但存在若干问题：

```text
1. PRIORMAP 路径稀疏仍然使用 ROGMap。
2. clipLocalPathByRogBoundary 返回值被忽略。
3. createPlan TF 失败时伪装 frame。
4. PRIORMAP odom fallback 空 frame 被当成 map。
5. EXPLORATION goal 在窗口内但不可达时没有 fallback 到边界。
6. setMap 可能绕过 mode wrapper。
7. mode/frame 参数 hot reload 未完全禁止。
```

其中明确决定：**速度 / yaw 转 planning_frame 的问题暂时不需要修改**。

---

## 4. 第二次 Codex 修复任务

第二版任务是小修，不再大改架构，排除速度 / yaw 处理：

```text
1. 新增 sparsify_query_：
   PRIORMAP = global_search_query_ / Nav2 costmap
   EXPLORATION = dynamic_query_ / ROGMap

2. 修复 clipLocalPathByRogBoundary：
   首点越界时 clear path；
   ReplanLocal 必须检查返回值。

3. createPlan 中 TF 失败不能伪装 frame。

4. PRIORMAP odom fallback：
   空 frame 按 rog_frame/camera_init 处理，再 TF 到 map。

5. EXPLORATION goal traversable 但不可达时 fallback 到 reachable boundary search。

6. setMap 后根据当前 mode 重新 wrap dynamic_query。

7. 禁止 planner_mode、frames、priormap、exploration 等 configure-time 参数热更新。
```

修完后判断：**双模式功能基本完备，可以进入编译和实机验证阶段**，但不能只凭源码断言完全可运行。

---

## 5. 代码臃肿问题与重构方向

后来指出 `minco_planner.cpp` 过于臃肿，包含了大量 query adapter、工具函数、搜索细节、安全检测等不属于它的职责。

最终确定合理的职责边界：

```text
MincoPlanner:
  Nav2 plugin 生命周期
  参数加载
  模式适配
  FSM 调度
  搜索/优化/发布串联

PlannerModeContext:
  PRIORMAP / EXPLORATION frame 与 query 绑定

MapQueryAdapters:
  Nav2CostmapQuery
  FrameAwareRogQuery

GlobalPathSearcher:
  PRIORMAP 全局搜索
  EXPLORATION reachable boundary search

LocalPathProcessor:
  局部路径截取
  ROGMap 边界裁剪
  路径稀疏
  seed 生成

TrajectorySafetyChecker:
  点/轨迹安全检测
  ESDF 距离查询
```

给 Codex 的重构任务重点要求：

```text
不要编译。
不要运行 colcon build。
只做代码组织重构，不改算法行为。
```

新增/拆分目标文件包括：

```text
map_query_adapters.hpp/cpp
planner_mode_context.hpp/cpp
global_path_searcher.hpp/cpp
local_path_processor.hpp/cpp
trajectory_safety_checker.hpp/cpp
```

---

## 6. 最新上传代码检查结果

上传了重构后的 planner 相关组件，包括：

```text
minco_planner.cpp
local_path_processor.cpp
map_query_adapters.cpp
planner_mode_context.cpp
trajectory_safety_checker.cpp
global_path_searcher.cpp
minco_fsm.cpp
recovery_behaivor.cpp
minco_utils.cpp
ROGMap 相关文件
LIWO 相关文件
```

检查结论：

```text
重构方向正确。
MincoPlanner 已经明显瘦身。
PlannerModeContext / GlobalPathSearcher / LocalPathProcessor / TrajectorySafetyChecker 的职责划分基本合理。
PRIORMAP / EXPLORATION 主语义基本保持正确。
```

但发现 2 个必须修的问题和 2 个建议修的问题。

---

## 7. 必须修问题 1：TrajectorySafetyChecker 忽略 safe_dist_

当前 `TrajectorySafetyChecker::checkPoint()` 里传入了 `safe_dist_`，但实际使用：

```cpp
(void)safe_dist_;
...
return esdf_dist > 0.0;
```

问题：

```text
只要 ESDF 距离大于 0 就认为安全；
没有要求 esdf_dist >= safe_dist_；
安全检测比优化约束宽松，可能贴障碍通过。
```

建议改成：

```cpp
const bool eval_ok = dynamic_query_->evaluate(pos, esdf_dist, esdf_grad);
if (!eval_ok || !std::isfinite(esdf_dist)) {
  return false;
}
return esdf_dist >= safe_dist_;
```

如果希望“栅格占据”和“ESDF 安全距离”双重检测，则保持 lethal cost 判断，再用 `safe_dist_` 判断距离。

---

## 8. 必须修问题 2：projectStartToFreeCell 投影逻辑错误

文件：

```text
minco_core/components/global_path_searcher.cpp
```

函数：

```cpp
projectStartToFreeCell(...)
```

### 8.1 问题位置 1：4 邻域分支

当前发现邻居 free 后直接：

```cpp
return false;
```

但没有更新 `mx/my`。

应改为：

```cpp
mx = static_cast<unsigned int>(sx);
my = static_cast<unsigned int>(sy);
return true;
```

也就是：

```cpp
if (map->isFree(static_cast<unsigned int>(sx), static_cast<unsigned int>(sy))) {
  mx = static_cast<unsigned int>(sx);
  my = static_cast<unsigned int>(sy);
  return true;
}
```

### 8.2 问题位置 2：makePlanOnQuery() 中 start worldToMap 后

当前只调用：

```cpp
projectStartToFreeCell(query, mx_start, my_start);
```

但不检查返回值。

应改为：

```cpp
if (!projectStartToFreeCell(query, mx_start, my_start)) {
  RCLCPP_ERROR(
    logger_,
    "%s failed to project start cell to a traversable free cell near world coordinates (%.2f, %.2f)",
    failure_source.c_str(), wx, wy);
  return false;
}
```

### 8.3 问题位置 3：planExploration() 中 start cell 不可通行后的投影

当前逻辑类似：

```cpp
const size_t start_idx = index_of(sx, sy);
if (!traversable(start_idx)) {
  projectStartToFreeCell(query, sx, sy);
}
```

应改为：

```cpp
const size_t start_idx = index_of(sx, sy);
if (!traversable(start_idx)) {
  if (!projectStartToFreeCell(query, sx, sy)) {
    RCLCPP_ERROR(
      logger_,
      "[MincoPlanner] EXPLORATION start cell is not traversable and cannot be projected to a nearby free cell.");
    return false;
  }
}
```

后面的 `updated_start_idx` 检查可以保留：

```cpp
const size_t updated_start_idx = index_of(sx, sy);
if (!traversable(updated_start_idx)) {
  RCLCPP_ERROR(
    logger_,
    "[MincoPlanner] EXPLORATION projected start cell is still not traversable in ROGMap.");
  return false;
}
```

---

## 9. 建议修问题 1：LocalPathProcessor::isLineFree query 无效时不应默认 true

当前：

```cpp
if (!map || map->resolution() <= 0.0) {
  return true;
}
```

建议改为更安全的：

```cpp
if (!map || map->resolution() <= 0.0) {
  return false;
}
```

或者在 query 不可用时退化为仅距离稀疏，不做 line-free 穿越判断。

---

## 10. 建议修问题 2：EXPLORATION 边界候选应排除 start 本身

如果机器人本来就在 ROGMap 边界 margin 内，reachable boundary search 可能选中 start 自己，导致路径只有 1 个点。

建议：

```text
boundary candidate 排除 updated_start_idx；
或要求路径长度 >= 最小距离，例如 0.5m 或 3 个 cell。
```

示例条件：

```cpp
if (current.idx == updated_start_idx) {
  continue;
}
```

或者：

```cpp
if (current.cost < min_boundary_path_length) {
  continue;
}
```

---

## 11. 当前最终状态判断

截至最后一次检查：

```text
架构重构：基本达标。
双模式语义：基本正确。
MincoPlanner 瘦身：基本成功。
PRIORMAP 接入旧 Nav2/map/BT/controller 的设计：仍然保持。
EXPLORATION reachable boundary 设计：方向正确。
```

但还不能说完全闭环，因为：

```text
1. safe_dist_ 未生效，安全检测有风险。
2. projectStartToFreeCell 有明显 bug。
3. 未确认所有 .hpp 和 CMakeLists.txt 是否同步完整。
4. 还没有实际 colcon build。
```

---

## 12. 下一步最小动作

建议下一步只做小修，不再大重构：

```text
1. 修 TrajectorySafetyChecker safe_dist_。
2. 修 global_path_searcher.cpp 中 projectStartToFreeCell 及其调用点。
3. 可选修 LocalPathProcessor::isLineFree query 无效默认 true。
4. 可选修 EXPLORATION 边界候选排除 start。
5. 然后执行 colcon build --symlink-install。
6. 优先实测 PRIORMAP，确认旧 Nav2/map/BT/controller 语义没有被破坏。
```

---

## 13. 给 Codex 的最小修复提示词

```text
请基于当前已重构后的 planner 代码进行小修，不要再次大规模重构。

需要修复：

1. TrajectorySafetyChecker::checkPoint() 必须使用 safe_dist_。
   当前只判断 esdf_dist > 0.0，这是错误的。
   应改为 evaluate 成功且 esdf_dist >= safe_dist_ 才认为安全。
   保留 lethal / inscribed cost 栅格检查。

2. 修复 minco_core/components/global_path_searcher.cpp 中 projectStartToFreeCell()：
   - 当 4 邻域发现 free cell 时，必须更新 mx/my，并 return true；
   - 不要发现 free 邻居后直接 return false。

3. 修复 makePlanOnQuery()：
   - start worldToMap 成功后，必须检查 projectStartToFreeCell() 返回值；
   - 如果投影失败，打印错误并 return false。

4. 修复 planExploration()：
   - start cell 不 traversable 时，必须检查 projectStartToFreeCell() 返回值；
   - 如果投影失败，打印明确错误并 return false；
   - 保留后续 updated_start_idx traversable 检查。

可选修复：

5. LocalPathProcessor::isLineFree() 中 query 无效时不要默认 true。
   建议改为 false，或明确退化为距离稀疏。

6. EXPLORATION reachable boundary candidate 排除 start 本身，或要求最小路径长度，避免 start 已在边界时生成单点路径。

限制：

- 不要修改 MincoOptimizer 算法内部。
- 不要修改 Smac / Astar 搜索算法原理。
- 不要修改 FSM 主流程。
- 不要再次大规模重构文件结构。
- 修完后可以编译检查，但不要改动无关文件。
```
