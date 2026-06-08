# MincoPlanner 结构重构静态报告

## 1. 新增文件列表

- `include/minco_core/components/map_query_adapters.hpp`
- `src/minco_core/components/map_query_adapters.cpp`
- `include/minco_core/components/planner_mode_context.hpp`
- `src/minco_core/components/planner_mode_context.cpp`
- `include/minco_core/components/global_path_searcher.hpp`
- `src/minco_core/components/global_path_searcher.cpp`
- `include/minco_core/components/local_path_processor.hpp`
- `src/minco_core/components/local_path_processor.cpp`
- `include/minco_core/components/trajectory_safety_checker.hpp`
- `src/minco_core/components/trajectory_safety_checker.cpp`

## 2. 修改文件列表

- `CMakeLists.txt`
- `include/minco_core/header.hpp`
- `include/minco_core/minco_fsm.hpp`
- `include/minco_core/minco_planner.hpp`
- `include/minco_core/visualizer.hpp`
- `src/minco_core/minco_planner.cpp`

## 3. 从 minco_planner.cpp 搬出的类/函数

- 搬到 `map_query_adapters.*`
  - `Nav2CostmapQuery`
  - `FrameAwareRogQuery`
  - TF 点变换、向量旋转、lethal cost 判断

- 搬到 `planner_mode_context.*`
  - PRIORMAP / EXPLORATION mode 解析
  - `planning_frame` / `output_frame` / `map_frame` / `rog_frame` 绑定
  - `globalQuery()` / `dynamicQuery()` / `sparsifyQuery()` 创建与重建

- 搬到 `global_path_searcher.*`
  - Nav2 costmap / ROGMap 全局路径搜索
  - SMAC / Astar 调度
  - EXPLORATION reachable boundary search
  - start/goal `worldToMap` 检查与 path reconstruction

- 搬到 `local_path_processor.*`
  - local path nearest-index / lookahead 截取
  - ROGMap boundary clipping
  - line-free sampling
  - `getSparseWaypoints()` 调用

- 搬到 `trajectory_safety_checker.*`
  - trajectory map collision sampling
  - point collision check
  - ESDF distance query
  - start point obstacle projection query

## 4. MincoPlanner 当前剩余职责

- Nav2 planner plugin 生命周期与参数加载。
- ROGMapROS 创建和 raw query 获取。
- 初始化 mode context、global searcher、local path processor、safety checker。
- FSM / timer / publisher / recovery server 连接。
- `createPlan()` pending goal 管理。
- `PlanGlobalPath()` 调度 global searcher 并更新全局路径缓存。
- `ReplanLocal()` 调度 local seed、MINCO optimizer、yaw optimizer、backup/visualization/publish。
- 对外兼容 `checkCollision()` / `getEsdfDistance()` / `makePlan()` 等接口，但内部委托新模块。

## 5. PRIORMAP 语义保持

- `PlannerModeContext` 中 PRIORMAP:
  - `planningFrame = map_frame`
  - `outputFrame = map_frame`
  - `useStaticEsdf = true`
  - `directOdomPose = false`
  - `globalQuery = Nav2CostmapQuery`
  - `dynamicQuery = FrameAwareRogQuery(raw_rog_query, map_frame -> rog_frame)`
  - `sparsifyQuery = globalQuery`
- `LocalPathProcessor` 稀疏使用 `mode_context.sparsifyQuery()`，所以 PRIORMAP 稀疏仍使用 Nav2 costmap / prior map，不使用 ROGMap。
- `TrajectorySafetyChecker` 只使用 `mode_context.dynamicQuery()`，所以 PRIORMAP safety/ESDF 查询通过 frame-aware ROGMap。
- `/opt_path` frame 仍由 `output_frame_` 发布，PRIORMAP 下为 `map`。

## 6. EXPLORATION 语义保持

- `PlannerModeContext` 中 EXPLORATION:
  - `planningFrame = rog_frame`
  - `outputFrame = rog_frame`
  - `useStaticEsdf = false`
  - `directOdomPose = true`
  - `globalQuery = raw ROGMap`
  - `dynamicQuery = raw ROGMap`
  - `sparsifyQuery = raw ROGMap`
- `GlobalPathSearcher` 保留 goal 在 ROGMap 内但不可达时 fallback 到 reachable boundary search 的逻辑。
- reachable boundary search 仍从 start cell 扩展，只从 start 可达连通域中选择边界候选。
- `/opt_path` frame 仍由 `output_frame_` 发布，EXPLORATION 下为 `camera_init` / `rog_frame`。

## 7. 静态检查结果

- `git diff --check -- src/navigation/minco_planner`：通过，无输出。
- `grep -R "class Nav2CostmapQuery" -n src/navigation/minco_planner/include src/navigation/minco_planner/src`
  - 仅命中 `include/minco_core/components/map_query_adapters.hpp`。
- `grep -R "class FrameAwareRogQuery" -n src/navigation/minco_planner/include src/navigation/minco_planner/src`
  - 仅命中 `include/minco_core/components/map_query_adapters.hpp`。
- `grep -R "planGlobalPathExploration" -n src/navigation/minco_planner/include src/navigation/minco_planner/src`
  - 无命中。
- `grep -R "clipLocalPathByRogBoundary" -n src/navigation/minco_planner/include src/navigation/minco_planner/src`
  - 仅命中 `LocalPathProcessor`。
- `grep -R "checkCollision" -n src/navigation/minco_planner/include src/navigation/minco_planner/src`
  - `MincoPlanner` 只保留对外兼容 wrapper，地图查询细节已移到 `TrajectorySafetyChecker`。
- `grep -R "colcon build" -n src/navigation/minco_planner`
  - 仅命中文档中的说明/历史命令文本，未执行。
- `grep -R "colcon test|ros2 launch|ros2 run" -n src/navigation/minco_planner`
  - 仅命中文档中的说明/历史命令文本，未执行。

## 8. 未编译确认

本次任务按用户要求未执行 `colcon build`，编译需要用户后续手动验证。

未执行：

- `colcon build`
- `colcon test`
- `ros2 launch`
- `ros2 run`

## 9. 用户后续手动验证命令

建议用户在合适环境中手动执行：

```bash
colcon build --symlink-install
```

可选静态/运行验证：

```bash
grep -R "class Nav2CostmapQuery" -n src/navigation/minco_planner/include src/navigation/minco_planner/src
grep -R "class FrameAwareRogQuery" -n src/navigation/minco_planner/include src/navigation/minco_planner/src
grep -R "planGlobalPathExploration" -n src/navigation/minco_planner/include src/navigation/minco_planner/src
grep -R "clipLocalPathByRogBoundary" -n src/navigation/minco_planner/include src/navigation/minco_planner/src
grep -R "checkCollision" -n src/navigation/minco_planner/include src/navigation/minco_planner/src
```

## 10. 可能的风险点

- 本轮按要求未编译，仍需用户手动验证 include 依赖和链接。
- `GlobalPathSearcher` 以 non-owning 指针持有 Astar / Smac，生命周期由 `MincoPlanner` 管理；当前 cleanup 顺序先 reset searcher，再 reset Astar / Smac。
- `CMakeLists.txt` 已从 minco_core GLOB 改为显式源文件列表；后续新增 minco_core 源文件时需要手动加入列表。
- `MincoPlanner::checkCollision()` 仍作为外部兼容接口存在，但安全地图查询细节已由 `TrajectorySafetyChecker` 执行。
