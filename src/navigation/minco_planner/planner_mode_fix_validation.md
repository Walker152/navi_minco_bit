# MincoPlanner 双模式剩余逻辑修复验证

## 1. 修改文件列表

- `src/navigation/minco_planner/include/minco_core/minco_planner.hpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/minco_planner/planner_mode_fix_validation.md`

## 2. 修复点与代码位置

1. PRIORMAP 路径稀疏不再使用 ROGMap
   - 新增 `sparsify_query_`：`include/minco_core/minco_planner.hpp:225`
   - PRIORMAP 设置为 `global_search_query_`，EXPLORATION 设置为 `dynamic_query_`：`src/minco_core/minco_planner.cpp:393`
   - `ReplanLocal()` 稀疏 line-free 改为 `sparsify_query_`：`src/minco_core/minco_planner.cpp:1579`

2. `clipLocalPathByRogBoundary()` 返回值被检查
   - `ReplanLocal()` 检查 `clip_ok` 和裁剪后长度：`src/minco_core/minco_planner.cpp:1559`
   - 空 path 返回 false：`src/minco_core/minco_planner.cpp:2275`
   - 首点越界时 `path.clear()` 并返回 false：`src/minco_core/minco_planner.cpp:2319`
   - 裁剪后不足 2 点返回 false：`src/minco_core/minco_planner.cpp:2343`

3. `createPlan()` 中 TF 失败不再伪装 frame
   - start/goal 必须成功归一到 `planning_frame_`：`src/minco_core/minco_planner.cpp:1239`
   - 失败时清除 pending goal 并返回空 path：`src/minco_core/minco_planner.cpp:1243`

4. PRIORMAP odom fallback 空 frame 不再当成 map
   - EXPLORATION 空 frame 仍按 `planning_frame_` 处理：`src/minco_core/minco_planner.cpp:2818`
   - PRIORMAP 空 frame 按 `rog_frame_` 处理：`src/minco_core/minco_planner.cpp:2829`
   - TF buffer 为空或 TF 失败时返回 false：`src/minco_core/minco_planner.cpp:2843`

5. EXPLORATION goal 在窗口内但不可达时 fallback 到边界搜索
   - direct ROGMap 搜索成功才返回 true：`src/minco_core/minco_planner.cpp:1350`
   - 搜索失败时打印 WARN 并继续 reachable boundary search：`src/minco_core/minco_planner.cpp:1354`

6. `setMap()` 不再绕过 mode wrapper
   - `setMap()` 只更新 `rog_query_raw_` 并调用 `rebuildModeDependentQueries()`：`src/minco_core/minco_planner.cpp:895`
   - PRIORMAP 下 `dynamic_query_` 重建为 `FrameAwareRogQuery`：`src/minco_core/minco_planner.cpp:400`

7. 禁止 mode / frame / 双模式核心参数 hot reload
   - 拒绝 `planner_mode`、`frames.*`、`priormap.*`、`exploration.*` 核心参数：`src/minco_core/minco_planner.cpp:953`
   - 返回 configure-time only 错误：`src/minco_core/minco_planner.cpp:1021`

## 3. PRIORMAP 语义验证

- `global_search_query_` 是 Nav2 costmap：`rebuildModeDependentQueries()` 中 `Nav2CostmapQuery(costmap_ros_->getCostmap())`。
- `sparsify_query_` 是 Nav2 costmap：PRIORMAP 分支设置 `sparsify_query_ = global_search_query_`。
- `dynamic_query_` 是 `FrameAwareRogQuery`：PRIORMAP 分支用 raw ROGMap、TF、`map_frame_`、`rog_frame_` 重建 wrapper。
- `/opt_path` frame 是 `map`：PRIORMAP 初始化中 `output_frame = map_frame_`，发布 header 使用 `output_frame_`。

## 4. EXPLORATION 语义验证

- `global_search_query_` 是 ROGMap：EXPLORATION 分支设置为 `rog_query_raw_`。
- `sparsify_query_` 是 ROGMap：EXPLORATION 分支设置为 `dynamic_query_`。
- `dynamic_query_` 是 ROGMap：EXPLORATION 分支设置为 `rog_query_raw_`。
- `/opt_path` frame 是 `camera_init`：EXPLORATION 初始化中 `output_frame = rog_frame_`，发布 header 使用 `output_frame_`。

## 5. 静态验证结果

- `git diff --check -- src/navigation/minco_planner/include/minco_core/minco_planner.hpp src/navigation/minco_planner/src/minco_core/minco_planner.cpp`：通过，无输出。
- 旧反模式 grep：
  - `normalized_start = start`
  - `normalized_goal = goal`
  - `dynamic_query_ = map`
  - `isLineFree(this->dynamic_query_`
  - `odom_pose.header.frame_id.empty() ||`
  - `planner_mode is not hot-reloadable`

  以上反模式均无命中。

## 6. 编译结果

用户已确认编译没问题。本轮中我按用户要求未再执行本地编译。

## 7. 仍需实机验证

- PRIORMAP 下 `map -> camera_init` TF 可用时，动态障碍梯度和 safety check 查询正常。
- PRIORMAP 下 Nav2 costmap 可用时，global search、path sparsify、control point selection 不受 ROGMap 影响。
- EXPLORATION 下目标在 ROGMap 内但被障碍隔离时，会落回从机器人当前位置出发的 reachable boundary path。
- 两种模式下 `/opt_path` header frame 分别为 `map` 和 `camera_init`。
- 参数热更新请求被 planner_server 拒绝后，现有动态优化器参数仍可按原逻辑更新。
