# MINCO 调试可视化补充改造记录

## User Intent

补充局部截取终点、validation 前候选轨迹和候选状态可视化，不修改规划、优化、validation 或控制逻辑，并完成编译检查。

## Scope

- `ReplanLocal()` 中新增调试可视化状态更新。
- `Visualizer` 中新增候选 Path publisher、Marker 和互斥保护的缓存状态。
- 增加源码级回归检查。

## Out of Scope

- local dense path、A*、sparse waypoint 可视化。
- MINCO 优化、validation 判定、局部截取策略。
- `/opt_path`、MPC 和已接受轨迹的数据流。

## Explorer Findings

### Files inspected

- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/minco_planner/src/minco_core/visualizer.cpp`
- `src/navigation/minco_planner/include/minco_core/visualizer.hpp`
- `src/navigation/minco_planner/CMakeLists.txt`

### Active logic path

`ReplanLocal()` 先调用 `buildSeed()`，再执行 MINCO optimize 和 `validateTrajectory()`；只有 validation 通过后才继续原有接受、发布和 Visualizer 更新路径。

### Data flow

- 真实局部截取终点来自 `seed.dense_path.back()`。
- 优化器有限 cost 的 `opt_traj` 在 validation 结果产生后写入独立候选缓存。
- 原有接受轨迹继续通过原路径写入 `last_traj_`、`/opt_path` 和 MPC。

### Risk notes

本次修改涉及比赛验证 planner 调用链，采用最小改动策略。主要风险是候选调试状态误入接受轨迹数据流，以及 transient-local 可视化残留。

### Recommended modification boundary

仅在 seed、非有限 cost、validation 三个边界调用 Visualizer 新接口；新增显示由现有可视化 timer 独立发布。

## Modifier Changes

### Files changed

- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/navigation/minco_planner/src/minco_core/visualizer.cpp`
- `src/navigation/minco_planner/include/minco_core/visualizer.hpp`
- `src/navigation/minco_planner/test/test_debug_visualization_source.py`
- `docs/ai_refactor_records/20260710_minco_debug_visualization.md`

### Key changes

- 新增 `/minco_candidate_path_vis`，QoS 为 transient-local + KeepLast(1)。
- `/minco_control_points_vis` 新增 `minco_local_end` 与 `minco_candidate_status` Marker。
- dense path 为空、优化失败和 cleanup 时清除对应旧显示。

### Behavior preserved

- `/opt_path` 和 `/opt_path_vis` 仍只使用 validation 通过的轨迹。
- validation 失败候选不写入 `last_traj_`，不发送 MPC，不改变 `has_last_traj_`。
- 未修改 validation、安全状态、优化和局部截取判定。

### Behavior intentionally adjusted

新增独立调试可视化；validation 失败轨迹现在可在候选话题观察。

### Notes

候选状态锚点优先使用候选轨迹起点；无候选轨迹但有明确失败原因时使用局部截取终点。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次未修改这些文件）
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可

执行结果：

- `python3 -m pytest src/navigation/minco_planner/test/test_debug_visualization_source.py -q`：4 passed。
- `git diff --check`：通过。
- `colcon build --packages-select minco_planner`：通过；仅有来自既有 `rog_map` 安装头文件的编译警告。

### Issues found

未发现越界修改或候选轨迹进入控制数据流。编译警告不来自本次修改文件。

### Final result

PASS
