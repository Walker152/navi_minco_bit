# 删除 ROGMap drone TF 改造记录

## User Intent

删除 ROGMap 中已经滞后的 `drone` 坐标变换；保留 Point-LIO 发布的 `body` 坐标变换。

## Scope

仅删除 ROGMap 的 `drone` TF broadcaster、发布逻辑及对应头文件依赖。

## Out of Scope

- 不修改 Point-LIO 的 `camera_init -> body` TF。
- 不修改 ROGMap 的 odom、点云订阅和建图数据流。
- 不处理点云与 odom 的时间同步。
- 不修改 topic、frame 配置或比赛参数。

## Explorer Findings

### Files inspected

- `src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp`
- `src/perception/Point-LIO/src/laserMapping.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- `src/navigation/navi2_bringup/config/our.rviz`

### Active logic path

ROGMap 订阅 `/aft_mapped_to_init`，在 odom 回调中更新机器人状态，并额外发布 `camera_init -> drone` TF。Point-LIO 已独立发布 `camera_init -> body` TF。

### Data flow

`/aft_mapped_to_init` 仍进入 ROGMap 的 `odomCallback()` 并调用 `updateRobotState()`；删除的 TF 发布不参与地图更新。

### Risk notes

仓库内未发现 ROGMap 建图、规划或控制逻辑消费 `drone` frame。仓库外若有工具直接依赖 `drone`，删除后该工具将无法查询此 frame。

### Recommended modification boundary

只删除 ROGMap 的冗余 TF 发布，不替换为 `body`，避免与 Point-LIO 的 `body` TF broadcaster 冲突。

## Modifier Changes

### Files changed

- `src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp`
- `docs/ai_refactor_records/20260715_remove_rogmap_drone_tf.md`

### Key changes

- 删除 `camera_init/map -> drone` TF 发布代码。
- 删除不再使用的 TF broadcaster 成员、初始化和头文件依赖。

### Behavior preserved

- ROGMap odom 状态更新保持不变。
- 点云接收、地图更新、滑窗、投影层和 ESDF 保持不变。
- Point-LIO 的 `camera_init -> body` TF 保持不变。

### Behavior intentionally adjusted

ROGMap 不再发布 `drone` frame。

### Notes

本次修改未将 `drone` 改名为 `body`，以避免重复 TF 发布者。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可（本次不需要且未执行构建）

### Issues found

无。

### Final result

PASS
