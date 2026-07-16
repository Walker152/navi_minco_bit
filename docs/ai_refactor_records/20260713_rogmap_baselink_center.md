# ROGMap 使用 base_link 中心改造记录

## User Intent

将 ROGMap 的地图滑动、局部更新窗口和 ESDF 更新中心从 LiDAR odom 原点可选迁移到车辆 `base_link` 中心，同时保持 raycasting 使用 LiDAR 原点。

## Scope

ROGMap Config、`ROGMap::updateMapInternal()`、`ProbMap::updateProbMap()` 接口和当前车辆 `sentry1.yaml`。

## Out of Scope

Point-LIO、odom topic、ROS 点云/odom 同步、输入点云、raycasting 算法、ProjectionLayer、DynamicLayer、Field、MincoPlanner 和 controller。

## Explorer Findings

### Files inspected

- `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp`
- `src/perception/rog_map/include/rog_map/prob_map.h`
- `src/perception/rog_map/include/rog_map/rog_map.h`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/perception/rog_map/src/rog_map/prob_map.cpp`
- `src/perception/rog_map/include/rog_map/rog_map_core/common_lib.hpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Active logic path

`ROGMap::updateMapInternal()` 先更新机器人状态，再调用 `ProbMap::updateProbMap()`。原实现用同一个 `pose.first` 同时负责地图滑动和 raycasting 起点。

### Data flow

ROS 同步回调继续提供 LiDAR 世界位姿。入口根据 yaw 和固定局部偏移计算地图中心，LiDAR pose 与地图中心分别传入概率地图更新。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。必须避免将车辆中心传给 `raycastProcess()`，否则会改变射线量程和自由空间更新语义。

### Recommended modification boundary

只在 ROGMap 核心入口计算一次车辆中心，并为 `updateProbMap()` 增加一个位置参数，不修改 raycasting 内部实现。

## Modifier Changes

### Files changed

- `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp`
- `src/perception/rog_map/include/rog_map/prob_map.h`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/perception/rog_map/src/rog_map/prob_map.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Key changes

新增 `map_sliding.center_offset_enable` 和 `map_sliding.center_offset`；按 yaw 旋转偏移计算 `map_center_pos`；显式拆分 `sensor_pos` 和 `map_center_pos`。

### Behavior preserved

参数默认关闭时所有位置与修改前相同。raycasting、高度检查、首次近场清空继续使用 LiDAR 原点；ROS 回调和 odom topic 不变。

### Behavior intentionally adjusted

当前 `sentry1.yaml` 启用 `[0.0, 0.20, 0.0]`。机器人状态位置、地图滑动判断和执行、local update box、ESDF 更新改用车辆中心。

### Notes

偏移数组长度非法时输出一次警告并回退为零，不阻止节点启动。每帧仅在功能启用时增加一次 yaw、sin 和 cos 计算。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

未发现 raycasting 原点迁移或越界模块修改。未执行构建，因为 AGENTS.md 禁止未授权构建。

### Final result

PASS
