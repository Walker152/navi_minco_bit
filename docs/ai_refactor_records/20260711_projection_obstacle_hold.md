# Projection 障碍延迟清除改造记录

## User Intent

为 ProjectionLayer 增加最高优先级的二维障碍退出时间保持：raw OCCUPIED 始终立即占据；从 OCCUPIED 退出后保持固定时间；期间重新 OCCUPIED 取消旧计时；再次退出时重新计时。

## Scope

ProjectionLayer 状态、ROGMap 参数传递与刷新联动、sentry1 参数。

## Out of Scope

三维 ProbMap decay、Hole Fill 算法、DynamicLayer、MINCO、运行期参数切换、日志和性能字段改造。

## Explorer Findings

### Files inspected

- `src/perception/rog_map/include/rog_map/projection_layer.hpp`
- `src/perception/rog_map/src/rog_map/projection_layer.cpp`
- `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Active logic path

ProbMap dirty columns → ProjectionLayer `updateFull/updateDirty` → `cell_buffer_` → row-major mask → DynamicLayer Field → Query snapshot。

### Data flow

`current_update_time_` 由 ROGMap 传入 ProjectionLayer。deadline 保存在 `CellData`，随滑动哈希重叠区保留，并由现有 reset 路径在哈希槽复用时清空。

### Risk notes

仅依赖 dirty column 无法保证无新观测时按时清除，因此需要独立扫描 deadline。当前 sentry1 关闭 Hole Fill，本次不扩大到补洞邻域重算。

### Recommended modification boundary

本次修改涉及比赛验证逻辑，采用最小改动策略。保持 topic、frame、QoS、地图几何和三维衰减不变。

## Modifier Changes

### Files changed

- `projection_layer.hpp/.cpp`
- `rog_map_core/config.hpp`
- `rog_map.cpp`
- `sentry1.yaml`

### Key changes

- 增加 `occupied_clear_deadline` 和单一 `obstacle_hold_time` 参数。
- raw OCCUPIED 立即占据并取消 deadline。
- 最近一次退出 OCCUPIED 时建立一次 deadline，连续非占据不续期。
- 无 dirty 时通过 `advanceObstacleClearance()` 推进到期。
- timer-only mask 变化纳入 projection/field 更新链路。

### Behavior preserved

`obstacle_hold_time <= 0` 时继续使用原有 hysteresis；二维滑动、三维 decay、Field 算法保持不变。

### Behavior intentionally adjusted

`sentry1.yaml` 设置 `projection.obstacle_hold_time: 3.0`，raw OCCUPIED 退出后二维障碍保持 3 秒。

### Notes

未增加配置日志、周期日志、性能字段或额外索引结构。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

### Issues found

仓库没有现成 ProjectionLayer 单元测试目标；未获构建许可，未新增或运行编译型测试。

### Final result

PASS（静态审计范围）
