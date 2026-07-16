# ProjectionLayer 滑动增量更新改造记录

## User Intent

将 ProjectionLayer 改造成与 ROGMap 风格一致的固定容量滑动层，正常滑动时保留重叠区域状态，只扫描新进入条带与 ProbMap 内容变化产生的 dirty columns；优先继承 `SlidingMap` 并重写基类方法，不修改 Field EDT、MINCO 或分类语义。用户明确要求本次不增加和运行测试。

## Scope

- `src/perception/rog_map/include/rog_map/projection_layer.hpp`
- `src/perception/rog_map/src/rog_map/projection_layer.cpp`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`
- 本改造记录

## Out of Scope

- DynamicLayer 的增量 EDT
- Projection 分类、hysteresis、hole fill 规则调整
- ROGMap 概率更新、decay、MINCO、controller
- 自动化测试（用户明确要求不用 test）

## Explorer Findings

### Files inspected

- `src/task.md`
- `rog_map_core/sliding_map.h` 与 `sliding_map.cpp`
- `projection_layer.hpp` 与 `projection_layer.cpp`
- `prob_map.h` 与 `prob_map.cpp`
- `rog_map.h` 与 `rog_map.cpp`
- `counter_map.h`、`field_layer.cpp`
- `sentry1.yaml`

### Active logic path

ProbMap 更新体素并维护当前窗口局部 row-major dirty column IDs。`ROGMap::refreshLayers()` 扫描 dirty XY 柱生成 ProjectionLayer，再将完整 row-major mask 交给 DynamicLayer 重建二维 EDT。

### Data flow

- ProbMap dirty columns：窗口内部 hit/miss/decay 内容变化。
- SlidingMap `resetCell()`：窗口滑动时新进入条带所复用的 hash slots。
- 两类 dirty 合并并去重后交给 ProjectionLayer 扫描。
- `cell_buffer_`：SlidingMap X-major 环形哈希布局下的权威 CellData。
- `cells_/values_/mask_`：当前窗口 Y-major row-major 连续输出视图。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。滑动与 dirty 更新不可互相替代：前者处理窗口边界变化，后者处理固定窗口内地图内容变化。正常滑动时必须忽略 ProbMap 因 `markAllDirtyColumns()` 设置的通用 full 标志，但初始化、大跨度滑动和非滑动异常 full 标志仍需全量恢复。

### Recommended modification boundary

ProjectionLayer 受保护继承 SlidingMap，复用哈希、边界、正负索引和条带清理，只重写 `resetCell()` 与 `resetLocalMap()`；保留下游连续视图和 DynamicLayer 全量 EDT。

## Modifier Changes

### Files changed

- `src/perception/rog_map/include/rog_map/projection_layer.hpp`
- `src/perception/rog_map/src/rog_map/projection_layer.cpp`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/navigation/navi2_bringup/params/sentry1.yaml`

### Key changes

- `ProjectionLayer : protected SlidingMap`，二维层以 Z 尺寸 1 复用原生三维滑动基类。
- 增加 `syncSlidingWindow()`，首次初始化固定容量 buffer，正常滑动调用基类 `mapSliding()`。
- `resetCell()` 清空被新进入单元复用的槽位并记录 slide-dirty；`resetLocalMap()` 固定容量全清。
- 分类、迟滞和 hole fill 的权威状态改存 `cell_buffer_`，滑动重叠区不复制、不重置。
- 每次需要输出时从 hash buffer 重建 row-major `cells_/values_/mask_`，不重新扫描三维 Z 柱。
- ROGMap 合并 ProbMap dirty 与 slide-dirty；正常滑动不再因通用 full 标志直接全量扫描。
- mask 版本判定仅在窗口未移动时逐下标比较；窗口移动直接视为空间视图变化，避免比较不同世界坐标语义的数组位置。
- `performance.dirty_column_enable` 改为 `true`，启用现有内容增量更新路径。

### Behavior preserved

- FREE/PASSABLE/OCCUPIED/UNKNOWN 分类语义。
- hysteresis、hole fill、value/mask 生成规则。
- DynamicLayer 继续使用完整 mask 重建 EDT。
- Query、可视化与下游继续读取 row-major 连续数组。

### Behavior intentionally adjusted

- 正常窗口滑动保留重叠区域完整 CellData 状态。
- 仅新进入条带和真实 dirty columns 重新扫描三维柱。
- 初始化、大跨度滑动、dirty 比例超限或非滑动异常仍允许全量刷新。

### Notes

`cell_buffer_` 沿用 SlidingMap 的 X-major hash；对外视图保持原 Y-major row-major。存储尺寸和分辨率为运行期固定配置，不支持在同一 ProjectionLayer 实例上二次初始化。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可

已执行：`git diff --check`、clang-format 静态格式检查、YAML 解析、关键索引/调用链 grep。

未执行测试：用户明确要求不用 test。

用户完成构建后已确认通过。日志 `log/build_2026-07-11_20-50-16_2/rog_map/command.log`
显示 `cmake --build` 与 `cmake --install` 均返回 `0`，构建产物时间戳已更新。

### Issues found

构建 stderr 中存在项目既有 warning，未发现构建失败。

### Final result

PASS
