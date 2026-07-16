# Projection / Field 滑动一致性改造记录

## User Intent

按 ROGMap 原生统一调度关系修复 Projection / Field 滑动一致性。Field 作为
Projection 的派生结果，在 Projection 更新后立即重建，不再使用独立更新频率；同时禁止
向查询快照发布与当前 Projection geometry 不一致的距离场。

关键约束：最小修改，不改变 Projection 分类与 DynamicLayer EDT 算法，不修改
SlidingMap、CounterMap、planner、QueryAdapter 或外部接口，不删除现有配置兼容项，不执行
Git 提交。

## Scope

- `ROGMap::init()` 的初始滑动入口。
- `ROGMap::refreshLayers()` 中 Projection 到 Field 的调度关系。
- `ROGMap::refreshQuery()` 中 layer/field geometry 一致性保护。
- `DynamicLayer` 的 geometry 匹配与有效性检查。

本次修改属于 map / ROGMap 模块的 bug 修复，允许小幅改变 Field 调度行为。本次涉及比赛
验证逻辑，采用最小改动策略。

## Out of Scope

- SlidingMap、CounterMap、InfMap、ESDFMap 的实现。
- ProjectionLayer 分类、滞回、孔洞填充逻辑。
- DynamicLayer EDT 计算流程和 `ESDFUtils::computeEDT2D()`。
- planner、QueryAdapter、topic、frame、QoS、launch 和参数默认值。
- 删除 `field_update_rate` 配置项。

## Explorer Findings

### Files inspected

- `src/perception/rog_map/src/rog_map/prob_map.cpp`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/perception/rog_map/include/rog_map/rog_map.h`
- `src/perception/rog_map/include/rog_map/projection_layer.hpp`
- `src/perception/rog_map/src/rog_map/projection_layer.cpp`
- `src/perception/rog_map/include/rog_map/field_layer.hpp`
- `src/perception/rog_map/src/rog_map/field_layer.cpp`
- `src/perception/rog_map/include/rog_map/query_adapter.hpp`
- `src/perception/rog_map/src/rog_map/query_adapter.cpp`
- `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp`

### Active logic path

`ProbMap::slideAllMap()` 统一滑动主地图、InfMap，以及按配置启用的 Frontier/ESDF map，并调用
`markAllDirtyColumns()`。`ROGMap::refreshLayers()` 根据 geometry、full refresh 标记和 dirty
columns 更新 ProjectionLayer，随后原实现通过 `field_dirty_` 和 `field_update_rate` 独立限频
DynamicLayer。`ROGMap::refreshQuery()` 原先无条件复制 DynamicLayer distances。

### Data flow

概率地图或滑动变化 → dirty/full refresh 状态 → ProjectionLayer full/dirty update → mask →
DynamicLayer EDT → MapSnapshot → QueryAdapter。

### Risk notes

- 初始滑动若绕过 `slideAllMap()`，Frontier/ESDF 和 dirty/full refresh 状态可能与主地图不一致。
- Projection 已更新但 Field 因限频跳过时，会形成新 layer 与旧 distances 的短时错位。
- 仅检查 Field 非空不足以证明其尺寸、分辨率和原点与当前 Projection 一致。

### Recommended modification boundary

只替换 ROGMap 初始化滑动入口、移除 Field 独立调度条件、增加 DynamicLayer geometry 校验并
保护 snapshot distances；保留算法、对象所有权、配置加载和外部接口。

## Modifier Changes

### Files changed

- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/perception/rog_map/include/rog_map/rog_map.h`
- `src/perception/rog_map/include/rog_map/field_layer.hpp`
- `src/perception/rog_map/src/rog_map/field_layer.cpp`
- `docs/ai_refactor_records/20260708_projection_field_sliding_consistency.md`

### Key changes

- `ROGMap::init()` 改为通过 `slideAllMap()` 完成初始滑动。
- `DynamicLayer` 新增 `matchesGeometry()`，`isValid()` 同时校验距离数组尺寸。
- Field 更新条件改为 `layer_updated`、geometry 变化或 Field 无效，不再读取
  `field_update_rate` 或 stale timeout。
- 删除 `field_dirty_` 调度状态；兼容统计字段固定表达“无独立调度”。
- `refreshQuery()` 仅在 Field 有效、geometry 匹配且非 stale 时复制 distances，否则发布空
  distances 并标记 `field_stale=true`。

### Behavior preserved

- ProjectionLayer 和 DynamicLayer 的分离结构及所有权不变。
- Projection full/dirty 判定、分类、滞回和孔洞填充逻辑不变。
- DynamicLayer 的 mask 输入、EDT、膨胀、截断和插值计算不变。
- `field_update_rate` 配置仍可加载，旧 YAML 不失效。
- planner 和 QueryAdapter 外部接口不变。

### Behavior intentionally adjusted

- Projection 每次 full/dirty 更新后，Field 立即从同一 mask 重建。
- Projection 未更新且 Field geometry 有效时，不重复重建 Field。
- Field 与当前 layer geometry 不一致或无效时，查询快照不再携带旧 distances。

### Notes

`last_field_update_time_` 仅保留用于既有更新间隔统计，不参与调度。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次未修改 XML/launch；确认配置项仍保留）
- [x] 用户允许范围内的测试或静态检查
- [ ] 如需构建，已取得用户明确许可

执行结果：

- 修改前静态回归契约按预期失败，覆盖初始统一滑动、Field geometry API、独立限频移除、
  Projection 跟随更新和 snapshot geometry 防护。
- 修改后相同契约全部通过。
- `git diff --check` 通过。
- 变更行的 `clang-format --dry-run --Werror` 检查通过。
- grep 确认 `refreshLayers()` 不再包含 `field_update_rate`、`field_period`、控制用
  `period_ready`、`period_not_ready`、stale age timeout 或 `field_dirty_`。
- diff 文件范围符合任务边界，未修改禁止触碰的模块。
- 未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
- 用户于 2026-07-08 反馈编译通过；Agent 未重复执行构建。

### Issues found

未发现静态审计或编译问题。ROGMap 启动、实际滑动场景和 planner 运行时联调仍需实机验证。

### Final result

PASS
