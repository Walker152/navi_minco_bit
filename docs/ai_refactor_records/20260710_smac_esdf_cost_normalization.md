# SMAC 动态 ESDF 代价归一化改造记录

## User Intent

将 SMAC 动态 ESDF 代价归一化为相对单位路径代价的无量纲倍率，并缓存 ROGMap 范围外或无效查询的零结果；保持先验搜索地图、动态查询源、软代价语义及 MINCO/validation 不变。

## Scope

- `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- 检查 `src/navigation/minco_planner/include/smac_search/smac_planner_2d_simple.hpp`
- 本改造记录

## Out of Scope

- 动态硬阻塞或新的安全距离参数
- 先验地图可通行判定
- MINCO 优化器与 validation
- ROGMap 与 ESDF 查询源关系
- 现有 ESDF 参数

## Explorer Findings

### Files inspected

- `src/navigation/minco_planner/include/smac_search/smac_planner_2d_simple.hpp`
- `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`

### Active logic path

`SmacPlanner2DSimple::createPath()` 先依据 `map_` 判定邻居可通行并计算基础移动代价，再可选累加 `getESDFPotentialCost()`。`MincoPlanner::configure()` 和 `rebuildModeDependentQueries()` 负责装配查询源。

### Data flow

- `map_ = globalQuery()`，用于先验地图搜索和 `mapToWorld()`。
- `esdf_query_ = dynamicQuery()`，只提供动态 ESDF 软代价。
- `getESDFPotentialCost()` 的返回值缓存在当前 `planning_id_` 下。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。ESDF 不得参与可通行性判断，其代价必须按单步距离缩放，避免地图分辨率改变软代价相对强度。

### Recommended modification boundary

仅修改软代价公式、无效查询缓存以及删除 `configure()` 中重复的 `setMap()`，不改变公开接口。

## Modifier Changes

### Files changed

- `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`

`smac_planner_2d_simple.hpp` 经检查无需修改，现有成员和接口已满足本次需求。

### Key changes

- ESDF potential 计算拆分为 `[0, 1]` 的指数衰减项及权重/上限处理后的无量纲倍率。
- 搜索累加改为 `step_cost * weighted_potential`。
- 动态查询失败或距离非有限时，缓存当前栅格的零 potential。
- 删除 `configure()` 后段重复设置 SMAC 全局查询的代码块。

### Behavior preserved

- 先验地图可通行判断、基础栅格代价和坐标转换。
- `map_ = globalQuery()`、`esdf_query_ = dynamicQuery()`。
- ROGMap 范围外只使用先验搜索代价。
- 现有参数、MINCO、validation 与控制发布逻辑。

### Behavior intentionally adjusted

归一化前：`g_next = g_current + step_cost * traversal_factor + weighted_potential`。

归一化后：`g_next = g_current + step_cost * traversal_factor + step_cost * weighted_potential`。

因此 `esdf_weight = 1.0` 时，障碍物附近最多增加约一倍基础移动代价，而不是固定增加约 `1.0` 路径代价。

### Notes

未新增参数、动态硬阻塞、TF 处理或距离阈值。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（未修改）
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可

构建命令：`colcon build --packages-select minco_planner --event-handlers console_direct+`

构建结果：通过；`Finished <<< minco_planner`，1 个包完成，退出码 0。仅有 ROGMap 既有头文件 warning。

### Issues found

未发现越界修改。工作区已有的 `sentry1.yaml` 修改属于用户，未触碰。

### Final result

PASS
