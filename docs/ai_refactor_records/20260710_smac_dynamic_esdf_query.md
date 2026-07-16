# SMAC 动态 ESDF 查询改造记录

## User Intent

仅为 SMAC 补充独立的动态 ESDF 软代价查询能力；保持先验地图为搜索地图，不增加动态硬阻塞、距离阈值、额外安全距离或动态起终点投影，并保持 MINCO、validation 与控制发布逻辑不变。

## Scope

- `smac_planner_2d_simple.hpp`
- `smac_planner_2d_simple.cpp`
- `minco_planner.cpp`
- 本改造记录

## Out of Scope

- SMAC 可通行性与搜索地图修改
- MINCO 优化、轨迹 validation、控制发布逻辑修改
- 参数新增或默认值调整
- 动态障碍硬阻塞与起终点投影

## Explorer Findings

### Files inspected

- `src/navigation/minco_planner/include/smac_search/smac_planner_2d_simple.hpp`
- `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`
- `src/perception/rog_map/include/rog_map/map_query_interface.hpp`
- `src/navigation/minco_planner/src/minco_core/components/map_query_adapters.cpp`

### Active logic path

`MincoPlanner::configure()` 创建并配置 SMAC；`rebuildModeDependentQueries()` 在地图查询重建后更新 SMAC 查询源；`SmacPlanner2DSimple::createPath()` 使用先验地图判断邻居可通行性，并可选累加 ESDF 势场代价。

### Data flow

- `map_`：来自 `globalQuery()`，负责搜索栅格、代价值、尺寸和 `mapToWorld()`。
- `esdf_query_`：来自 `dynamicQuery()`，仅负责世界坐标下的动态 ESDF 查询。
- 动态查询无效或范围外时返回零软代价，搜索继续使用先验地图。

### Risk notes

本次修改涉及比赛验证逻辑，采用最小改动策略。必须避免把 `dynamicQuery()` 用作 SMAC 搜索地图，也不能把 ESDF 结果用于邻居拒绝扩展。

### Recommended modification boundary

只新增独立查询成员与 setter、在两个查询装配点注入 `dynamicQuery()`，并将既有 ESDF 势场查询从 `map_` 切换到 `esdf_query_`。

## Modifier Changes

### Files changed

- `src/navigation/minco_planner/include/smac_search/smac_planner_2d_simple.hpp`
- `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`

### Key changes

- 新增 `esdf_query_` 与 `setESDFQuery()`；设置查询时重置 `planning_id_`。
- SMAC 创建和查询重建时保持 `map_ = globalQuery()`、`esdf_query_ = dynamicQuery()`。
- 世界坐标继续由 `map_->mapToWorld()` 生成，ESDF 距离改由 `esdf_query_->query()` 获取。
- 无效或非有限查询结果返回零软代价。

### Behavior preserved

- `use_esdf_cost` 关闭时的搜索行为。
- 先验地图的可通行性、栅格代价、尺寸与坐标转换职责。
- 原有 ESDF 参数及指数衰减、最大代价限制。
- MINCO、validation 与控制发布逻辑。

### Behavior intentionally adjusted

`use_esdf_cost` 开启时，SMAC 在动态查询有效范围内使用 ROGMap 动态 ESDF 软代价引导路径；范围外退化为纯先验地图搜索。

### Notes

未增加参数、TF 处理、动态硬阻塞、距离阈值或起终点投影。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本次未修改，参数保持原状）
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可

构建命令：`colcon build --packages-select minco_planner --event-handlers console_direct+`

构建结果：通过，构建、安装命令均返回 `0`。编译输出包含 ROGMap 既有头文件的 warning，无本次修改导致的 error。

### Issues found

未发现越界修改或动态 ESDF 参与硬阻塞的逻辑。

### Final result

PASS
