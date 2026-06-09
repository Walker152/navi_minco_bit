# ROGMap 参数审计报告

## 1. 已补齐注释的文件列表

- `src/navigation/navi2_bringup/params/sentry1.yaml`
- `src/perception/rog_map/src/rog_map/projection_layer.cpp`
- `src/perception/rog_map/src/rog_map/field_layer.cpp`
- `src/perception/rog_map/src/rog_map/rog_map.cpp`
- `src/perception/rog_map/src/rog_map/query_adapter.cpp`
- `src/navigation/minco_planner/src/minco_core/components/trajectory_safety_checker.cpp`

## 2. 已新增或修改的文档列表

- `docs/rog_map_parameters.md`
- `docs/rog_map_parameter_audit.md`

## 3. 参数生效性检查表

表中 YAML 路径均省略共同前缀 `planner_server.ros__parameters.MincoPlanner.rog_map`。

| 参数 | YAML 路径 | 代码读取位置 | 实际作用 | 是否生效 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `frame_id` | `frame_id` | `config.hpp::load("frame_id")` | ROGMap 工作坐标系 | 生效 | 不再被 `visualization.frame_id` 覆盖 |
| `resolution` | `resolution` | `config.hpp::load("resolution")` | 三维概率地图分辨率 | 生效 | 影响 raycasting/projection/field 尺寸 |
| `inflation_resolution` | `inflation_resolution` | `config.hpp::load("inflation_resolution")` | 膨胀地图分辨率 | 生效 | 用于膨胀地图和邻域步长 |
| `map_size` | `map_size` | `config.hpp::loadVec3("map_size")` | 局部地图窗口大小 | 生效 | `resetMapSize()` 转成栅格尺寸 |
| `fix_map_origin` | `fix_map_origin` | `config.hpp::loadVec3("fix_map_origin")` | 固定原点/初始化位置 | 生效 | 非滑动模式意义更明确 |
| `map_sliding.enable` | `map_sliding.enable` | `config.hpp::load("map_sliding.enable")` | 控制局部地图滑动 | 生效 | `ROGMap::init()` 根据该值选择滑动/固定 |
| `map_sliding.threshold` | `map_sliding.threshold` | `config.hpp::load("map_sliding.threshold")` | 滑动触发阈值 | 生效 | 由 sliding map 逻辑使用 |
| `ros_callback.enable` | `ros_callback.enable` | `config.hpp::load("ros_callback.enable")` | 是否创建 ROS 订阅和更新定时器 | 生效 | `ROGMapROS::initializeRos()` 使用 |
| `ros_callback.cloud_topic` | `ros_callback.cloud_topic` | `config.hpp::load("ros_callback.cloud_topic")` | 普通点云订阅 topic | 生效 | `use_dense_cloud=false` 时使用 |
| `ros_callback.dense_cloud_topic` | `ros_callback.dense_cloud_topic` | `config.hpp::load("ros_callback.dense_cloud_topic")` | 稠密点云订阅 topic | 生效 | `use_dense_cloud=true` 时使用 |
| `ros_callback.odom_topic` | `ros_callback.odom_topic` | `config.hpp::load("ros_callback.odom_topic")` | 里程计订阅 topic | 生效 | `ROGMapROS::initializeRos()` 使用 |
| `ros_callback.odom_timeout` | `ros_callback.odom_timeout` | `config.hpp::load("ros_callback.odom_timeout")` | 点云-里程计时间匹配阈值 | 生效 | cloud callback 中用于丢弃超时 odom |
| `ros_callback.update_period_ms` | `ros_callback.update_period_ms` | `config.hpp::load("ros_callback.update_period_ms")` | ROGMap 更新定时器周期 | 生效 | `ROGMapROS::createWallTimer()` 使用，`<=0` 修正为 1 |
| `ros_callback.use_dense_cloud` | `ros_callback.use_dense_cloud` | `config.hpp::load("ros_callback.use_dense_cloud")` | 选择普通/稠密点云 | 生效 | 决定订阅 `cloud_topic` 或 `dense_cloud_topic` |
| `raycasting.enable` | `raycasting.enable` | `config.hpp::load("raycasting.enable")` | 是否执行概率 raycasting | 生效 | `updateProbMap()` 使用 |
| `raycasting.batch_update_size` | `raycasting.batch_update_size` | `config.hpp::load("raycasting.batch_update_size")` | 点云攒批大小 | 生效 | `<=0` 修正为 1 |
| `raycasting.ray_range` | `raycasting.ray_range` | `config.hpp::loadVec2("raycasting.ray_range")` | 射线最小/最大距离 | 生效 | 转成平方距离用于过滤 |
| `raycasting.local_update_box` | `raycasting.local_update_box` | `config.hpp::loadVec3("raycasting.local_update_box")` | 局部 raycast 更新盒 | 生效 | `resetMapSize()` 转成半盒栅格 |
| `raycasting.unk_thresh` | `raycasting.unk_thresh` | `config.hpp::load("raycasting.unk_thresh")` | unknown 判断阈值 | 生效 | 影响 GridType 判断 |
| `raycasting.p_hit` | `raycasting.p_hit` | `config.hpp::load("raycasting.p_hit")` | 命中 log-odds 增量来源 | 生效 | 转成 `l_hit` |
| `raycasting.p_miss` | `raycasting.p_miss` | `config.hpp::load("raycasting.p_miss")` | 射线穿过 log-odds 来源 | 生效 | 转成 `l_miss` |
| `raycasting.p_min` | `raycasting.p_min` | `config.hpp::load("raycasting.p_min")` | log-odds 下限 | 生效 | 转成 `l_min` |
| `raycasting.p_max` | `raycasting.p_max` | `config.hpp::load("raycasting.p_max")` | log-odds 上限 | 生效 | 转成 `l_max` |
| `raycasting.p_occ` | `raycasting.p_occ` | `config.hpp::load("raycasting.p_occ")` | occupied 阈值 | 生效 | 转成 `l_occ` |
| `raycasting.p_free` | `raycasting.p_free` | `config.hpp::load("raycasting.p_free")` | free 阈值 | 生效 | 转成 `l_free` |
| `raycasting.parallel_enable` | `raycasting.parallel_enable` | `config.hpp::load("raycasting.parallel_enable")` | 并行 raycast 开关 | 被覆盖后生效 | 随后 `performance.parallel_raycast_enable` 读取到同一变量 |
| `raycasting.num_threads` | `raycasting.num_threads` | `config.hpp::load("raycasting.num_threads")` | raycast 线程数 | 被覆盖后生效 | 随后 `performance.raycast_num_threads` 读取到同一变量 |
| `decay.enable` | `decay.enable` | `config.hpp::load("decay.enable")` | 是否执行动态遗忘 | 生效 | `ROGMap::updateMapInternal()` 调用 `applyDecay()` |
| `decay.keep_time` | `decay.keep_time` | `config.hpp::load("decay.keep_time")` | 观测保持时间 | 生效 | `ProbMap::applyDecay()` 使用 |
| `decay.decay_time` | `decay.decay_time` | `config.hpp::load("decay.decay_time")` | 衰减时间尺度 | 生效 | 生成 `decay_rate` |
| `decay.active_list_enable` | `decay.active_list_enable` | `config.hpp::load("decay.active_list_enable")` | 是否只衰减活跃 cell | 生效 | `ProbMap::applyDecay()` 分支使用 |
| `projection.enable` | `projection.enable` | `config.hpp::load("projection.enable")` | 是否生成二维 layer | 生效 | 写入 `layer_en` |
| `projection.min_z` | `projection.min_z` | `config.hpp::load("projection.min_z")` | 投影 z 下界 | 生效 | `ROGMap::refreshLayers()` 转为 z index |
| `projection.max_z` | `projection.max_z` | `config.hpp::load("projection.max_z")` | 投影 z 上界 | 生效 | `ROGMap::refreshLayers()` 转为 z index |
| `projection.unknown_as_occupied` | `projection.unknown_as_occupied` | `config.hpp::load("projection.unknown_as_occupied")` | UNKNOWN value/mask 语义 | 生效 | `ProjectionLayer::applyValueAndMask()` 使用 |
| `projection.min_observed_voxels` | `projection.min_observed_voxels` | `config.hpp::load("projection.min_observed_voxels")` | 列可信观测数 | 生效 | `classifyCell()` 使用 |
| `projection.low_obstacle_height` | `projection.low_obstacle_height` | `config.hpp::load("projection.low_obstacle_height")` | 低矮障碍阈值 | 生效 | `classifyCell()` 使用 |
| `projection.obstacle_height` | `projection.obstacle_height` | `config.hpp::load("projection.obstacle_height")` | 实体障碍高度阈值 | 生效 | `classifyCell()` 使用 |
| `projection.min_ratio` | `projection.min_ratio` | `config.hpp::load("projection.min_ratio")` | 占据密度/高度比例阈值 | 生效 | `classifyCell()` 使用 |
| `projection.passable_cost` | `projection.passable_cost` | `config.hpp::load("projection.passable_cost")` | PASSABLE 代价值 | 生效 | `applyValueAndMask()` 使用，取值 clamp 到 0..252 |
| `projection.hysteresis_enable` | `projection.hysteresis_enable` | `config.hpp::load("projection.hysteresis_enable")` | 语义迟滞开关 | 生效 | `ProjectionLayer::applyHysteresis()` 使用 |
| `projection.hysteresis_count` | `projection.hysteresis_count` | `config.hpp::load("projection.hysteresis_count")` | 类型切换确认次数 | 生效 | `applyHysteresis()` 使用 |
| `projection.hole_fill_enable` | `projection.hole_fill_enable` | `config.hpp::load("projection.hole_fill_enable")` | 小洞填充开关 | 生效 | `ProjectionLayer::applyHoleFill()` 使用 |
| `projection.hole_fill_radius` | `projection.hole_fill_radius` | `config.hpp::load("projection.hole_fill_radius")` | 小洞填充半径 | 生效 | 也影响 dirty 扩展邻域 |
| `projection.hole_fill_min_occupied_neighbors` | `projection.hole_fill_min_occupied_neighbors` | `config.hpp::load("projection.hole_fill_min_occupied_neighbors")` | 填洞障碍邻居阈值 | 生效 | `applyHoleFill()` 使用 |
| `projection.terrain_enable` | `projection.terrain_enable` | `config.hpp::load("projection.terrain_enable")` | 地形可通行分析开关 | 生效 | `classifyCell()` 和 `updateTerrainNeighborhood()` 使用 |
| `projection.robot_body_z_min` | `projection.robot_body_z_min` | `config.hpp::load("projection.robot_body_z_min")` | 车体碰撞带下界 | 生效 | `refreshLayers()` 扫描列时分类统计 |
| `projection.robot_body_z_max` | `projection.robot_body_z_max` | `config.hpp::load("projection.robot_body_z_max")` | 车体碰撞带上界 | 生效 | `refreshLayers()` 扫描列时分类统计 |
| `projection.overhead_clearance_margin` | `projection.overhead_clearance_margin` | `config.hpp::load("projection.overhead_clearance_margin")` | 顶部净空余量 | 生效 | `classifyCell()` 使用 |
| `projection.surface_thickness` | `projection.surface_thickness` | `config.hpp::load("projection.surface_thickness")` | 可通行表面厚度 | 生效 | `classifyCell()` 使用 |
| `projection.max_step_height` | `projection.max_step_height` | `config.hpp::load("projection.max_step_height")` | 最大可通过台阶 | 生效 | `classifyCell()` 和邻域地形检查使用 |
| `projection.max_slope_deg` | `projection.max_slope_deg` | `config.hpp::load("projection.max_slope_deg")` | 最大可通过坡度 | 生效 | `classifyCell()` 和邻域地形检查使用 |
| `projection.clearance_check_enable` | `projection.clearance_check_enable` | `config.hpp::load("projection.clearance_check_enable")` | 顶部净空检查开关 | 生效 | `classifyCell()` 使用 |
| `projection.min_clearance_height` | `projection.min_clearance_height` | `config.hpp::load("projection.min_clearance_height")` | 车体顶部额外净空 | 生效 | `classifyCell()` 使用 |
| `projection.tunnel_wall_min_height` | `projection.tunnel_wall_min_height` | `config.hpp::load("projection.tunnel_wall_min_height")` | 竖直墙/隧道墙阈值 | 生效 | `classifyCell()` 使用 |
| `projection.passable_as_free` | `projection.passable_as_free` | `config.hpp::load("projection.passable_as_free")` | PASSABLE value 是否写 0 | 生效 | `applyValueAndMask()` 使用，mask 仍为 1 |
| `field.enable` | `field.enable` | `config.hpp::load("field.enable")` | 是否生成二维 ESDF | 生效 | `ROGMap::refreshLayers()` 使用 |
| `field.inflation_radius` | `field.inflation_radius` | `config.hpp::load("field.inflation_radius")` | ESDF 距离膨胀半径 | 生效 | `DynamicLayer::rebuild()` 执行 `raw - inflation_radius` |
| `field.max_distance` | `field.max_distance` | `config.hpp::load("field.max_distance")` | ESDF 正向截断 | 生效 | `DynamicLayer::rebuild/evaluate()` 使用 |
| `field.min_distance` | `field.min_distance` | `config.hpp::load("field.min_distance")` | ESDF 负向截断 | 生效 | `DynamicLayer::rebuild/evaluate()` 使用 |
| `field.clamp_distance` | `field.clamp_distance` | `config.hpp::load("field.clamp_distance")` | 是否截断 ESDF | 生效 | `DynamicLayer` 和 `QueryAdapter` 使用 |
| `field.interpolation` | `field.interpolation` | `config.hpp::load("field.interpolation")` | ESDF 插值方式 | 生效 | `parseInterpolationMode()` 后用于 field 和 snapshot |
| `field.update_rate` | `field.update_rate` | `config.hpp::load("field.update_rate")` | field 刷新频率上限 | 生效 | `ROGMap::refreshLayers()` 中按周期限速 |
| `performance.enable` | `performance.enable` | `config.hpp::load("performance.enable")` | 性能统计总开关 | 生效 | `PerformanceMonitor` 和可视化发布使用 |
| `performance.csv_enable` | `performance.csv_enable` | `config.hpp::load("performance.csv_enable")` | 是否写性能 CSV | 生效 | `PerformanceMonitor::configure()` 使用 |
| `performance.csv_path` | `performance.csv_path` | `config.hpp::load("performance.csv_path")` | 性能 CSV 路径 | 生效 | `PerformanceMonitor::configure()` 使用 |
| `performance.map_info_csv_path` | `performance.map_info_csv_path` | `config.hpp::load("performance.map_info_csv_path")` | 地图信息 CSV 路径 | 生效 | `ROGMap::init()` 写 map info |
| `performance.publish_enable` | `performance.publish_enable` | `config.hpp::load("performance.publish_enable")` | 性能 topic 发布开关 | 生效 | `ROGMapVisualizer` 和 `vizCallback()` 使用 |
| `performance.topic` | `performance.topic` | `config.hpp::load("performance.topic")` | 性能消息 topic | 生效 | `ROGMapVisualizer::configure()` 使用 |
| `performance.print_enable` | `performance.print_enable` | `config.hpp::load("performance.print_enable")` | 预期为终端打印开关 | 疑似未完全生效 | 进入 `PerformanceConfig`，未看到实际打印调用 |
| `performance.summary_rate` | `performance.summary_rate` | `config.hpp::load("performance.summary_rate")` | 预期为摘要频率 | 疑似未完全生效 | 进入 `PerformanceConfig`，未看到限频逻辑 |
| `performance.dirty_column_enable` | `performance.dirty_column_enable` | `config.hpp::load("performance.dirty_column_enable")` | projection 增量刷新开关 | 生效 | `ROGMap::refreshLayers()` 使用 |
| `performance.dirty_full_ratio` | `performance.dirty_full_ratio` | `config.hpp::load("performance.dirty_full_ratio")` | dirty 超比例全量刷新阈值 | 生效 | `ROGMap::refreshLayers()` 使用 |
| `performance.parallel_raycast_enable` | `performance.parallel_raycast_enable` | `config.hpp::load("performance.parallel_raycast_enable")` | 并行 raycast 覆盖项 | 生效 | 覆盖 `raycasting.parallel_enable` |
| `performance.raycast_num_threads` | `performance.raycast_num_threads` | `config.hpp::load("performance.raycast_num_threads")` | raycast 线程覆盖项 | 生效 | 覆盖 `raycasting.num_threads` |
| `visualization.enable` | `visualization.enable` | `config.hpp::load("visualization.enable")` | 可视化总开关 | 生效 | `ROGMapROS::initializeRos()` 使用 |
| `visualization.frame_id` | `visualization.frame_id` | `config.hpp::load("visualization.frame_id")` | 可视化消息 frame | 生效 | 仅影响可视化消息 header；为空时回退到 ROGMap `frame_id` |
| `visualization.rate` | `visualization.rate` | `config.hpp::load("visualization.rate")` | 可视化发布频率 | 生效 | 读取到 `viz_time_rate`，`<=0` 时回退到 5Hz |
| `visualization.range` | `visualization.range` | `config.hpp::loadVec3("visualization.range")` | 可视化范围 | 生效 | `vizCallback()` boxSearch 使用 |
| `visualization.lazy_publish` | `visualization.lazy_publish` | 已删除 | 懒发布开关 | 已废弃 | 现在所有可视化 topic 都固定检查订阅数后再构造消息 |
| `visualization.*.enable` | 各可视化子项 `enable` | 已删除 | 单 topic 发布开关 | 已废弃 | 统一由 `visualization.enable` 控制可视化 timer 和 publisher |
| `visualization.*.topic` | 各可视化子项 `topic` | 已删除 | 单 topic 名称 | 已废弃 | topic 名称改为代码固定值 |

## 4. 疑似未生效参数列表

- `performance.print_enable`：配置进入 `PerformanceConfig`，但未看到实际打印性能摘要的调用。
- `performance.summary_rate`：配置进入 `PerformanceConfig`，但未看到按该频率限流打印或发布的逻辑。
- 旧的 `visualization.lazy_publish`、`visualization.*.enable`、`visualization.*.topic` 已废弃；当前可视化统一使用固定 topic，并在构造消息前检查订阅数。

## 5. 当前参数对 RM 哨兵机器人的合理性评价

- `frame_id=camera_init` 合理，但所有 projection 高度必须按 `camera_init` 解释。若雷达离地 `0.207m`，当前地面大约应在 `z≈-0.207m`。
- `projection.min_z=-0.25` 能覆盖地面附近，较合理；`max_z=0.80` 能覆盖车体和低顶障碍。
- `robot_body_z_min=-0.2`、`robot_body_z_max=0.1` 与雷达离地高度换算后的车体带接近，方向合理。
- `min_clearance_height=0.05` 比默认 `0.30` 更适合 280mm 高度级别的车体顶部额外净空。
- `max_slope_deg=20.0` 对 23 度通过角留有余量，合理。
- `max_step_height=0.5` 对 RM 哨兵底盘明显偏大，可能把不可跨越台阶判为 PASSABLE。建议后续实测后改到 `0.04~0.06m` 附近。
- `field.inflation_radius=0.10` 作为初值合理；需结合 planner `safe_dist`，两者会叠加保守性。
- `unknown_as_occupied=false` 适合更开放的局部动态规划，但未知区域会更冒险；正式比赛场景需结合传感器盲区评估。

## 6. 后续建议修改项

仅列建议，不在本任务中自动执行：

- 将 `projection.max_step_height` 从 `0.5m` 调整到接近 RM 哨兵实测跨越能力的范围，例如先试 `0.05m`。
- 明确 `raycasting.parallel_enable` 与 `performance.parallel_raycast_enable` 的优先级，或删除重复配置之一，避免调参误解。
- 为 `performance.print_enable/summary_rate` 补充实际打印或限频逻辑，或在配置中标注为保留项。
- 可视化 topic 已固定，调试时直接订阅 `/rog_map/occupied`、`/rog_map/layer_value`、`/rog_map/field` 等标准 topic。
- 增加启动时参数摘要日志，打印 projection 高度范围、车体碰撞带、field inflation 和 safe_dist，降低坐标基准误配风险。
