# ROGMap 参数说明

本文说明 `sentry1.yaml` 中 `planner_server.ros__parameters.MincoPlanner.rog_map` 参数的语义。当前集成方式为 MincoPlanner 进程内创建 ROGMap，并通过 `MapQueryInterface` 向全局搜索、MINCO 优化和轨迹安全检查提供二维 layer 与 ESDF 查询。

## 1. 总体数据流

```text
PointCloud2 + Odometry
        ↓
Raycasting 概率占据更新
        ↓
Decay 动态遗忘
        ↓
Projection 三维占据列转二维 layer
        ↓
Field 根据 layer mask 生成二维 ESDF
        ↓
QueryAdapter 向 MincoPlanner 提供 isFree / value / evaluate 查询
        ↓
Global search / Local MINCO optimization / Safety check
```

ROGMap 不是 Nav2 costmap。三维概率地图先根据点云和里程计更新，再由 projection 将每个 xy 栅格列压成 `UNKNOWN / FREE / PASSABLE / OCCUPIED`，最后由 projection 的 `mask` 生成二维 ESDF。

## 2. 坐标系与高度基准

`frame_id` 是 ROGMap 工作坐标系。`projection.min_z/max_z`、`robot_body_z_min/max`、净空和高程判断都按这个 frame 的 z 轴解释，不能默认 `z=0` 是地面。

当前 `sentry1.yaml` 配置为 `camera_init`。若 `camera_init` 近似在雷达高度，且雷达离地约 `0.207m`，地面可能约为 `z=-0.207m`。此时车体高度参数也要换算到 `camera_init`，例如车体下沿离地 `0.02m`、车体高度 `0.28m` 时，可参考 `robot_body_z_min≈-0.187`、`robot_body_z_max≈0.073`。

## 3. 基础地图参数

| 参数 | 单位 | 作用与调参方向 |
| --- | --- | --- |
| `frame_id` | frame | ROGMap 工作坐标系。影响点云融合、projection 高度解释、可视化和规划查询对齐。 |
| `resolution` | m/cell | 三维概率地图分辨率。越小越精细，但 raycasting、projection、ESDF 和内存开销越大。 |
| `inflation_resolution` | m/cell | 膨胀地图分辨率。通常与 `resolution` 一致，过粗会使膨胀边界粗糙。 |
| `map_size` | m | 局部地图窗口大小，不是全局地图大小。调大可看更远，计算量和内存上升。 |
| `fix_map_origin` | m | 固定地图原点/初始化参考。滑动地图开启时主要影响初始位置，非滑动模式下意义更明确。 |

## 4. map_sliding 参数

`map_sliding.enable` 控制局部地图是否随机器人位置滑动。开启后可保持机器人位于局部窗口内，关闭时地图边界固定。

`map_sliding.threshold` 单位 m，是触发滑动的位移阈值。阈值过小会导致频繁滑动、projection 和 ESDF 抖动；阈值过大时机器人可能靠近局部地图边界，规划查询范围变小。

## 5. ros_callback 参数

| 参数 | 单位 | 说明 |
| --- | --- | --- |
| `enable` | bool | 是否让 ROGMap 自己订阅点云/里程计并定时更新。 |
| `cloud_topic` | topic | 普通点云输入，当前为 `/cloud_registered`。 |
| `dense_cloud_topic` | topic | 稠密点云输入，配合 `use_dense_cloud` 使用。 |
| `odom_topic` | topic | 里程计输入，用于点云位姿匹配。 |
| `odom_timeout` | s | 点云匹配里程计的最大时间差。过小易丢点云，过大会使用滞后位姿。 |
| `update_period_ms` | ms | `updateCallback` 定时器周期，代码会把 `<=0` 修正为 `1ms`。 |
| `use_dense_cloud` | bool | 选择订阅普通点云或稠密点云。稠密点云更完整，但 CPU 占用更高。 |

## 6. raycasting 参数

Raycasting 负责把点云转成三维概率占据，不是二维 costmap 更新。

`p_hit` 表示命中点增加占据概率的强度；`p_miss` 表示射线穿过区域增加自由概率的强度；`p_min/p_max` 是概率上下限；`p_occ/p_free` 是 occupied/free 判定阈值。`unk_thresh` 与 unknown 判断相关，最终会影响 projection 看到的 `GridType`。

`ray_range=[min,max]` 单位 m。`min` 太大会忽略近处点，`max` 太大会增加计算量并引入远处噪声。`local_update_box` 单位 m，限制每次局部更新范围。`parallel_enable` 和 `num_threads` 控制并行 raycast，但当前代码后续还会读取 `performance.parallel_raycast_enable` 和 `performance.raycast_num_threads` 到同一变量，performance 项会覆盖 raycasting 项。

## 7. decay 参数

Decay 用于动态障碍遗忘。`enable=false` 会让旧占据更容易残留；`keep_time` 单位 s，表示观测后保持时间；`decay_time` 单位 s，表示从 occupied 衰减到 free 的时间尺度。时间过短会导致障碍闪烁，过长会让动态障碍残留。`active_list_enable` 开启后只衰减活跃 cell，通常降低计算量。

## 8. projection 参数

Projection 是三维 ROGMap 到二维 layer/ESDF mask 的关键环节。

`min_z/max_z` 单位 m，决定参与二维投影统计的 z 范围，基于 `frame_id`。在 `camera_init` 为雷达高度的场景中，若 `min_z` 高于真实地面，地面点可能不参与高程分析，导致坡度、台阶、地面高度判断异常。

`unknown_as_occupied` 控制 UNKNOWN 是否写成障碍 mask。为 true 时 UNKNOWN 会阻挡 ESDF 和规划；为 false 时 UNKNOWN 可能被当作可通行，适合局部动态规划或探索，但风险更高。

`min_observed_voxels` 是每个 xy 栅格列至少需要的已观测 voxel 数。过大产生大量 UNKNOWN，过小容易受噪声点影响。

`low_obstacle_height`、`obstacle_height`、`min_ratio` 用于低矮障碍、柱状障碍和墙体判断。`min_ratio` 与占据点数、高度和分辨率相关，用于区分稀疏噪声与实体障碍。

`terrain_enable` 开启后会综合判断车体碰撞带、顶部净空、坡度、台阶、表面厚度以及竖直墙/隧道墙。`robot_body_z_min/max` 是车体碰撞带上下界，必须基于 ROGMap frame。`clearance_check_enable` 开启后会检查顶部净空，`min_clearance_height` 是车体顶部额外净空，不是机器人总高度。

`max_slope_deg` 单位 degree，表示最大可通行坡度。对 23 度通过角底盘，建议先用 `20~22` 度留出点云噪声和姿态误差余量。`max_step_height` 单位 m，表示最大可跨越台阶，对 RM 哨兵可从 `0.04~0.06m` 试起。

`hysteresis_enable/count` 用于降低语义闪烁；`hole_fill_enable/radius/min_occupied_neighbors` 用于填补被障碍包围的小洞；`passable_as_free` 控制 PASSABLE 的 `layer_value` 是否写成 0，但 PASSABLE 的 `mask` 始终为 1。

## 9. field 参数

Field 是二维 dynamic field / ESDF。它不是由 `layer_value` 直接生成，而是由 projection 输出的 `mask` 生成。

代码中的核心关系是：

```text
final_distance = raw_distance - inflation_radius
```

因此 `field.inflation_radius` 越大，ESDF 数值整体越小，机器人越容易被认为靠近障碍。`max_distance/min_distance` 是正负距离截断范围，`clamp_distance` 控制是否截断。`interpolation` 可为 `bilinear` 或 `quadratic`；`quadratic` 通常梯度更平滑，但边界或邻域不足时会回退到双线性。`update_rate` 单位 Hz，实际用于限制 field 刷新频率。

## 10. performance 参数

Performance 参数用于性能统计、CSV 输出、dirty column 增量刷新和并行 raycast 配置。

`dirty_column_enable` 开启后 projection 可只刷新变化列及其邻域。`dirty_full_ratio` 是 dirty column 超过总列数后触发全量刷新的比例。`parallel_raycast_enable` 和 `raycast_num_threads` 会覆盖 raycasting 下同名语义配置。`csv_enable/csv_path/map_info_csv_path` 控制 CSV 文件输出。`publish_enable/topic` 控制性能消息发布。`print_enable/summary_rate` 当前只看到加载和进入 monitor 配置，未看到实际打印限频逻辑，疑似未完全生效。

## 11. visualization 参数

当前只保留 `enable`、`frame_id`、`rate`、`range` 四个参数。

`enable=false` 时不创建可视化 timer，也不发布任何可视化 topic，不影响地图和规划计算。`frame_id` 只影响可视化消息 header；为空时使用 ROGMap `frame_id`。`rate` 单位 Hz，`<=0` 时默认 5Hz。`range` 只限制调试消息范围。

所有可视化 topic 名称固定，且每个 topic 都会先检查是否存在订阅者，再构造和发布消息。

| 参数 | 用途 |
| --- | --- |
| `/rog_map/occupied` | 三维占据点，用于检查障碍物是否被正确建图。 |
| `/rog_map/unknown` | 三维未知体素，用于检查未知空间分布。 |
| `/rog_map/inflated_occupied` | 膨胀后的三维占据点。 |
| `/rog_map/inflated_unknown` | 膨胀后的未知体素。 |
| `/rog_map/frontier` | 探索边界；还依赖 `frontier_extraction_en`。 |
| `/rog_map/layer_value` | 二维投影代价值，可检查 FREE/PASSABLE/OCCUPIED/UNKNOWN 的最终 value。 |
| `/rog_map/layer_type` | 二维投影分类，可检查 UNKNOWN/FREE/PASSABLE/OCCUPIED 语义。 |
| `/rog_map/layer_confidence` | 每个二维列的观测置信度，可检查点云稀疏或 `min_observed_voxels` 过大。 |
| `/rog_map/layer_height` | 高度、地面高度或障碍高度相关可视化，用于调试高程分析。 |
| `/rog_map/field` | 二维 ESDF / dynamic field，可检查障碍距离是否合理。 |
| `/rog_map/decay_cells` | 正在衰减或遗忘的 cell。 |
| `/rog_map/map_bound` | 当前局部滑动地图边界。 |

旧的 `visualization.lazy_publish`、`visualization.*.enable`、`visualization.*.topic` 已废弃。

## 12. layer_type / layer_value / mask / ESDF 语义

```text
FREE:
  value = 0
  mask = 1

PASSABLE:
  value = 0 或 passable_cost
  mask = 1

OCCUPIED:
  value = 254
  mask = 0

UNKNOWN:
  value = 255 或 254
  mask 取决于 unknown_as_occupied
```

`layer_value` 大面积黑色通常表示 value=0，即 FREE 或 PASSABLE。这不代表障碍物很多，反而可能表示大面积可通行。

真正影响 ESDF 障碍距离的是 `mask`。`mask=0` 的 cell 会被 DynamicLayer 当作障碍；`mask=1` 的 cell 会被当作自由或可通行。

## 13. 常见异常现象与调参方向

ESDF 大面积低数值：检查 `field.inflation_radius` 是否过大、`mask=0` 是否过多、`unknown_as_occupied` 是否为 true、高程参数是否按 `camera_init` 修正。

`layer_value` 大面积黑色：先确认 raw value 是否为 0。若为 0，通常是 FREE/PASSABLE，不是障碍多。

地面被判障碍：检查 `projection.min_z` 是否覆盖真实地面、`robot_body_z_min/max` 是否按 ROGMap frame 修正、`surface_thickness` 和 `max_slope_deg` 是否过小。

坡道被判障碍：检查 `max_slope_deg`、`max_step_height`、地面点密度和高度基准。对 23 度通过角，建议先用 `20~22` 度。

动态障碍残留：检查 `decay.enable`、`keep_time`、`decay_time`、active list 和点云去畸变。

规划器总认为自己在障碍内：检查当前位置 `worldToMap`、`layer_value/type`、ESDF distance、ROGMap boundary、frame transform、`field.inflation_radius` 和 planner `safe_dist` 叠加是否过保守。

## 14. RM 哨兵推荐初值

已知参考：

```text
雷达离地高度：0.207m
机器人车体高度：约 0.280m
机器人宽度：约 0.600m
底盘通过角：23°
```

若 ROGMap 使用 `camera_init` 且其 z 原点近似在雷达高度，可参考但不要直接盲改：

```yaml
projection:
  min_z: -0.30
  max_z: 0.60
  robot_body_z_min: -0.187
  robot_body_z_max: 0.073
  overhead_clearance_margin: 0.03
  surface_thickness: 0.06
  max_step_height: 0.05
  max_slope_deg: 21.0
  clearance_check_enable: true
  min_clearance_height: 0.05
  tunnel_wall_min_height: 0.18
  passable_as_free: true

field:
  inflation_radius: 0.10
  max_distance: 4.0
  min_distance: -2.0
  clamp_distance: true
  interpolation: quadratic
```

若后续引入地面高度基准，则可改用更直观的地面系高度，例如 `robot_body_z_min=0.02`、`robot_body_z_max=0.28`。

## 15. 参数生效性检查表

- 确认 `Config::loadFromRosNode()` 日志中的 prefix 为 `MincoPlanner.rog_map`。
- 确认 `frame_id`、点云 frame、odom frame 和 planner frame 有可用 TF。
- 订阅 `/rog_map/layer_type`，检查 UNKNOWN/FREE/PASSABLE/OCCUPIED 是否符合预期。
- 订阅 `/rog_map/layer_value`，确认黑色是否为 value=0 的可通行区。
- 订阅 `/rog_map/field`，检查障碍附近距离是否因 `inflation_radius` 过小或过大而异常。
- 检查 `/rog_map/performance` 或 CSV 中 projection/field/raycast 时间，评估 dirty column 和并行 raycast 是否有效。
- 若修改 `performance.print_enable/summary_rate` 无终端输出变化，应按疑似未完全生效处理。
