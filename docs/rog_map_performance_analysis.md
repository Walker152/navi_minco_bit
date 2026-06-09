# ROGMap 性能分析说明

本文说明 `planner_server.ros__parameters.MincoPlanner.rog_map.performance` 下新增的性能统计、CSV 和窗口 summary，用于定位 `/rog_map/field` 频率低、field 重建慢、projection 全量刷新过多、输入侧频率不足等问题。统计只做诊断，不改变 ROGMap 占据、projection、ESDF 或 planner 逻辑。

## 1. 如何开启性能分析

推荐调试配置：

```yaml
performance:
  enable: true
  csv_enable: false
  csv_path: /tmp/rog_map_performance.csv
  map_info_csv_path: /tmp/rog_map_info.csv
  detailed_enable: true
  detailed_csv_enable: true
  detailed_csv_path: /tmp/rog_map_perf_detailed.csv
  summary_csv_enable: true
  summary_csv_path: /tmp/rog_map_perf_summary.csv
  csv_flush_every_n: 30
  publish_enable: true
  topic: /rog_map/performance
  print_enable: true
  summary_rate: 1.0
```

`detailed_csv_enable` 每次有效 `updateMapInternal()` 写一行。`summary_csv_enable` 按 `summary_rate` 输出窗口统计。`csv_flush_every_n` 控制 flush 间隔，避免每行强制 flush 成为新瓶颈。`print_enable` 开启后终端会打印简洁窗口 summary。

## 2. 如何采集数据

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /rog_map/field
ros2 topic hz /rog_map/layer_value
tail -f /tmp/rog_map_perf_summary.csv
```

检查 CSV：

```bash
ls -lh /tmp/rog_map_perf_detailed.csv
ls -lh /tmp/rog_map_perf_summary.csv
head -n 2 /tmp/rog_map_perf_detailed.csv
head -n 2 /tmp/rog_map_perf_summary.csv
```

## 3. 如何判断瓶颈

### 输入频率不足

判断条件：`cloud_callback_hz` 或 summary 中 `cloud_callback_hz` 低于期望点云频率。若 `/cloud_registered` 本身低，则 ROGMap 不可能更高频更新。

### 有效更新不足

判断条件：`valid_update_hz << cloud_callback_hz`，同时 `dropped_cloud_no_odom_count` 或 `dropped_cloud_odom_timeout_count` 增长。优先检查 odom topic、时间戳、`odom_timeout` 和 executor 负载。

### projection 瓶颈

判断条件：`projection_total_time_ms` 占 `update_total_time_ms` 大头，或 summary 中 `avg_projection_time_ms` 明显高。继续看 `projection_refresh_reason`、`projection_scanned_voxel_estimate`、`projection_z_layers`。

### dirty 退化为 full refresh

判断条件：`projection_refresh_reason` 高频为 `full_dirty_over_ratio`、`full_geometry_changed`、`full_required` 或 `full_dirty_disabled`。若 `dirty_ratio` 长期很高，说明局部变化覆盖了大部分二维列；若 `dirty_column_enable=false`，则每帧都会更接近全量刷新。

### field EDT 瓶颈

判断条件：`field_time_ms` 高，且 `field_edt_positive_time_ms`、`field_edt_negative_time_ms` 或 `field_distance_fill_time_ms` 高。此时瓶颈在二维 ESDF 重建，而不是 projection 或 visualization。

### field_dirty 触发不足

判断条件：summary 中 `field_update_hz` 低，但 `field_skip_not_dirty_delta` 高，且 `layer_mask_changed=0` 经常出现。这说明 mask 没变，field 不更新是合理结果。

### field period 限制

判断条件：`field_skip_reason=period_not_ready` 或 summary 中 `field_skip_period_not_ready_delta` 高。此时 field 被 `field.update_rate` 限制，不是 EDT 慢。

### query copy 瓶颈

判断条件：`query_refresh_time_ms` 或 `query_copy_field_distances_time_ms` 高。当前 snapshot 会复制 layer value/type/height/confidence 和 field distances；若 field 没更新但仍频繁复制 distances，该列会暴露成本。

## 4. 推荐调参方向

- 输入慢：先检查点云发布端、QoS、CPU 占用和 `/cloud_registered` 频率。
- odom 对齐差：适当增大 `ros_callback.odom_timeout`，同时确认 odom 时间戳不是滞后数据。
- projection 慢：降低 `map_size` 或 z 范围，启用 `dirty_column_enable`，检查 `dirty_full_ratio` 是否过低。
- full refresh 过多：检查地图滑动阈值是否过小、dirty column 是否覆盖过大、是否频繁 geometry changed。
- field 慢：降低二维地图范围/分辨率，或降低 `field.update_rate`，但不要用统计代码修改 ESDF 数学逻辑。
- query copy 慢：观察 `query_copy_field_distances_time_ms`，必要时后续考虑 snapshot 复用或按 field sequence 避免重复复制。
- 可视化 topic 频率低：新版可视化只在有订阅者时构造和发布消息；用 `ros2 topic hz /rog_map/field` 或 RViz 订阅对应 topic 验证发布频率。

## 5. CSV 字段说明

Detailed CSV 关键字段：

| 字段 | 含义 |
| --- | --- |
| `cloud_callback_hz_window` | ROGMap cloud callback 实际接收点云频率。 |
| `valid_update_hz_window` | 真正进入 `updateMapInternal()` 的有效更新频率。 |
| `dropped_cloud_no_odom_count` | 因没有 odom 被丢弃的累计点云数。 |
| `dropped_cloud_odom_timeout_count` | 因 odom 超时被丢弃的累计点云数。 |
| `cloud_queue_delay_ms` | 点云 header stamp 到 callback 处理时刻的延迟。 |
| `odom_age_ms` | 当前有效点云使用的 odom 年龄。 |
| `raycast_used_point_count` | 实际参与 raycast 的点数。 |
| `raycast_skipped_near_count` | 因小于 `ray_range[0]` 跳过的点数。 |
| `raycast_skipped_far_count` | 因超过 `ray_range[1]` 被截断/视为非 hit 的点数。 |
| `projection_refresh_reason` | `none/full_geometry_changed/full_required/full_dirty_disabled/full_dirty_over_ratio/dirty_update/no_dirty`。 |
| `projection_scanned_voxel_estimate` | 估算扫描 voxel 数，full 为 cell_count*z_layers，dirty 为 expanded_dirty*z_layers。 |
| `layer_mask_changed` | 本帧 projection mask 是否变化。 |
| `layer_mask_diff_ratio` | mask 与上一帧相比的变化比例。 |
| `field_skip_reason` | `none/disabled/layer_empty/not_dirty/period_not_ready/unknown`。 |
| `field_edt_positive_time_ms` | `computeEDT2D(mask)` 耗时。 |
| `field_edt_negative_time_ms` | `computeEDT2D(inv_mask)` 耗时。 |
| `query_copy_field_distances_time_ms` | snapshot 复制 field distances 的耗时。 |

Summary CSV 关键字段：

| 字段 | 含义 |
| --- | --- |
| `cloud_callback_hz` | 窗口内 cloud callback 频率。 |
| `valid_update_hz` | 窗口内有效地图更新频率。 |
| `field_update_hz` | 窗口内 field 实际重建频率。 |
| `avg_update_total_time_ms` / `max_update_total_time_ms` | 总更新耗时均值/最大值。 |
| `avg_projection_time_ms` / `max_projection_time_ms` | projection 耗时均值/最大值。 |
| `avg_field_time_ms` / `max_field_time_ms` | field 重建耗时均值/最大值。 |
| `full_refresh_count_delta` | 窗口内 full refresh 次数。 |
| `dirty_update_count_delta` | 窗口内 dirty update 次数。 |
| `field_skip_not_dirty_delta` | 窗口内因 mask 未 dirty 跳过 field 次数。 |
| `field_skip_period_not_ready_delta` | 窗口内因 field 周期未到跳过次数。 |
| `avg_mask_diff_ratio` | 窗口内 mask 变化比例均值。 |
| `avg_dirty_ratio` | 窗口内 dirty column 比例均值。 |
