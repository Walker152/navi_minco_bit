# 🗺️ ROGMap · Projection · ESDF

> 为高速局部规划定制的滚动占据地图：接收 Point-LIO 稠密点云，维护概率占据、语义投影层与有符号 ESDF，并向 MincoPlanner 提供低延迟进程内查询。

[返回项目主页](../../../README.md) · [Point-LIO](../Point-LIO/README.md) · [MINCO Planner](../../navigation/minco_planner/README.md)

## ✨ 模块定位

本仓库中的 ROGMap 不是独立地图节点：`MincoPlanner` 插件在配置阶段创建地图实例，并获取 `MapQueryInterface` 指针。地图仍通过 ROS 2 接收点云与里程计、发布可视化，但规划搜索、轨迹优化与安全检查可以直接查询内存中的地图状态。

主要能力：

- 以机器人为中心的三维滑动概率占据栅格。
- 基于 raycast 的 hit / miss 更新与可选时间衰减。
- 将三维观测投影为二维可通行性/地形层。
- dirty-column 增量维护，避免每周期全图重复投影。
- 有符号 ESDF，为 MINCO 障碍代价与安全检查提供距离和梯度。
- SensorData QoS `keep_last(1)` 接收最新稠密点云。
- 地图更新、投影、ESDF 与可视化频率可独立控制。

## 🧠 数据链路

```mermaid
flowchart LR
  C["/cloud_registered_full"] --> R[Range Filter / Raycast]
  O["/aft_mapped_to_init"] --> A[Active Window / Map Center]
  A --> V[3D Probabilistic Occupancy]
  R --> V
  V --> D[Dirty Columns]
  D --> P[2D Projection Layer]
  V --> E[Signed ESDF]
  P --> Q[MapQueryInterface]
  E --> Q
  Q --> M[Search / MINCO / Safety / Recovery]
```

## 🧱 地图层语义

| 层 | 作用 |
|---|---|
| Occupancy | 三维 free / occupied / unknown 概率状态 |
| Inflated Occupancy | 按机器人安全尺度膨胀后的障碍状态 |
| Projection Layer | 柱状投影得到的 free、passable、occupied、unknown 等二维语义 |
| ESDF | 到最近障碍的有符号距离及梯度 |
| Active Window | 随机器人移动的局部有效地图范围 |

投影层不是简单“取最高点”。它综合指定 Z 范围内的观测数量、表面高度变化、墙面/隧道判据、未知状态和迟滞保持，减少坡面、孔洞及稀疏回波导致的瞬时跳变。

## 📡 ROS 接口

### 输入

| Topic | 类型 | 说明 |
|---|---|---|
| `/cloud_registered_full` | `sensor_msgs/msg/PointCloud2` | `camera_init` 世界系稠密点云 |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 地图中心、传感器状态与超时判断 |

### 可视化输出

| Topic | 内容 |
|---|---|
| `/rog_map/occupied` | 占据点 |
| `/rog_map/occupied_raw` | 未膨胀占据点 |
| `/rog_map/unknown` | 未知体素 |
| `/rog_map/inflated` | 膨胀障碍 |
| `/rog_map/frontier` | 前沿区域 |
| `/rog_map/esdf` | ESDF 可视化 |
| `/rog_map/layer_value` | 动态与 PGM 静态先验融合后的二维障碍投影 |
| `/rog_map/layer_value_dynamic` | 仅在线三维感知生成的动态二维障碍投影 |
| `/rog_map/layer_value_static` | 仅 PGM 静态先验在当前 ROGMap 网格上的二维障碍投影 |
| `/rog_map/layer_type` | 四类投影结果；RViz Map 使用 `costmap` 配色显示 UNKNOWN/FREE/PASSABLE/OCCUPIED |
| `/rog_map/layer_confidence` | 分类置信度 |
| `/rog_map/layer_height_delta` | 柱内高度变化 |
| `/rog_map/field` | 势场/距离场诊断 |
| `/rog_map/decay_cells` | 衰减单元诊断 |
| `/rog_map/map_bound` | 当前滑动地图边界 |

## ⚙️ 关键配置

参数位于 `src/navigation/navi2_bringup/params/sentry1.yaml`：

```text
planner_server.ros__parameters.MincoPlanner.rog_map
```

### 地图几何

| 参数 | 当前典型值 | 说明 |
|---|---:|---|
| `resolution` | `0.05` m | 体素分辨率 |
| `inflation_resolution` | `0.05` m | 膨胀层分辨率 |
| `map_size` | `[10, 10, 1.5]` m | 滑动地图物理尺寸 |
| `map_sliding.threshold` | `0.2` | 触发中心滑动的位移阈值 |
| `map_sliding.center_offset_enable` | `true` | 启用地图中心偏置 |
| `map_sliding.center_offset` | `[0, 0.20, 0]` | 地图窗口相对 odom 参考点的中心偏移 |

`center_offset` 用于移动局部地图窗口，不是雷达外参，也不改变点云坐标。它通常与车体几何中心相关，但必须根据实际参考点定义标定。

### 概率更新与 raycast

| 参数组 | 作用 |
|---|---|
| `raycasting.ray_range` | 限制有效回波最小/最大距离 |
| `raycasting.local_update_box` | 单帧允许更新的局部范围 |
| `raycasting.p_hit`, `p_miss` | 命中/穿越的概率更新参数 |
| `raycasting.p_min`, `p_max` | 占据概率上下界 |
| `raycasting.p_occ`, `p_free` | occupied/free 判定阈值 |
| `ros_callback.odom_timeout` | 里程计过期保护 |
| `ros_callback.update_period_ms` | 地图更新周期 |

动态物体残影优先从点云时间、raycast 穿越、active window 和 decay 是否启用排查，不应只增大 miss 权重。

### ⏳ 衰减

当前比赛配置 `decay.enable: true`，并使用 `keep_time: 0.8 s` 与 `clear_time: 1.2 s`；`active_list_enable: false` 表示不只维护活跃列表。启用衰减时应理解为：近期命中的单元先保持，之后回到 free 语义；它不能替代可靠的空闲射线观测。

### 📏 投影高度与雷达安装高度

当前典型投影范围为：

```yaml
projection:
  scan_z_min_abs: -0.2
  scan_z_max_abs: 1.5
```

这两个值位于 ROGMap 的 `camera_init`/局部地图坐标定义中。若初始姿态近似水平，可按以下关系理解：

```text
相对雷达 Z = 障碍物世界高度 - 雷达安装高度
```

因此雷达安装越高，地面对应的相对 Z 越负。参数调整建议：

1. 测量雷达光心到底盘接触地面的高度 `h_lidar`。
2. 在静止点云中确认地面峰值是否约为 `-h_lidar`。
3. `z_min` 应排除大部分地面噪声，但保留需要识别的坡面/台阶结构。
4. `z_max` 应覆盖对车体有碰撞意义的障碍高度。
5. 同时检查 `map_size.z`：当前投影跨度可能大于局部 Z 尺寸，源码会裁剪到有效索引，并不意味着超出地图的高度仍被观测。

不要仅凭“地图障碍少”放宽 Z 范围；先确认 Point-LIO frame、雷达高度和车体俯仰/横滚补偿。

### 投影分类

| 参数组 | 影响 |
|---|---|
| `min_observed_voxels` | 柱被视为已观测所需证据 |
| `surface_height_delta_max` | 平整表面高度变化阈值 |
| `wall_height_delta_min`, `wall_occupancy_ratio_min` | 墙面判据 |
| `tunnel_height_delta_*`, `tunnel_occupancy_ratio_max` | 低矮/夹层地形分类判据 |
| `passable_cost` | 可通行但有代价区域的规划代价 |
| `hysteresis_count`, `obstacle_hold_time` | 分类迟滞与保持 |
| `mask_filter_en` / `fill_occ_min` / `denoise_occ_max` | 二维 8 邻域补洞与孤立障碍去噪 |

### ESDF 与性能

| 参数 | 说明 |
|---|---|
| `field.max_distance`, `min_distance` | 距离场截断范围 |
| `field.interpolation` | 插值方式，当前为 `quadratic` |
| `field.update_rate` | ESDF 更新频率，典型为 50 Hz |
| `performance.dirty_column_enable` | 仅更新受影响的投影列 |
| `performance.dirty_full_ratio` | dirty 比例过高时转为全量刷新的阈值 |
| `performance.parallel_raycast_enable` | 是否并行 raycast |
| `performance.raycast_num_threads` | raycast 并行线程数 |

## 🚀 启动与检查

ROGMap 随 `planner_server` 中的 MincoPlanner 配置创建。启动完整导航后检查：

```bash
ros2 topic hz /cloud_registered_full
ros2 topic hz /rog_map/occupied
ros2 topic echo /rog_map/map_bound --once
ros2 param list /planner_server | grep rog_map
```

RViz 调试建议依次打开原始点云、`occupied_raw`、`inflated`、`layer_type` 和 `esdf`，避免只看最终轨迹反推地图问题。

## 🛠️ 常见问题

| 现象 | 优先检查 |
|---|---|
| 地图完全为空 | 点云 topic/frame、QoS、odom 超时、raycast 距离 |
| 地图随机器人漂移 | Point-LIO 外参/去畸变、点云是否已在 `camera_init` |
| 地面被判为障碍 | 雷达高度、`projection.scan_z_min_abs`、姿态与地面噪声 |
| 高处障碍消失 | `scan_z_max_abs`、`map_size.z` 和局部地图中心 Z |
| 动态障碍残留 | 点云时序、miss ray、decay 开关与保持时间 |
| ESDF 与占据层不一致 | ESDF 更新频率、dirty 更新和膨胀参数 |
| 规划器报告查询不可用 | ROGMap 是否由插件成功创建、生命周期与 registry 状态 |

## 🗂️ 关键源码

- `src/rog_map/rog_map.cpp`：地图配置、点云/里程计回调与更新入口。
- `include/rog_map_ros/rog_map_ros2.hpp`：ROS 2 回调、QoS、timer 与接口实现。
- `src/rog_map_ros/rog_map_ros1.cpp`：当前构建使用的 ROS 2 接口编译入口（文件名为历史沿用）。
- `src/rog_map/rog_map_visualizer.cpp`：地图可视化。
- `src/rog_map/projection_layer.cpp`：二维投影分类与增量更新。
- `src/rog_map/esdf_map.cpp`：ESDF 维护。
- `include/rog_map/`：地图数据结构和 `MapQueryInterface`。
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp`：插件内创建与共享方式。

## 📚 延伸阅读

上游点云性能链路见 [Point-LIO](../Point-LIO/README.md)，地图查询如何进入搜索、优化与恢复见 [MincoPlanner](../../navigation/minco_planner/README.md)。
