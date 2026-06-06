# Codex 最终收口任务：ROGMap + MincoPlanner 同进程指针架构、完整内部功能与 sentry1.yaml 参数加载

## 0. 当前最终决策

本工程当前不做 planner 独立化，不做 standalone rog_map_node，不做跨进程地图同步。

最终架构必须是：

```text
planner_server 进程
  └── MincoPlanner Nav2 plugin
        ├── 在 configure() 内创建 ROGMapROS
        ├── ROGMapROS 使用 planner_server 的 LifecycleNode / node interfaces
        ├── ROGMapROS 内部维护 ROGMap core
        ├── ROGMap core 维护 ProbMap / ProjectionLayer / DynamicLayer / QueryAdapter
        ├── MincoPlanner 持有 rog_map_ros_
        ├── MincoPlanner 持有 map_ = rog_map_ros_->queryInterface()
        ├── Smac / Astar / MincoOptimizer / Corridor / CollisionChecker 共用同一个 map_
        └── MapRegistry 只作为同进程兜底，不作为主架构
```

禁止实现或保留为主路径：

```text
1. standalone rog_map_node
2. sentry_planner_node
3. PlannerManager
4. MapSnapshotPublisher
5. MapQueryClient
6. 跨进程 snapshot topic
7. planner 通过 topic 查询 ROGMap
8. 各规划子模块各自 MapRegistry::get()
9. MincoPlanner 内部新建一个没有加入 executor 的普通 rclcpp::Node
```

目标是做成类似 SUPER 的同进程直接共享地图对象架构，但保留 `MapQueryInterface` 抽象，不让所有 planner 逻辑强依赖 `ROGMapROS` 具体类。

---

## 1. 清理旧架构：删除 PlannerManager / sentry_planner_node 路径

请从工程中删除或完全停用以下文件、target、install、launch 引用：

```text
planner_manager.cpp
planner_manager.hpp
sentry_planner_node.cpp
sentry_planner_node executable target
任何单独创建 rog_node_ / sentry_planner_rog_map 的逻辑
```

如果 CMakeLists.txt 中存在：

```cmake
add_executable(sentry_planner_node ...)
add_library(... planner_manager.cpp ...)
install(TARGETS sentry_planner_node ...)
```

全部删除。

最终系统只通过 Nav2 的 `planner_server` 加载 `minco_planner/MincoPlanner` plugin。

验收要求：

```text
grep -R "PlannerManager" src include CMakeLists.txt launch config
grep -R "sentry_planner_node" src include CMakeLists.txt launch config
```

除历史注释外，不应再存在有效引用。

---

## 2. MincoPlanner 作为唯一调度入口

### 2.1 MincoPlanner 成员

确保 `MincoPlanner` 中有且只有这条 ROGMap 主链路：

```cpp
std::shared_ptr<rog_map::ROGMapROS> rog_map_ros_;
std::shared_ptr<rog_map::MapQueryInterface> map_;
```

允许保留：

```cpp
void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);
```

`setRogMap()` 可选。如果保留，必须写完整：

```cpp
void MincoPlanner::setRogMap(const std::shared_ptr<rog_map::ROGMapROS> & rog_map)
{
  rog_map_ros_ = rog_map;
  if (rog_map_ros_) {
    setMap(rog_map_ros_->queryInterface());
  } else {
    setMap(nullptr);
  }
}
```

但最终主路径必须是 `configureRogMap()`，不是外部 `PlannerManager` 注入。

### 2.2 configureRogMap()

保留并完善：

```cpp
bool MincoPlanner::configureRogMap(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & plugin_prefix)
{
  if (!node) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Cannot configure ROGMap without planner_server LifecycleNode.");
    return false;
  }

  try {
    rog_map::Config rog_cfg;
    rog_cfg.loadFromRosNode(node, plugin_prefix + "rog_map");

    rog_map_ros_ = std::make_shared<rog_map::ROGMapROS>(node, rog_cfg);
    rog_map_ros_->init();  // 如果构造函数不自动 init，则必须显式调用

    setMap(rog_map_ros_->queryInterface());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Failed to configure ROGMap: %s", e.what());
    rog_map_ros_.reset();
    map_.reset();
    return false;
  }

  if (!map_) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap queryInterface is null.");
    rog_map_ros_.reset();
    return false;
  }

  rog_map::MapRegistry::set(map_);

  RCLCPP_INFO(
    logger_,
    "[MincoPlanner] ROGMap is created inside MincoPlanner plugin and shared by pointer.");
  return true;
}
```

如果当前 `ROGMapROS` 构造函数已经自动 `init()`，则不要重复 init；但必须在代码注释里明确生命周期：

```text
ROGMapROS constructor initializes subscriptions/timers/query interface.
```

或：

```text
ROGMapROS constructor only stores config; init() must be called explicitly.
```

### 2.3 configure() 调用顺序

`MincoPlanner::configure()` 中必须按这个顺序：

```text
1. 保存 parent/name/tf/costmap_ros
2. logger_ = node->get_logger()
3. prefix = name_ + "."
4. 读取 global_frame、基础参数
5. configureRogMap(node, prefix)
6. 如果 configureRogMap 失败，尝试 ensureMapAvailable() 兜底
7. 再创建 Smac / Astar / Optimizer / Corridor / CollisionChecker
8. 创建后立刻 setMap(map_)
9. 创建 FSM / timers
```

不要先创建 optimizer/corridor 后才创建 ROGMap，否则子模块可能拿到空地图。

### 2.4 ensureMapAvailable()

统一收敛所有 `MapRegistry::get()` 逻辑：

```cpp
bool MincoPlanner::ensureMapAvailable()
{
  if (map_) {
    return true;
  }

  auto map = rog_map::MapRegistry::get();
  if (map) {
    setMap(map);
    return true;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[MincoPlanner] MapQueryInterface unavailable: ROGMap was not created and MapRegistry is empty.");
  } else {
    RCLCPP_ERROR(logger_, "[MincoPlanner] MapQueryInterface unavailable.");
  }
  return false;
}
```

要求：

```text
1. makePlan / PlanGlobalPath / ReplanLocal / safetyTimerCallback / collision check 统一调用 ensureMapAvailable()
2. 不允许在多个函数里复制 MapRegistry::get()
3. MapRegistry 只用于兜底，不是主架构
```

### 2.5 setMap() 必须分发给全部子模块

`setMap()` 必须覆盖所有依赖 ROGMap 的模块：

```cpp
void MincoPlanner::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  map_ = map;

  if (smac_planner_) {
    smac_planner_->setMap(map_);
  }

  if (astar_planner_) {
    astar_planner_->setMap(map_);  // 如果 Astar 当前没有接口，请新增
  }

  if (minco_optimizer_) {
    minco_optimizer_->setMap(map_);
  }

  if (corridor_gen_) {
    corridor_gen_->setMap(map_);
  }

  if (collision_checker_) {
    collision_checker_->setMap(map_);
  }

  if (recovery_server_) {
    recovery_server_->setMap(map_);  // 如果 recovery 需要地图，请新增；否则不要硬加
  }
}
```

如果某些模块当前没有 `setMap()`，请补齐接口。所有影响“能不能走 / 能不能执行 / 会不会撞”的判断必须直接查 ROGMap，不允许只查静态 ESDF 或 Nav2 costmap。

---

## 3. ROGMapROS 必须使用 planner_server LifecycleNode

不要在 MincoPlanner 内部新建普通 node：

```cpp
auto rog_node = std::make_shared<rclcpp::Node>("rog_map");
```

这是错误架构，因为该 node 不一定在 Nav2 executor 中 spin，订阅/timer 可能不回调。

ROGMapROS 必须支持：

```cpp
ROGMapROS(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const rog_map::Config & cfg);
```

或支持 node interfaces：

```cpp
ROGMapROS(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & topics,
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr & timers,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const rog_map::Config & cfg);
```

优先实现第一种，简单可靠。

ROGMapROS 必须提供：

```cpp
std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const;
```

并确保返回的是 ROGMap core 内部 `QueryAdapter` 的 shared_ptr。

---

## 4. sentry1.yaml 参数加载最终形态

当前 `sentry1.yaml` 已经有：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["MincoPlanner"]

    MincoPlanner:
      plugin: "minco_planner/MincoPlanner"
      ...
```

但当前还缺完整 `MincoPlanner.rog_map` 段。请把所有 ROGMap 参数放到：

```yaml
planner_server:
  ros__parameters:
    MincoPlanner:
      rog_map:
        ...
```

不要放到顶层：

```yaml
rog_map:
  ros__parameters:
    ...
```

因为当前 ROGMap 是由 MincoPlanner plugin 在 planner_server 节点内创建并读取参数。

如果 plugin id 后续不是 `MincoPlanner`，而是例如 `GridBased`，则参数路径必须跟着 plugin id 变成：

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "minco_planner/MincoPlanner"
      rog_map:
        ...
```

因为代码里使用的是：

```cpp
const std::string prefix = name_ + ".";
rog_cfg.loadFromRosNode(node, prefix + "rog_map");
```

---

## 5. Config 参数加载要求

`rog_map::Config::loadFromRosNode()` 必须支持从 LifecycleNode 读取嵌套参数：

```cpp
void Config::loadFromRosNode(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & prefix);
```

读取路径示例：

```text
MincoPlanner.rog_map.frame_id
MincoPlanner.rog_map.resolution
MincoPlanner.rog_map.ros_callback.cloud_topic
MincoPlanner.rog_map.projection.terrain_enable
MincoPlanner.rog_map.field.interpolation
MincoPlanner.rog_map.performance.enable
MincoPlanner.rog_map.visualization.enable
```

要求：

```text
1. 每个参数必须 declare_parameter_if_not_declared
2. 缺省值来自 Config 当前默认值
3. bool/int/double/string/vector 全部支持
4. vector 参数支持 YAML list，例如 map_size: [12.0, 12.0, 1.5]
5. 参数加载后打印一条简洁 summary
6. 不要再要求 rog_map_config_path
7. 保留 loadFromYamlFile 兼容旧路径，但当前主路径是 loadFromRosNode
```

---

## 6. sentry1.yaml 中加入完整 rog_map 段

请在当前 `planner_server.ros__parameters.MincoPlanner` 下加入：

```yaml
      rog_map:
        frame_id: camera_init

        resolution: 0.05
        inflation_resolution: 0.05
        map_size: [12.0, 12.0, 1.5]
        fix_map_origin: [0.0, 0.0, 0.0]

        map_sliding:
          enable: true
          threshold: 2.0

        ros_callback:
          enable: true
          cloud_topic: /cloud_registered
          dense_cloud_topic: /cloud_registered_dense
          odom_topic: /aft_mapped_to_init
          odom_timeout: 0.08
          update_period_ms: 1
          use_dense_cloud: false

        raycasting:
          enable: true
          batch_update_size: 1
          ray_range: [0.30, 10.0]
          local_update_box: [10.0, 10.0, 1.5]
          unk_thresh: 0.70
          p_hit: 0.70
          p_miss: 0.70
          p_min: 0.12
          p_max: 0.97
          p_occ: 0.80
          p_free: 0.30
          parallel_enable: true
          num_threads: 4

        decay:
          enable: true
          keep_time: 0.35
          decay_time: 1.0
          active_list_enable: true

        projection:
          enable: true
          min_z: -0.15
          max_z: 0.80
          unknown_as_occupied: true
          min_observed_voxels: 2
          low_obstacle_height: 0.08
          obstacle_height: 0.16
          min_ratio: 0.35
          passable_cost: 50
          hysteresis_enable: true
          hysteresis_count: 2
          hole_fill_enable: true
          hole_fill_radius: 1
          hole_fill_min_occupied_neighbors: 5

          terrain_enable: true
          robot_body_z_min: 0.02
          robot_body_z_max: 0.30
          overhead_clearance_margin: 0.03
          surface_thickness: 0.08
          max_step_height: 0.10
          max_slope_deg: 18.0
          clearance_check_enable: true
          min_clearance_height: 0.30
          tunnel_wall_min_height: 0.18
          passable_as_free: true

        field:
          enable: true
          inflation_radius: 0.30
          max_distance: 3.0
          min_distance: -1.0
          clamp_distance: true
          smooth_grad_enable: false
          interpolation: quadratic
          update_rate: 20.0

        performance:
          enable: true
          csv_enable: false
          csv_path: /tmp/rog_map_performance.csv
          map_info_csv_path: /tmp/rog_map_info.csv
          publish_enable: true
          topic: /rog_map/performance
          print_enable: false
          summary_rate: 1.0
          dirty_column_enable: true
          dirty_full_ratio: 0.30
          parallel_raycast_enable: true
          raycast_num_threads: 4

        visualization:
          enable: true
          frame_id: camera_init
          rate: 5.0
          range: [10.0, 10.0, 1.2]
          lazy_publish: true

          occupied:
            enable: true
            topic: /rog_map/occupied
          unknown:
            enable: false
            topic: /rog_map/unknown
          inflated_occupied:
            enable: false
            topic: /rog_map/occupied_inflate
          inflated_unknown:
            enable: false
            topic: /rog_map/unknown_inflate
          frontier:
            enable: false
            topic: /rog_map/frontier

          layer_value:
            enable: true
            topic: /rog_map/layer_value
          layer_type:
            enable: true
            topic: /rog_map/layer_type
          layer_confidence:
            enable: false
            topic: /rog_map/layer_confidence
          layer_height:
            enable: true
            topic: /rog_map/layer_height

          field:
            enable: true
            topic: /rog_map/field
          decay_cells:
            enable: false
            topic: /rog_map/decay_cells
          map_bound:
            enable: true
            topic: /rog_map/map_bound
```

注意：

```text
1. 如果当前 LIO 没有 /cloud_registered_dense，就默认 use_dense_cloud=false。
2. cloud_topic 默认先用当前已有的 /cloud_registered。
3. 不要硬编码 cloud topic。
4. 不要硬编码 frame_id = world。
```

---

## 7. ROGMap 内部功能最终补齐

### 7.1 QueryAdapter 必须支持 quadratic

当前 DynamicLayer 里已经有 quadratic 插值，但 planner 通过 `MapQueryInterface::evaluate()` 调用的是 `QueryAdapter::evaluate()`。

因此必须在 `QueryAdapter::evaluate()` 里根据 snapshot 插值模式切换：

```cpp
if (snap->interpolation == InterpolationMode::QUADRATIC) {
  // 3x3 quadratic
} else {
  // bilinear
}
```

要求：

```text
1. QueryAdapter quadratic 和 DynamicLayer quadratic 行为一致
2. 边界不足 1 cell 时 fallback 到 bilinear
3. 3x3 任一点非 finite 时 fallback 到 bilinear 或返回 max_distance + zero grad
4. gradient 单位必须是 m/m，不能是 cell/pixel 单位
5. clamp_distance 开启时 dist clamp 到 [field_min_distance, field_max_distance]
6. dist 被 clamp 到边界时 grad 置零或保持有限安全值
7. evaluate 不允许输出 NaN / inf
```

建议把 bilinear/quadratic 插值函数抽成工具函数，DynamicLayer 和 QueryAdapter 共用，避免两份逻辑不一致。

### 7.2 dirty column 真正用于 ProjectionLayer 局部更新

当前 dirty column 只标记，不应继续每帧全量扫描二维 layer。

新增或完善：

```cpp
void ProjectionLayer::updateFull(...);

void ProjectionLayer::updateDirty(
  int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<int> & dirty_columns,
  bool force_full_refresh);
```

规则：

```text
1. geometry changed 时 full update
2. map sliding 后 full update
3. full_layer_refresh_required_ == true 时 full update
4. dirty_column_enable == false 时 full update
5. dirty list 为空时不更新 ProjectionLayer，但仍可 refreshQuery
6. dirty_column_enable == true 且 dirty list 非空时只更新 dirty columns
7. dirty columns 必须膨胀：
   dirty_radius = max(1, hole_fill_radius + 1)
   因为 hole fill / hysteresis / slope / step 依赖邻域
8. dirty 数量超过 dirty_full_ratio * width * height 时 full update
9. 未更新 cell 必须保留上一帧 CellData
10. 只有实际更新 ProjectionLayer 成功后才能 clearDirtyColumns()
```

`RuntimeStats` 增加并填写：

```text
dirty_column_count
dirty_expanded_column_count
full_layer_refresh_count
dirty_layer_update_count
```

### 7.3 field.update_rate 做成 dirty-aware

当前 field update rate 只是按时间降频。请补齐：

ROGMap 成员：

```cpp
double last_field_update_time_ = -std::numeric_limits<double>::infinity();
bool field_dirty_ = true;
uint64_t field_sequence_ = 0;
double last_field_stamp_ = 0.0;
```

规则：

```text
1. ProjectionLayer mask 发生变化时 field_dirty_ = true
2. full refresh / map sliding / load pcd 后 field_dirty_ = true
3. field.update_rate <= 0：dirty 后立即重建 field
4. field.update_rate > 0：只有超过周期才重建 field
5. 没到周期时，不重建 DynamicLayer，但 QueryAdapter snapshot 仍刷新 layer values/types/heights/confidence
6. snapshot 里的 distances 沿用上一次 field
7. snapshot 增加 field_sequence / field_stamp / field_stale
8. field_skipped_count 写入 RuntimeStats
```

不要实现复杂增量 EDT。本轮只做：

```text
ProjectionLayer dirty 局部更新
DynamicLayer 全量 EDT 按 dirty + rate 控制重建
```

### 7.4 ProjectionLayer 地形语义补齐

当前 height-only 分类不够。请补齐 RM 场景所需的 body-band / clearance / ground 语义。

扩展 `ColumnStats`：

```cpp
struct ColumnStats {
  int observed = 0;
  int occupied = 0;

  int occupied_in_body_band = 0;
  int occupied_below_body = 0;
  int occupied_above_body = 0;

  double min_z = inf;
  double max_z = -inf;

  double ground_z = NaN;
  double ceiling_z = inf;

  double body_band_min_z = inf;
  double body_band_max_z = -inf;

  double last_hit_time = 0.0;
  double last_update_time = 0.0;
};
```

扩展 `CellData`：

```cpp
struct CellData {
  CellType type;
  CellType raw_type;
  CellType pending_type;

  uint8_t value = 255;
  uint8_t mask = 1;

  uint8_t pending_count = 0;
  uint8_t stable_count = 0;

  float confidence = 0.0f;

  float min_z = NaN;
  float max_z = NaN;
  float height = 0.0f;
  float ratio = 0.0f;

  float ground_z = NaN;
  float ceiling_z = NaN;
  float slope_deg = 0.0f;
  float step_height = 0.0f;

  uint8_t traversable = 0;

  float last_hit_time = 0.0f;
  float last_update_time = 0.0f;
};
```

分类规则：

```text
UNKNOWN:
  observed < min_observed_voxels

FREE:
  observed 足够且 occupied == 0

PASSABLE:
  1. occupied > 0
  2. body band 内无不可跨越障碍
  3. surface_thickness 合格
  4. slope_deg <= max_slope_deg
  5. step_height <= max_step_height
  6. overhead clearance 足够

OCCUPIED:
  1. occupied_in_body_band > 0 且不可跨越
  2. 竖直墙体：height > tunnel_wall_min_height 且 ratio > min_ratio
  3. clearance 不足
  4. 洞口/顶梁阻挡 robot_body_z_min ~ robot_body_z_max 通行 band
```

RM 地形目标：

```text
15 度梯形高地斜坡：
  应为 PASSABLE 或 FREE，不应大面积 OCCUPIED。

300mm 高洞口：
  不允许因顶梁/洞口上沿存在于同一 xy column 就整列 OCCUPIED。
  必须判断 robot body band 是否真正被占据。

800mm 宽通道：
  左右墙 OCCUPIED，中间 FREE/PASSABLE。
  inflation_radius=0.30 时，中心线仍应有正距离，不应完全闭合。

竖直隧道墙：
  墙体 OCCUPIED，通道中心 FREE/PASSABLE。
```

### 7.5 decay / fading 一致性检查

确保：

```text
hitPointUpdate:
  更新 log odds
  更新 last_hit_time
  更新 last_update_time
  加入 active list
  状态变化时 updateCellState()
  标记 dirty column

missPointUpdate:
  更新 log odds
  更新 last_update_time
  状态变化时 updateCellState()
  标记 dirty column

applyDecay:
  遍历 active occupied cells
  now - last_hit_time > keep_time 后按 decay_rate 衰减
  状态变化时 updateCellState()
  标记 dirty column
  更新 decayed_count
```

`updateCellState()` 必须同步：

```text
InfMap
ESDF counter
frontier/free counter
dirty column
```

不能只改 `occupancy_buffer_`。

### 7.6 parallel raycasting 保持线程安全

如果当前已有 parallel raycasting，请检查并确保：

```text
1. 多线程阶段只写 thread-local hit/miss
2. merge 阶段统一去重
3. hit 优先于 miss
4. 共享 occupancy / InfMap / counter 只在串行阶段或安全分块更新
5. 统计 raycast_parallel_time / raycast_merge_time
```

---

## 8. PerformanceMonitor 独立类

新增：

```text
rog_map/performance_monitor.hpp
rog_map/performance_monitor.cpp
```

类名：

```cpp
class PerformanceMonitor
```

职责：

```text
1. 保存 RuntimeStats
2. 提供 scoped timer
3. 受 performance.enable 总开关控制
4. 受 csv_enable 控制 CSV
5. 受 publish_enable 控制 ROS topic 内容
6. 受 print_enable 控制终端 summary
```

替换 ROGMap / ProbMap 内散落的：

```text
time_consuming_
time_consuming_name_
writeTimeConsumingToLog()
writeMapInfoToLog()
直接打开 rm_performance_log.csv
直接打开 rm_info_log.csv
```

保留 wrapper 可以，但实际实现委托给 `PerformanceMonitor`。

当：

```yaml
performance:
  enable: false
```

时：

```text
不打开文件
不写 CSV
不发布性能 topic
不打印 summary
scoped timer 应接近零开销
```

统计字段至少包括：

```text
total_update_time
raycast_time
prob_update_time
inflation_time
input_point_count
cache_count
inflation_count
raycast_parallel_time
raycast_merge_time
hit_count
miss_count
decay_time
projection_time
field_time
query_refresh_time
occupied_count
unknown_count
passable_count
free_count
decayed_count
dirty_column_count
dirty_expanded_column_count
full_layer_refresh_count
dirty_layer_update_count
field_skipped_count
visualization_time
```

---

## 9. ROGMapVisualizer 独立类

新增：

```text
rog_map/rog_map_visualizer.hpp
rog_map/rog_map_visualizer.cpp
```

类名：

```cpp
class ROGMapVisualizer
```

职责：

```text
1. 创建所有 ROGMap 可视化 publisher
2. 管理 visualization timer
3. 根据 visualization.enable 总开关控制所有可视化
4. 根据子开关控制不同 topic
5. lazy_publish=true 时无订阅者不生成点云/栅格
6. header.frame_id 全部来自 visualization.frame_id
```

从 ROGMapROS 中迁移：

```text
VisualizeMap
vizCallback
occupied/unknown/frontier/layer/field/map_bound 发布逻辑
```

子 topic 开关：

```text
occupied
unknown
inflated_occupied
inflated_unknown
frontier
layer_value
layer_type
layer_confidence
layer_height
field
decay_cells
map_bound
```

要求：

```text
visualization.enable == false:
  不创建 timer，不发布任何可视化。

某个子项 enable == false:
  不创建或不发布该 topic。

lazy_publish == true:
  没有订阅者时跳过数据生成。
```

---

## 10. Planner 使用 ROGMap 的边界原则

静态 ESDF / Nav2 costmap 只能作为 guide，不作为最终安全判断。

原则：

```text
只影响“往哪边走”：
  可以用静态地图 / Nav2 costmap。

影响“能不能走 / 能不能执行 / 会不会撞”：
  必须用 ROGMap。
```

具体要求：

```text
1. Smac 搜索可继续使用 costmap 或 static ESDF 引导，但应支持 ROGMap 动态障碍约束
2. MincoOptimizer obstacle cost 必须查 map_->evaluate()
3. trajectory validation 必须查 ROGMap
4. safetyTimerCallback 必须查 ROGMap
5. collision check 必须查 ROGMap
6. corridor 生成如果需要动态安全边界，必须查 ROGMap
7. recovery / escape 判断如果涉及障碍距离，必须查 ROGMap
```

---

## 11. 生命周期清理

`MincoPlanner::cleanup()` 必须释放：

```cpp
on_set_parameters_callback_handle_.reset();

fsm_timer_.reset();
safety_timer_.reset();

fsm_.reset();
recovery_server_.reset();
planner_handle_.reset();

visualizer_.reset();

astar_planner_.reset();
smac_planner_.reset();
minco_optimizer_.reset();
corridor_gen_.reset();
collision_checker_.reset();

backup_opt_.reset();
yaw_opt_.reset();

opt_path_pub_.reset();
backup_path_pub_.reset();
odom_sub_.reset();

map_.reset();
rog_map_ros_.reset();
```

如果 `ROGMapROS` 有 `cleanup()` / `shutdown()`，在 reset 前调用。

不要关闭 planner_server node，不要调用 `rclcpp::shutdown()`。

---

## 12. 编译和运行验收

### 12.1 编译

必须通过：

```bash
colcon build --packages-select <rog_map_package> <minco_planner_package> --cmake-args -DCMAKE_BUILD_TYPE=Release
```

不得出现：

```text
undefined reference to MincoPlanner::setRogMap
找不到 PlannerManager
找不到 sentry_planner_node
ROGMapROS 构造函数不匹配
Config::loadFromRosNode 不匹配
```

### 12.2 架构验收

启动 Nav2 planner_server 后日志应出现：

```text
[MincoPlanner] ROGMap is created inside MincoPlanner plugin and shared by pointer.
```

并满足：

```text
1. 没有启动 standalone rog_map_node
2. 没有启动 sentry_planner_node
3. 没有 PlannerManager 日志
4. ROGMapROS 的 cloud/odom subscription 正常回调
5. map_ 非空
6. smac/minco/corridor/collision checker 共用同一个 MapQueryInterface 指针
```

### 12.3 参数验收

在 `sentry1.yaml` 中修改以下参数后应生效：

```text
MincoPlanner.rog_map.ros_callback.cloud_topic
MincoPlanner.rog_map.frame_id
MincoPlanner.rog_map.projection.terrain_enable
MincoPlanner.rog_map.field.interpolation
MincoPlanner.rog_map.field.update_rate
MincoPlanner.rog_map.performance.enable
MincoPlanner.rog_map.visualization.enable
```

### 12.4 QueryAdapter quadratic 验收

构造测试 grid：

```text
f(x, y) = x^2 + y^2
```

检查：

```text
dist ≈ x^2 + y^2
grad ≈ [2x, 2y]
```

并确认 planner 调用 `map_->evaluate()` 时走的是 quadratic，而不是只在 DynamicLayer 内部生效。

### 12.5 dirty column 验收

测试：

```text
1. 第一帧 full refresh
2. map sliding full refresh
3. 少量 hit/miss dirty update
4. dirty 数量超过 dirty_full_ratio 后 full refresh
5. field.update_rate=20 时，不应每个 cloud callback 都重建 EDT
```

统计字段应正常变化：

```text
dirty_column_count
dirty_expanded_column_count
full_layer_refresh_count
dirty_layer_update_count
field_skipped_count
```

### 12.6 RM 地形验收

用 bag 或构造点云验证：

```text
15 度斜坡：
  中心区域 PASSABLE/FREE，不大面积 OCCUPIED。

300mm 高洞口：
  robot body band 可通行时，不应被顶梁整列阻断。

800mm 宽通道：
  左右墙 OCCUPIED，中间 FREE/PASSABLE。
  inflation_radius=0.30 时中心仍可通过。

竖直隧道墙：
  墙体 OCCUPIED，通道中心 FREE/PASSABLE。
```

---

## 13. 不要做的事

```text
1. 不要重新引入 PlannerManager。
2. 不要重新引入 sentry_planner_node。
3. 不要创建额外 rog_node。
4. 不要实现跨进程 MapQueryClient。
5. 不要通过 topic 给 planner 传地图快照。
6. 不要让 ROGMap 参数放在顶层 rog_map.ros__parameters 作为当前主路径。
7. 不要让 ROGMap 硬编码 cloud topic。
8. 不要让 ROGMap 硬编码 frame_id。
9. 不要让 QueryAdapter 仍然只做 bilinear。
10. 不要只标记 dirty column 但 refreshLayers 仍每帧全量扫描。
11. 不要在 performance.enable=false 时仍写 CSV。
12. 不要在 visualization.enable=false 时仍创建大量 publisher/timer。
13. 不要让不同 planner 子模块持有不同 map 实例。
```

---

## 14. 最终交付内容

请提交：

```text
1. 删除 PlannerManager / sentry_planner_node 后的 CMake 和源码清理
2. MincoPlanner plugin 内部创建并维护 ROGMapROS
3. ROGMapROS LifecycleNode 构造与 queryInterface()
4. Config::loadFromRosNode 完整嵌套参数加载
5. sentry1.yaml 中 MincoPlanner.rog_map 完整参数段
6. QueryAdapter quadratic evaluate
7. ProjectionLayer dirty update
8. dirty-aware field update rate
9. terrain/body-band/clearance/ground/slope/step 语义
10. PerformanceMonitor
11. ROGMapVisualizer
12. cleanup 生命周期修正
13. 简短说明文档：
    - 当前为什么是 SUPER-like 同进程指针架构
    - ROGMap 生命周期由 MincoPlanner plugin 如何维护
    - MapRegistry 只作为什么兜底
    - ROGMap 参数在 sentry1.yaml 中的路径
    - quadratic 在 DynamicLayer 和 QueryAdapter 中如何一致
    - dirty column 如何触发局部 ProjectionLayer 更新
    - field.update_rate 如何避免频繁 EDT 重建
    - RM 四类地形如何通过参数与分类逻辑判断
```
