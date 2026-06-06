# ROGMap + MincoPlanner 当前项目总结

## 1. 总目标

当前项目目标是复现/改造中科大 RoboWalker 2025 哨兵导航方案中的动态感知与规划部分，重点是：

* 基于 ROGMap 构建机器人中心动态 occupancy map；
* 增加 parallel raycasting；
* 增加 fading / decay 动态遗忘；
* 从 3D occupancy 投影到 2D ProjectionLayer；
* 构建 2D signed ESDF；
* 通过 QueryAdapter 给 planner 提供统一查询接口；
* MincoPlanner 使用 ROGMap 作为动态安全约束；
* 维持 Nav2 plugin 形态，不单独做 planner 主节点；
* 架构尽量类似 SUPER：同进程直接共享地图指针。

当前暂时不处理或不作为本轮重点：

* Batch-LIWO 完整复现；
* 轮速融合；
* 控制器；
* 稠密点云发布与注入；
* 跨进程 MapSnapshot / MapQueryClient；
* 独立 rog_map_node；
* 独立 sentry_planner_node / PlannerManager。

---

## 2. 当前最终架构决策

已经确定采用：

```text
planner_server 进程
  └── MincoPlanner Nav2 plugin
        ├── 在 configure() 内创建 ROGMapROS
        ├── ROGMapROS 使用 planner_server 的 LifecycleNode
        ├── ROGMapROS 内部维护 ROGMap core
        ├── MincoPlanner 持有 rog_map_ros_
        ├── MincoPlanner 持有 map_ = rog_map_ros_->queryInterface()
        ├── Smac / Astar / MincoOptimizer / Corridor 共用同一个 map_
        └── MapRegistry 只作为同进程 fallback
```

不再采用：

```text
standalone rog_map_node
sentry_planner_node
PlannerManager
MapSnapshotPublisher
MapQueryClient
跨进程地图同步
planner 通过 topic 订阅地图
```

之前有一次误把历史旧索引中的 `PlannerManager / sentry_planner_node` 当作当前源码残留，后来已修正。按最新上传文件来看，当前有效架构中没有这些作为主路径。

---

## 3. MincoPlanner 对 ROGMap 的调用方式

最新 `minco_planner.cpp` 中，`MincoPlanner::configureRogMap()` 已经是正确架构：

```cpp
rog_map::Config rog_cfg;
rog_cfg.loadFromRosNode(node, plugin_prefix + "rog_map");
rog_map_ros_ = std::make_shared<rog_map::ROGMapROS>(node, rog_cfg);
setMap(rog_map_ros_->queryInterface());
rog_map::MapRegistry::set(map_);
```

也就是说：

* ROGMap 在 MincoPlanner plugin 内部创建；
* ROGMap 与 planner 同进程；
* planner 通过 `MapQueryInterface` 查询地图；
* `MapRegistry` 只是兜底，不是主架构。

`setMap()` 会把同一个 `map_` 分发给：

```text
smac_planner_
astar_planner_
minco_optimizer_
corridor_gen_
```

因此当前已经是 SUPER-like 的进程内地图指针共享状态。

---

## 4. sentry1.yaml 参数状态

最新 `sentry1.yaml` 中，ROGMap 参数路径已经正确放在：

```yaml
planner_server:
  ros__parameters:
    MincoPlanner:
      rog_map:
        ...
```

这与代码中的：

```cpp
const std::string prefix = name_ + ".";
rog_cfg.loadFromRosNode(node, prefix + "rog_map");
```

匹配。

当前 yaml 中已经包含：

```text
frame_id
resolution
inflation_resolution
map_size
fix_map_origin
map_sliding
ros_callback
raycasting
decay
projection
field
performance
visualization
```

关键设置包括：

```yaml
ros_callback:
  cloud_topic: /cloud_registered
  dense_cloud_topic: /cloud_registered_dense
  use_dense_cloud: false

projection:
  terrain_enable: true
  robot_body_z_min: 0.02
  robot_body_z_max: 0.30
  max_slope_deg: 18.0
  max_step_height: 0.10

field:
  interpolation: quadratic
  update_rate: 20.0

performance:
  dirty_column_enable: true
  dirty_full_ratio: 0.30
```

所以当前参数加载路径已经基本完备。
但因为 `use_dense_cloud: false`，当前还不是稠密点云输入。

---

## 5. ROGMap 当前功能进度

当前 ROGMap 已经形成完整主链路：

```text
3D ProbMap / Occupancy
  → parallel raycasting
  → hit / miss update
  → fading / decay
  → dirty column
  → ProjectionLayer
  → DynamicLayer 2D signed ESDF
  → QueryAdapter snapshot
  → MincoPlanner MapQueryInterface
```

### 5.1 ROGMap init

`ROGMap::init()` 当前会创建：

```text
ProjectionLayer
DynamicLayer
QueryAdapter
PerformanceMonitor
```

并将 `query_` 注册到 `MapRegistry`。

### 5.2 主更新流程

当前主流程为：

```text
updateRobotState
updateProbMap
applyDecay
refreshLayers
refreshQuery
PerformanceMonitor stats
```

这已经符合动态地图前端主体链路。

---

## 6. 已完成或基本完成的模块

### 6.1 parallel raycasting

当前已经实现并行 raycasting：

* OpenMP 条件满足时进入 parallel；
* 每个线程使用 thread-local hit/miss；
* merge 阶段统一去重；
* hit 优先于 miss；
* 避免多线程直接写 occupancy buffer。

判断：基本完成。

### 6.2 fading / decay

当前 hit/miss/decay 会：

* 更新 log odds；
* 更新 `last_hit_time` / `last_update_time`；
* active list 管理；
* 状态变化时调用 `updateCellState()`；
* 同步 inflation / ESDF counter / frontier counter；
* 标记 dirty column。

判断：基本完成。

### 6.3 ProjectionLayer / 地形语义

当前 ProjectionLayer 已经支持：

```text
UNKNOWN
FREE
PASSABLE
OCCUPIED
```

并且不再只是 height-only，而是加入：

```text
occupied_in_body_band
occupied_below_body
occupied_above_body
ground_z
ceiling_z
body_band_min_z / max_z
slope_deg
step_height
traversable
```

理论上可以处理：

```text
15° 梯形坡：
  通过 max_slope_deg / max_step_height / surface_thickness 判为 PASSABLE/FREE。

竖直隧道墙：
  body_band_blocked + height + ratio 判为 OCCUPIED。

800mm 通道：
  左右墙 OCCUPIED，中间 FREE/PASSABLE，2D ESDF 提供通道距离。

300mm 高洞口：
  通过 body-band / ceiling_z 判断，不再简单因为顶梁存在就整列 OCCUPIED。
```

判断：算法结构已具备，但必须通过 bag/实地调参验证。

### 6.4 dirty column

当前已经实现：

* dirty columns 标记；
* `ProjectionLayer::updateFull()`；
* `ProjectionLayer::updateDirty()`；
* dirty column 膨胀；
* dirty ratio 超阈值时 full update；
* 统计 dirty count / expanded count / full refresh count / dirty update count。

判断：主体完成。

### 6.5 field update rate

当前 field update 已经 dirty-aware：

* Projection mask 变化后 `field_dirty_ = true`；
* 根据 `field.update_rate` 控制 DynamicLayer 全量 EDT 重建；
* 没到周期时沿用旧 distances；
* snapshot 中有 field stale / sequence / stamp 语义；
* 统计 `field_skipped_count`。

判断：主体完成。

### 6.6 DynamicLayer 2D signed ESDF

当前 DynamicLayer 使用 2D EDT：

* 对 mask 做 EDT；
* 对 inverse mask 做 EDT；
* 得到正负 signed distance；
* 减去 inflation radius；
* 支持 clamp；
* 支持 bilinear / quadratic mode。

判断：完成。

### 6.7 QueryAdapter quadratic

之前短板是 DynamicLayer 有 quadratic，但 planner 查询仍是 bilinear。
最新 `query_adapter.cpp` 已经补齐：

* `sampleBilinear()`；
* `sampleQuadratic()`；
* `snap->interpolation == QUADRATIC` 时优先二次插值；
* fallback 到 bilinear；
* clamp 后保证 grad 有限。

判断：完成。
说明：当前是 3x3 quadratic Lagrange 型插值，不是严格最小二乘二次曲面拟合，但工程上可以缓解 ESDF 梯度震荡。

### 6.8 PerformanceMonitor

已经独立出 `PerformanceMonitor`：

* enable/csv/publish/print 开关；
* scoped timer；
* CSV 输出；
* RuntimeStats。

判断：基本完成。

### 6.9 ROGMapVisualizer

已经独立出 `ROGMapVisualizer` 管理 publisher/timer：

* visualization 总开关；
* 各子 topic 开关；
* occupied / unknown / layer / field / decay_cells / performance / map_bound 等 publisher。

判断：基本完成。
注意：如果 `ROGMapROS` 里仍残留大量 fill/publish 逻辑，那么可视化还不是完全剥离，但不影响导航功能主链路。

---

## 7. MincoPlanner 与 ROGMap 耦合状态

当前 planner 侧：

* `MincoPlanner` 内部创建 `ROGMapROS`；
* `map_ = rog_map_ros_->queryInterface()`；
* `setMap(map_)` 分发给子模块；
* Smac 支持 `setMap()`；
* Smac 可用 `map_->evaluate()` 做 ESDF potential；
* MincoOptimizer 已接收 `map_`；
* Corridor 已接收 `map_`；
* Astar 已接收 `map_`；
* 轨迹碰撞/安全检查应优先查 ROGMap。

设计原则：

```text
静态地图 / Nav2 costmap：
  只做 guide / 搜索引导。

ROGMap：
  用于动态障碍、安全距离、轨迹验证、优化约束。
```

判断：基本达到目标架构。

---

## 8. 当前仍未完成或需要验证的部分

### 8.1 稠密点云注入未完成

当前 yaml：

```yaml
use_dense_cloud: false
cloud_topic: /cloud_registered
dense_cloud_topic: /cloud_registered_dense
```

说明当前默认仍使用 `/cloud_registered`。

`laserMapping.cpp` 当前主要发布的是 `feats_down_world` 对应的 `/cloud_registered`，还没有真正将完整 `feats_undistort` 转换为 dense world cloud 并发布 `/cloud_registered_dense`。

这是当前最大缺口。

影响：

* ProjectionLayer 地形语义稳定性受限；
* 15°坡、300mm 洞口、800mm 通道的判断会受稀疏点云影响；
* ROGMap 内部算法完成度高，但输入不足会影响实机表现。

### 8.2 Batch-LIWO / 轮速融合未完成

这不属于当前 ROGMap 改造范围。
中科大完整系统还包括 Batch-LIWO、轮速融合、底盘观测器、控制器等，当前没有完整复现。

### 8.3 实机 / bag 验证未完成

当前只从代码结构判断，尚未看到：

* colcon build 结果；
* ros2 参数实际加载日志；
* bag 回放；
* layer_type / layer_value / field 可视化结果；
* RM 场地测试；
* Minco 轨迹优化实际避障效果。

所以当前可以说“理论链路已具备”，不能说“实车稳定性已证明”。

---

## 9. 当前复现进度评分

大致判断：

```text
ROGMap 内部链路：约 90%
MincoPlanner 进程内指针架构：约 90%
sentry1.yaml 参数路径：约 90%
ProjectionLayer 地形语义：约 80%~85%
dirty column + field update rate：约 85%~90%
QueryAdapter quadratic：约 90%
PerformanceMonitor / Visualizer：约 75%~85%
稠密点云输入：未完成
完整中科大系统复现：未完成
RM 场地理论动态导航分析：可以开始
实车比赛级稳定性：还需验证
```

---

## 10. 当前最终判断

除稠密点云注入以外，当前 ROGMap + MincoPlanner 已经从“骨架”进入“可理论分析和 bag 回放验证”的阶段。

当前已经具备：

```text
动态 occupancy
fading / decay
parallel raycasting
ProjectionLayer 地形语义
2D signed ESDF
quadratic interpolation
dirty column
field.update_rate
MapQueryInterface
MincoPlanner 同进程直接持有 ROGMapROS
sentry1.yaml 参数加载
```

当前还不能称为“完整中科大系统复现”，因为还缺：

```text
稠密点云注入
Batch-LIWO / 轮速融合
控制器闭环
实机验证
bag 回放验证
参数调优
```

但如果问题限定为：

```text
除稠密点云注入以外，ROGMap + MincoPlanner 动态地图导航前端是否具备？
```

答案是：**基本具备。**

---

## 11. 下一轮优先任务建议

下一轮最建议继续做这几件事：

### 11.1 先做工程确认

```bash
grep -R "PlannerManager\|sentry_planner_node\|sentry_planner_rog_map" src include CMakeLists.txt launch
```

如果无有效引用，即可确认没有旧架构残留。

然后：

```bash
colcon build --packages-select <rog_map_pkg> <minco_planner_pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 11.2 做 bag 回放验证

重点看：

```text
/rog_map/layer_type
/rog_map/layer_value
/rog_map/layer_height
/rog_map/field
/rog_map/performance
/opt_path
/backup_path
```

验证：

```text
动态障碍出现 → OCCUPIED / ESDF 变小
动态障碍移走 → decay 后清除
坡面 → PASSABLE/FREE
竖直墙 → OCCUPIED
通道中心 → 正 ESDF 距离
轨迹优化 → 避开动态障碍
```

### 11.3 再做稠密点云注入

目标：

```text
feats_undistort
  → 当前 EKF/LIO 状态
  → world/camera_init
  → /cloud_registered_dense
  → ROGMap use_dense_cloud=true
```

这是后续提升地形识别稳定性的关键。

### 11.4 最后做 RM 场地参数调优

重点参数：

```text
projection.robot_body_z_min
projection.robot_body_z_max
projection.overhead_clearance_margin
projection.min_clearance_height
projection.max_slope_deg
projection.max_step_height
projection.surface_thickness
projection.tunnel_wall_min_height
field.inflation_radius
field.update_rate
decay.keep_time
decay.decay_time
raycasting.ray_range
```

---

## 12. 下一轮对话开头建议

下一轮可以这样开头：

“以下是上一轮总结。请基于这个上下文继续分析最新代码/继续生成 Codex prompt/继续做 bag 验证方案。当前目标是保持 MincoPlanner 作为 Nav2 plugin，由 MincoPlanner 内部创建并持有 ROGMapROS，ROGMap 与 planner 同进程共享 MapQueryInterface，不做 standalone rog_map_node，也不做跨进程 MapQueryClient。除稠密点云注入外，ROGMap 主体链路已经基本完成，下一步重点是工程编译确认、bag 回放验证和稠密点云注入。”
