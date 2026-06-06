# 历史对话压缩总结：中科大哨兵 ROGMap 感知复现方案

## 1. 总体目标

我正在复现中科大 RoboWalker 2025 哨兵技术报告中的导航系统。当前：

* 规划器和控制器部分已经基本复现成功。
* 感知和定位部分尚未完全完成。
* 希望使用 **ROGMap** 为规划器提供动态地图。
* 希望具备狭窄空间 / 地形跨越能力。
* 希望在无先验地图或静态地图有误差的情况下，仍能依靠实时感知判断真实可通行性。

核心思想：

```text
静态地图 / Nav2 costmap：
  只能作为全局拓扑引导，不能作为安全约束。

ROGMap：
  作为真实局部环境来源，负责局部可通行判断、ESDF、碰撞检测、安全检查。
```

---

## 2. 里程计与点云输入问题

当前使用普通 PointLIO，存在两个主要问题：

1. MID360 点云频率约 10Hz，IMU 频率 200Hz，里程计更新受点云帧率限制。
2. PointLIO 输出的 `/cloud_registered` 是经过较大尺度降采样后的世界系点云，约 0.5m leaf size，太稀疏，无法给 ROGMap 提供足够的栅格观测，容易造成地形空洞。

讨论结论：

* 暂时不做完整 Batch-LIWO。
* 暂时不做轮速接口。
* 第一阶段应保持 PointLIO 主定位链路不动，只新增一个给 ROGMap 使用的稠密点云输出：

```text
/lio/cloud_registered_dense
```

来源应为：

```text
feats_undistort
  → 使用当前 EKF 状态转换到 world/camera_init
  → 发布 dense undistorted world cloud
```

注意：

```text
LIO 匹配仍然使用降采样点云。
ROGMap 使用稠密去畸变世界系点云。
```

可选快速提高频率：

```text
开启 Livox cut frame：
  cut_frame_init = true
  cut_frame_num = 5
  point_filter_num = 1
```

完整 Batch-LIO / Batch-LIWO 放到后续阶段。

---

## 3. ROGMap 复现目标

中科大报告中感知部分的关键点：

```text
ROGMap 3D Occupancy Grid
  + ROS2
  + parallel raycasting
  + fading / decay
  + 2D ESDF
  + 高程分析 / 狭窄地形处理
```

我们当前要复现的是 ROGMap 感知前端，不是 planner。

ROGMap 最终内部链路应为：

```text
3D Occupancy Grid
  → fading / decay 动态遗忘
  → ProjectionLayer 高程投影
  → DynamicLayer 2D signed ESDF
  → QueryAdapter snapshot
  → MapQueryInterface 只读查询
```

---

## 4. 为什么不用 ROGMap 原生 3D ESDF 作为主 ESDF

ROGMap 原生 ESDF 是 3D 几何距离场，适合空中机器人或 3D 避障。

哨兵底盘规划需要的是：

```text
机器人在平面上能不能走
底盘 footprint 到不可通行区域的距离
低矮障碍是否可跨越
```

所以不能直接使用原版 3D ESDF 作为主规划距离场。

正确链路是：

```text
3D Occupancy
  → z-column 高程分析
  → 2D value / mask
  → 2D ESDF
```

其中：

```text
value:
  给局部搜索 / 查询使用，类似 cost buffer。

mask:
  给 DynamicLayer / ESDF 使用。
  mask = 0 表示 obstacle seed
  mask = 1 表示 free
```

---

## 5. 当前 ROGMap 已经实现的骨架

最新代码中已经看到以下结构：

```text
ProjectionLayer
DynamicLayer
ESDFUtils
QueryAdapter
MapRegistry
```

ROGMap 初始化中已经创建：

```cpp
layer_ = std::make_shared<ProjectionLayer>();
field_ = std::make_shared<DynamicLayer>();
query_ = std::make_shared<QueryAdapter>();
MapRegistry::set(query_);
```

ROGMap 主流程已经接近：

```text
updateRobotState()
setUpdateTime()
updateProbMap()
applyDecay()
refreshLayers()
refreshQuery()
```

ProjectionLayer 当前已经实现：

```text
UNKNOWN
FREE
PASSABLE
OCCUPIED
```

并根据：

```text
observed voxel 数量
occupied voxel 数量
min_z
max_z
height = max_z - min_z
ratio = ((occupied + 1) * resolution) / max(height, resolution)
```

进行分类。

DynamicLayer 当前已经支持：

```cpp
updateFromMask(...)
```

内部逻辑为：

```text
mask → EDT → signed distance → inflation radius
```

QueryAdapter 当前已经提供：

```text
worldToMap
mapToWorld
value
values
copyValues
isValid
isFree
evaluate
snapshot
```

---

## 6. 当前 ROGMap 仍需继续改进的重点

目前 ROGMap 还没有完整复现中科大感知效果。后续需要补齐以下内容。

### 6.1 fading / decay 完整闭环

现在代码里有 `applyDecay(now)` 调用，但需要确认并补齐完整闭环。

必须做到：

```text
hit:
  更新 log odds
  更新 last_hit_time
  更新 last_update_time
  加入 active occupied list

miss:
  更新 log odds
  更新 last_update_time

decay:
  遍历 active occupied cells
  若 now - last_hit_time > keep_time:
      按 decay_rate 衰减 log odds
      若状态从 occupied 变 free/unknown:
          同步更新 InfMap / CounterMap / Inflation
          标记对应 column dirty 或触发 layer refresh
```

绝对不能只改 `occupancy_buffer_` 而不更新 inflation/counter，否则 3D map 清了，但 2D layer / ESDF 仍可能残留旧障碍。

### 6.2 parallel raycasting

ROGMap 需要支持多线程 raycasting，以便后续处理 dense cloud。

原则：

```text
parallel section:
  每个线程独立 raycast
  生成 thread-local hit_ids / miss_ids

merge section:
  合并 hit/miss
  去重
  hit 优先于 miss
  串行或安全分块更新概率地图
```

不要在多线程里直接写共享的 occupancy buffer、hit_cnt、miss_cnt、InfMap 或 CounterMap。

### 6.3 ProjectionLayer 稳定化

当前简单高程分类可以跑，但还需要增强稳定性。

需要加：

```text
temporal hysteresis:
  防止 FREE / OCCUPIED / PASSABLE 抖动。

small hole filling:
  修补由点云稀疏造成的小 unknown/free 洞。

confidence:
  记录观测置信度，例如 observed / min_observed_voxels。
```

当前暂时不做复杂地形检测，比如 slope、ground band、step model。

### 6.4 DynamicLayer / ESDF evaluate 稳定化

当前 ESDF 通过 EDT 构建，基本正确。

需要增强：

```text
越界保护
NaN / inf 保护
distance clamp
far distance 处理
gradient 异常保护
```

预留插值模式：

```text
bilinear
quadratic  // 后续扩展
```

中科大报告里提到狭窄地形存在 ESDF 梯度震荡，他们用了二次插值和两次优化策略。当前阶段可以只预留接口，不强制完成 quadratic。

### 6.5 QueryAdapter snapshot 安全

当前 QueryAdapter 已有 snapshot 思路，但仍需增强：

```text
MapSnapshot 增加：
  sequence
  stamp

推荐外部使用：
  snapshot()
  copyValues()

弱化裸指针：
  const unsigned char* values()
```

因为 `values()` 返回裸指针，在 snapshot 更新后可能出现生命周期风险。可以保留兼容，但要注释说明，优先用 `copyValues()` 或 `snapshot()`。

### 6.6 Debug 可视化输出

需要增加 ROS2 debug topic：

```text
/rog_map/layer/value
/rog_map/layer/type
/rog_map/layer/height
/rog_map/layer/confidence
/rog_map/field/distance
/rog_map/debug/decay_cells
/rog_map/debug/performance
```

没有这些 topic，很难判断问题来自：

```text
输入点云太稀疏
3D occupancy 没打中
高程分类错
mask 错
ESDF 错
query 坐标错
decay 没清干净
```

### 6.7 参数体系整理

建议整理成：

```yaml
projection:
  enable: true
  min_z: -0.20
  max_z: 0.80
  unknown_as_occupied: true
  min_observed_voxels: 2
  low_obstacle_height: 0.07
  obstacle_height: 0.14
  min_ratio: 0.35
  passable_cost: 60
  hysteresis_enable: true
  hysteresis_count: 2
  hole_fill_enable: true
  hole_fill_radius: 1

field:
  enable: true
  inflation_radius: 0.30
  max_distance: 3.0
  min_distance: -1.0
  clamp_distance: true
  interpolation: bilinear

decay:
  enable: true
  keep_time: 0.3
  decay_time: 1.0
  active_list_enable: true

performance:
  parallel_raycast_enable: true
  raycast_num_threads: 4
  dirty_column_enable: false
  field_update_rate: 20.0

debug:
  layer_pub_enable: true
  field_pub_enable: true
  pub_rate: 5.0
```

如果原配置系统不支持分组，可用前缀参数名。

### 6.8 性能日志和验证指标

每帧记录：

```text
input_point_count
prob_update_time
raycast_time
raycast_merge_time
decay_time
projection_time
field_time
query_refresh_time
total_update_time

occupied_count
unknown_count
passable_count
free_count
decayed_count
```

测试场景：

```text
1. 静态墙体
2. 低矮可跨越障碍
3. 动态障碍移走
4. 稀疏观测空洞
5. 狭窄通道
```

---

## 7. 当前 ROGMap 改造优先级

当前不讨论 planner，不处理 dense cloud 和复杂地形检测时，Codex 后续任务优先级为：

```text
P0:
  1. applyDecay 完整闭环
  2. parallel raycasting
  3. inflation/counter 状态一致性

P1:
  4. ProjectionLayer hysteresis
  5. ProjectionLayer small hole filling
  6. confidence 字段
  7. debug topic

P2:
  8. QueryAdapter snapshot sequence/stamp
  9. 弱化 values() 裸指针
  10. DynamicLayer evaluate clamp / NaN 保护

P3:
  11. dirty column
  12. ESDF quadratic interpolation 预留
  13. field update 降频
```

---

## 8. Planner 相关历史结论，仅作背景

虽然当前不处理 planner，但之前对 planner 的结论如下：

静态地图 / Nav2 costmap 因为和实际环境有误差，不能作为安全约束，只能作为引导。

推荐结构：

```text
Static Map / Nav2:
  global guide only

ROGMap:
  local search
  MINCO obstacle cost
  collision check
  recovery distance
  emergency stop
  trajectory validation
```

判断原则：

```text
只影响“往哪边走”：
  可以用静态地图。

影响“能不能走 / 能不能执行”：
  必须用 ROGMap。
```

---

## 9. 已生成过的 Codex 任务范围

上一轮已生成一版 Codex 目标执行 prompt，要求完成：

```text
fading / decay 完整闭环
parallel raycasting
ProjectionLayer 稳定化
DynamicLayer evaluate 稳定化
QueryAdapter snapshot 安全
ROGMap debug 可视化
参数体系整理
性能日志和验证指标
dirty column 预留
```

明确排除：

```text
dense cloud
复杂地形检测
planner 主逻辑
轮速接口
动态障碍聚类
Batch-LIWO
```

---

## 10. 下一轮对话建议起点

下一轮如果继续 ROGMap，应直接从这里开始：

```text
请根据当前 ROGMap 代码，优先检查并补齐 applyDecay / timestamp / inflation-counter 同步闭环，然后再检查 updateProbMap 是否具备 parallel raycasting。暂时不要处理 planner、dense cloud、复杂地形检测。
```

当前核心判断：

```text
ROGMap 已经有结构，但还缺动态遗忘闭环、并行性能、语义稳定性、debug 可视化和状态一致性验证。
```
