你现在需要在当前 ROS2 ROGMap 工程中做一次“最小侵入式修复和验证”。背景如下：

当前 `/rog_map/layer_value` 已经改成按 projection mask 显示：

* `mask=1` 显示白色，表示 FREE / PASSABLE / UNKNOWN-as-free
* `mask=0` 显示黑色，表示 OCCUPIED / UNKNOWN-as-occupied

现在现象是：

* unknown 区域已经正常变白；
* 但实际走廊可通行区域仍然是黑色；
* 打开 `/rog_map/occupied` 发现黑色 layer 上方几乎全是占据点；
* 占据点呈现从最低到最高的一整列，没有中间空心区域。

经过对照原始 ROGMap，结论是：

1. 不要修改原始 `ProbMap::isOccupied()`、`ProbMap::isUnknown()`、`ProbMap::isKnownFree()`、`ProbMap::getGridType()` 的语义。
2. 不要修改原始 raycast / hit / miss 概率更新逻辑。
3. 原始 `/rog_map/occ` 发布本来就是通过 `boxSearch(..., OCCUPIED, ...)`，其内部使用 `isOccupied(id_g)`，因此它包含 virtual ground / virtual ceil / safe_margin 的安全查询语义，并不等价于 raw `occupancy_buffer_`。
4. 当前 projection scanner 如果直接使用 `getGridType(id_g)`，会把原始 ROGMap 的“安全查询占据语义”当成“真实三维占据统计”，可能导致二维 layer 被虚拟边界或安全占据语义污染。
5. 当前需要保留原始 occupied 发布，同时新增 raw occupied debug 发布，并让 projection 使用 raw occupancy buffer 统计真实三维占据，而不是使用 `getGridType()`。

请完成以下修改。

---

## 一、保持原始占据逻辑不变

请检查并确保以下函数的原始语义不被修改：

* `ProbMap::isOccupied(const Vec3f & pos) const`
* `ProbMap::isOccupied(const Vec3i & id_g) const`
* `ProbMap::isUnknown(...)`
* `ProbMap::isKnownFree(...)`
* `ProbMap::getGridType(...)`
* `ProbMap::raycastProcess(...)`
* `ProbMap::insertUpdateCandidate(...)`
* `ProbMap::updateProbMap(...)`
* `ROGMap::boxSearch(...)`

这些函数中关于 virtual ground / virtual ceil / safe_margin 的逻辑应保留，因为它们是 ROGMap 原始安全查询语义的一部分。

不要为了修复 projection 问题去全局修改这些函数。

---

## 二、给 ROGMap 增加 raw occupied 查询/收集函数

在合适位置新增一个只基于 `occupancy_buffer_` 的 raw occupied 搜索函数，例如：

```cpp
void ROGMap::rawOccupiedBoxSearch(
  const Vec3f & box_min,
  const Vec3f & box_max,
  vec_E<Vec3f> & out_points) const;
```

或根据当前工程命名风格选择等价命名。

这个函数必须满足：

1. 遍历方式可以参考原有 `boxSearch(box_min, box_max, OCCUPIED, out_points)`。
2. 但判断 occupied 时禁止调用：

```cpp
isOccupied(id_g)
getGridType(id_g)
```

3. 必须只判断 raw `occupancy_buffer_`：

```cpp
if (!insideLocalMap(id_g)) {
  continue;
}

const int hash_id = getHashIndexFromGlobalIndex(id_g);
if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
  continue;
}

const double ret = occupancy_buffer_[hash_id];
if (isOccupied(ret)) {
  Vec3f pos;
  globalIndexToPos(id_g, pos);
  out_points.push_back(pos);
}
```

如果当前工程里 `getHashIndexFromGlobalIndex()` 返回值不是 int，而是 size_t 或其他类型，请按实际类型安全处理。

注意：

* 这里允许调用 `isOccupied(double occupancy_value)` 这种 raw buffer 判断函数；
* 不允许调用 `isOccupied(Vec3i)` 或 `isOccupied(Vec3f)`，因为它们包含 virtual ground / ceil 语义。

---

## 三、新增 `/rog_map/raw_occupied` debug topic

在 `rog_map_ros2.hpp` 中新增 raw occupied publisher。

目标 topic 名：

```text
/rog_map/raw_occupied
```

或保持当前命名空间风格：

```text
rog_map/raw_occupied
```

要求：

1. 保留原有 `/rog_map/occupied` 或 `/rog_map/occ` 发布逻辑不变。
2. 新增 raw occupied 发布逻辑。
3. raw occupied 发布使用上一步新增的 `rawOccupiedBoxSearch()`，不要使用 `boxSearch(..., OCCUPIED, ...)`。
4. 发布消息类型和原 occupied 一样，使用 `sensor_msgs::msg::PointCloud2`。
5. frame、stamp、转换函数尽量复用原 occupied 发布逻辑，例如 `vecEVec3fToPC2()`。

伪代码参考：

```cpp
if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
  vec_E<Vec3f> raw_occ_map;
  rawOccupiedBoxSearch(box_min, box_max, raw_occ_map);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  vecEVec3fToPC2(raw_occ_map, cloud_msg);
  cloud_msg.header.stamp = now();
  cloud_msg.header.frame_id = cfg_.frame_id;
  vm_.raw_occ_pub->publish(cloud_msg);
}
```

请根据当前工程真实成员变量命名适配。

---

## 四、修改 projection scanner：使用 raw occupancy buffer，不用 getGridType()

在 `ROGMap::refreshLayers()` 中查找 projection scanner。

当前逻辑大概率类似：

```cpp
GridType gt = getGridType(id_g);

if (gt == GridType::OCCUPIED || gt == GridType::KNOWN_FREE) {
  ++stats.observed;
}

if (gt == GridType::OCCUPIED) {
  ++stats.occupied;
  ...
}
```

请改成只基于 raw occupancy buffer 的局部查询。

建议在 scanner lambda 内部新增局部 lambda：

```cpp
auto rawGridType = [this](const Vec3i & id_g) -> GridType {
  if (!insideLocalMap(id_g)) {
    return GridType::OUT_OF_MAP;
  }

  const int hash_id = getHashIndexFromGlobalIndex(id_g);
  if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
    return GridType::OUT_OF_MAP;
  }

  const double ret = occupancy_buffer_[hash_id];

  if (isKnownFree(ret)) {
    return GridType::KNOWN_FREE;
  }
  if (isOccupied(ret)) {
    return GridType::OCCUPIED;
  }
  return GridType::UNKNOWN;
};
```

然后把 scanner 中的：

```cpp
GridType gt = getGridType(id_g);
```

替换为：

```cpp
GridType gt = rawGridType(id_g);
```

要求：

1. projection 的三维列统计只能使用 raw occupancy buffer。
2. 不要在 projection scanner 中使用 `getGridType(id_g)`。
3. 不要在 projection scanner 中使用 `isOccupied(id_g)`。
4. 继续保留 `cellLastHitTime(id_g)`、`cellLastUpdateTime(id_g)` 等统计逻辑。
5. 保留原有 `min_z / max_z / occupied_below_body / occupied_in_body_band / occupied_above_body / ground_z / ceiling_z` 的统计逻辑。
6. 不要删除 terrain 分析、hole fill、mask 生成逻辑；本次只修复其输入统计来源。

---

## 五、增加必要日志，便于实机判断

在 raw occupied 发布和 projection 更新处增加低频 throttle 日志，至少输出以下统计：

```text
raw_occupied_points
original_occupied_points
projection occupied_count
projection free_count
projection passable_count
projection unknown_count
projection mask0_count
projection mask1_count
```

如果已有 PerformanceMonitor 或 RuntimeStats 中已经统计了类似字段，就复用，不要重复引入复杂结构。

日志频率建议 1 Hz 或更低，避免刷屏。

---

## 六、验证标准

修改完成后，请在输出说明中告诉我以下判断方法。

### 情况 A

如果：

```text
/rog_map/occupied 仍然是一整列
/rog_map/raw_occupied 不是一整列
/rog_map/layer_value 走廊变白
```

说明问题来自原始 `isOccupied(id_g)` / `getGridType(id_g)` 的 virtual ground / virtual ceil 安全查询语义被 projection 错用。修复成功。

### 情况 B

如果：

```text
/rog_map/raw_occupied 也是一整列
/rog_map/layer_value 仍然黑
```

说明真实 `occupancy_buffer_` 已经被写成一整列。此时问题不在 projection，而在上游点云输入、frame 变换、raycast clearing、map sliding 或概率更新流程，需要继续排查。

### 情况 C

如果：

```text
/rog_map/raw_occupied 正常
/rog_map/layer_value 仍然黑
```

说明 projection 分类逻辑仍有问题，需要继续检查：

* `min_observed_voxels`
* `robot_body_z_min / robot_body_z_max`
* `occupied_in_body_band`
* `vertical_wall`
* `ceiling_blocks`
* `hole_fill`
* `terrain_enable`

---

## 七、输出要求

请完成代码修改后输出：

1. 修改了哪些文件；
2. 每个文件做了什么；
3. 是否保持原始 ROGMap 占据更新逻辑不变；
4. 新增 topic 名称；
5. projection 是否已经改为 raw occupancy buffer；
6. 编译命令；
7. 实机验证命令，例如：

```bash
colcon build --packages-select <相关包名> --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 topic list | grep rog_map
ros2 topic echo /rog_map/raw_occupied --once
ros2 topic hz /rog_map/raw_occupied
```

8. RViz 中需要同时观察：

```text
/rog_map/occupied
/rog_map/raw_occupied
/rog_map/layer_value
/rog_map/field
/cloud_registered 或 /cloud_registered_filtered
```

请严格控制修改范围，不要重构整体架构，不要改动 planner，不要改动 ESDF 符号，不要改动全局 `isOccupied()` / `getGridType()` 的原始语义。
