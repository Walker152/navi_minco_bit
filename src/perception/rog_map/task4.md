# ROGMap Visualization 简化改造任务

## 目标

简化 ROGMap 可视化配置，去掉过多的 topic name、per-topic enable 和 lazy_publish 参数，改为：

```text
固定话题名 + 全局 visualization.enable + 按订阅者自动发布
```

本任务只改可视化参数和发布逻辑，不新增可视化性能统计。

---

## 保留的 visualization 参数

最终 YAML 只保留：

```yaml
visualization:
  # [visualization][bool] 总开关。false 时不创建可视化 timer，不发布任何可视化 topic。
  enable: true

  # [visualization][frame] 可视化消息 frame_id。为空时使用 ROGMap frame_id。
  frame_id: camera_init

  # [visualization][Hz] 可视化发布上限频率。实际每个 topic 仅在存在订阅者时构造并发布。
  rate: 10.0

  # [visualization][m,m,m] 三维可视化范围，仅影响调试消息大小，不影响地图计算。
  range: [10.0, 10.0, 1.2]
```

---

## 废弃的 visualization 参数

删除或停止使用以下参数：

```text
visualization.lazy_publish

visualization.occupied.enable
visualization.unknown.enable
visualization.inflated_occupied.enable
visualization.inflated_unknown.enable
visualization.frontier.enable
visualization.layer_value.enable
visualization.layer_type.enable
visualization.layer_confidence.enable
visualization.layer_height.enable
visualization.field.enable
visualization.decay_cells.enable
visualization.map_bound.enable

visualization.occupied.topic
visualization.unknown.topic
visualization.inflated_occupied.topic
visualization.inflated_unknown.topic
visualization.frontier.topic
visualization.layer_value.topic
visualization.layer_type.topic
visualization.layer_confidence.topic
visualization.layer_height.topic
visualization.field.topic
visualization.decay_cells.topic
visualization.map_bound.topic
```

要求：

1. 旧 YAML 中如果仍存在这些参数，不要导致节点启动失败。
2. 可以不再 declare 这些旧参数。
3. 如果当前参数加载框架要求必须 declare，可以保留 declare，但不再使用。
4. 不要因为旧参数存在就改变发布行为。

---

## 固定话题名

保持当前已有 topic 名称不变。优先沿用当前代码已经使用的名字，例如：

```text
/rog_map/occupied
/rog_map/unknown
/rog_map/inflated_occupied
/rog_map/inflated_unknown
/rog_map/frontier
/rog_map/layer_value
/rog_map/layer_type
/rog_map/layer_confidence
/rog_map/layer_height
/rog_map/field
/rog_map/decay_cells
/rog_map/map_bound
```

不要再从 YAML 读取每个 topic 的自定义名称。

---

## 发布逻辑

每个可视化 topic 都必须在构造消息之前判断订阅者数量。

正确写法：

```cpp
if (!pub || pub->get_subscription_count() == 0) {
  return;
}

auto msg = buildMessage();
pub->publish(msg);
```

禁止写成：

```cpp
auto msg = buildMessage();

if (!pub || pub->get_subscription_count() == 0) {
  return;
}

pub->publish(msg);
```

原因：无订阅者时不能构造大消息，否则仍然会产生可视化开销。

---

## 全局 enable 行为

`visualization.enable=false` 时：

1. 不创建 visualization timer。
2. 可以不创建 visualization publishers。
3. 不执行任何 visualization publish 逻辑。
4. 不影响 ROGMap 内部建图。
5. 不影响 raycasting。
6. 不影响 decay。
7. 不影响 projection。
8. 不影响 field / ESDF。
9. 不影响 query adapter。
10. 不影响 MincoPlanner 通过指针查询 ROGMap。

---

## rate 行为

`visualization.rate` 只控制可视化 timer 的发布上限。

要求：

1. `rate <= 0` 时使用默认值，例如 5.0Hz。
2. 有订阅者时，最多按该频率发布。
3. 无订阅者时，timer 可以触发，但每个 topic 都必须立即 skip，不构造消息。
4. 该频率不代表 ROGMap 内部 field 更新频率。
5. 该频率不代表 projection 更新频率。

---

## range 行为

`visualization.range` 只限制三维可视化消息范围。

要求：

1. 主要影响 occupied、unknown、inflated_occupied、inflated_unknown、frontier、decay_cells 等大点云或 marker 消息。
2. 不影响 ROGMap 内部地图大小。
3. 不影响 projection 计算范围。
4. 不影响 field / ESDF 计算范围。
5. 不影响 planner 查询范围。

---

## frame_id 行为

`visualization.frame_id` 只用于可视化消息 header。

要求：

1. visualization 消息 header.frame_id 使用该参数。
2. 如果为空，则默认使用 ROGMap 的 `frame_id`。
3. 不改变 ROGMap 内部坐标系。
4. 不改变点云输入坐标系。
5. 不改变 planner 查询坐标系。

---

## 需要修改的文件

优先检查并修改：

```text
rog_map_ros2.cpp
rog_map_ros.hpp
rog_map.cpp
prob_map.cpp
sentry1.yaml
```

如果 visualization 参数结构定义在其他文件，也同步修改：

```text
config.hpp
config.h
rog_map_config.hpp
rog_map_param.hpp
```

---

## 不需要新增可视化性能统计

本任务不要新增以下内容：

```text
vis_skip_no_subscriber_*
vis_publish_*
visualization_*_build_time_ms
visualization_*_publish_time_ms
visualization_*_subscriber_count
visualization_*_publish_hz
```

也不要把这些字段加入 CSV。

之前提到的 ROGMap 性能 CSV 只关注核心建图链路，例如：

```text
input cloud
odom sync
updateMapInternal
raycast
prob update
decay
projection
mask changed
field / ESDF rebuild
query refresh
dirty column
full refresh
```

不要加入 visualization 相关性能字段。

---

## Publisher 创建

改造为固定创建这些 publishers：

```text
occupied_pub_
unknown_pub_
inflated_occupied_pub_
inflated_unknown_pub_
frontier_pub_
layer_value_pub_
layer_type_pub_
layer_confidence_pub_
layer_height_pub_
field_pub_
decay_cells_pub_
map_bound_pub_
```

可以只在 `visualization.enable=true` 时创建。

QoS 沿用当前代码，不要随意改变 QoS。

---

## 每个 topic 的发布函数要求

为每个可视化发布函数加前置订阅者判断。

示例：

```cpp
void ROGMapROS::publishField()
{
  if (!field_pub_ || field_pub_->get_subscription_count() == 0) {
    return;
  }

  // 只有这里之后才允许读取 snapshot / 构造 grid / publish
}
```

所有 topic 都按这个模式改造：

```text
publishOccupied
publishUnknown
publishInflatedOccupied
publishInflatedUnknown
publishFrontier
publishLayerValue
publishLayerType
publishLayerConfidence
publishLayerHeight
publishField
publishDecayCells
publishMapBound
```

如果代码当前不是按这些函数命名，就在对应逻辑位置改造。

---

## sentry1.yaml 修改

把 `sentry1.yaml` 中 ROGMap visualization 块简化为：

```yaml
visualization:
  enable: true
  frame_id: camera_init
  rate: 10.0
  range: [10.0, 10.0, 1.2]
```

删除旧的 per-topic enable、topic name 和 lazy_publish 配置。

---

## 验证要求

完成后编译：

```bash
colcon build --symlink-install
```

启动后验证：

```bash
ros2 topic list | grep rog_map
```

期望：

1. `visualization.enable=true` 时，固定 topic 存在。
2. `visualization.enable=false` 时，不执行可视化发布逻辑。
3. 无订阅者时，不构造可视化消息。
4. 执行 `ros2 topic hz /rog_map/field` 后，只触发 field 消息构造和发布。
5. 打开 RViz 只订阅 `/rog_map/layer_value` 后，不应构造其他 topic 的消息。
6. ROGMap 内部 projection、field、query 不受 visualization 参数简化影响。
7. MincoPlanner 正常规划。
8. 不出现参数未声明导致的启动失败。

---

## 禁止事项

不要做以下改动：

```text
不要改变 ROGMap 建图逻辑。
不要改变 raycasting 逻辑。
不要改变 decay 逻辑。
不要改变 projection 分类逻辑。
不要改变 field / ESDF 计算逻辑。
不要改变 QueryAdapter 查询语义。
不要改变 MincoPlanner 对 ROGMap 的调用方式。
不要改变 frame transform 逻辑。
不要新增可视化性能统计。
不要把可视化相关字段加入 CSV。
不要在没有订阅者时构造大消息。
```

---

## 最终输出

完成后只输出：

```text
1. 修改文件列表
2. 删除/废弃的 visualization 参数列表
3. 当前保留的 visualization 参数
4. 编译结果
5. 简单验证命令
```
