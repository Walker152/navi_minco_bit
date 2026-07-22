# Codex 改造任务：将 PGM 静态先验直接叠加到 ROGMap Projection 输出

## 1. 任务目标

在当前 ROGMap 的二维 Projection 生成链路中加入一个最小化的静态先验障碍融合能力。

用户提供一套标准 Nav2 地图文件：

- `*.yaml`
- `*.pgm`

不再增加 AABB、ROI 或局部裁剪逻辑。加载完整 PGM 后，在每次 Projection 输出生成完成时，根据当前 Projection 栅格在 `map` 坐标系中的位置查询 PGM，并将其中的静态障碍直接叠加到 Projection 的二维规划输出中。

最终链路应为：

```text
ROGMap 三维在线占据
→ ProjectionLayer 生成动态 cells / values / mask
→ 按 map 坐标查询完整 PGM
→ 静态障碍与动态障碍取并集
→ fused_values / fused_mask
→ 现有 DynamicLayer 构建二维 ESDF
→ 现有 Query / MINCO / Corridor / Safety Checker 继续使用统一结果
```

这是硬障碍融合，不是额外权重项。

---

## 2. 当前代码事实

当前代码中：

```text
ProjectionLayer::mask()
0 = obstacle
1 = free/passable
```

`ROGMap::refreshLayers()` 在 Projection 更新完成后：

1. 读取 `layer_->mask()` 判断 mask 是否变化；
2. 将 `layer_->mask()` 传给 `DynamicLayer::updateFromMask()`；
3. `refreshQuery()` 将 `layer_->values()` 写入 `MapSnapshot`。

因此本次改造应集中在：

```text
Projection 更新完成
与
mask 变化检测、Field 更新、Query 快照生成
之间
```

不要修改 MINCO、走廊生成器、安全检查器和 Field 的内部算法。

---

## 3. 总体实现原则

必须遵守以下原则：

1. 最小改动，不整体重构 ROGMap。
2. 不新增独立静态 ESDF。
3. 不新增新的地图 Layer 类。
4. 不订阅 `/map`。
5. 不访问 Nav2 StaticLayer 或 master costmap。
6. 不增加 AABB、ROI、裁剪框或多区域配置。
7. 不修改三维 ProbMap。
8. 不修改 `classifyCell()`。
9. 不修改地面、坡面、墙体、隧道、迟滞、Fill、Denoise、Decay 的现有语义。
10. 不将 PGM 障碍写入 `CellData::raw_type`、`base_type` 或 `type`。
11. PGM 只能增加障碍，绝不能用 PGM 自由区域清除在线动态障碍。
12. 保持 `layer_->cells()` 表示在线三维感知分类。
13. 使用融合后的 `values` 和 `mask` 供规划与 ESDF 使用。
14. 不对 PGM 做额外膨胀；现有 Field inflation 继续统一生效。
15. 不增加无关性能统计、调试话题和复杂缓存。
16. 不执行任何 Git commit、push、rebase 等操作。
17. 不启动完整导航或仿真，只允许编译、单元测试和静态检查。

---

## 4. 推荐的最小结构

不要把静态先验直接写回 `ProjectionLayer::cells_`。

在 `ROGMap` 中增加两个最终输出缓存：

```cpp
std::vector<uint8_t> fused_projection_mask_;
std::vector<uint8_t> fused_projection_values_;
```

它们只表示：

```text
在线 Projection 输出
+
PGM 静态障碍
```

每次 `ProjectionLayer` 更新完成后执行一次融合：

```cpp
rebuildFusedProjection();
```

概念流程：

```cpp
fused_projection_mask_ = layer_->mask();
fused_projection_values_ = layer_->values();

for each projection cell:
  if prior PGM says obstacle:
    fused_projection_mask_[i] = 0U;
    fused_projection_values_[i] = 254U;
```

这样：

- `layer_->cells()` 保持纯动态分类；
- `layer_->mask()` 保持纯动态 Projection；
- `fused_projection_mask_` 作为 Field 的输入；
- `fused_projection_values_` 作为 Query 的二维代价值；
- 不会因 dirty update 保留上一次静态叠加产生的脏状态；
- PGM 与 Projection 的职责仍然清楚；
- 改动集中在 `ROGMap::refreshLayers()` 和 `refreshQuery()`。

虽然融合缓存在 `ROGMap` 中，但必须在 `refreshLayers()` 的 Projection 生成阶段立即完成，不要再设计独立异步模块或第二套更新周期。

---

## 5. 配置参数

在现有：

```yaml
rog_map:
  projection:
```

下增加：

```yaml
projection:
  prior_map:
    enable: true
    yaml_path: /absolute/path/to/map.yaml
    pgm_path: /absolute/path/to/map.pgm
    frame_id: map
```

参数要求：

### 5.1 `enable`

```text
false：
完全保持当前行为；
fused mask/value 直接等于动态 layer mask/value。

true：
加载并启用静态 PGM 障碍叠加。
```

### 5.2 `yaml_path`

读取标准 Nav2 map YAML 元数据，包括：

```yaml
image: map.pgm
resolution: 0.05
origin: [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 5.3 `pgm_path`

- 非空时使用该路径；
- 为空时使用 YAML 的 `image` 字段；
- YAML 中的相对 `image` 路径必须相对于 YAML 文件所在目录解析；
- 不增加其他路径兼容逻辑。

### 5.4 `frame_id`

- 默认 `map`；
- 表示 YAML/PGM 所处坐标系；
- 不从 PGM 文件内部推断 frame。

不要增加：

```text
AABB
ROI
权重
置信度
额外 inflation
unknown_as_obstacle
多地图列表
运行时热更新
```

PGM 的未知区域固定为“不施加静态约束”。

---

## 6. PGM/YAML 加载

静态地图只在初始化阶段加载一次。

禁止在每次点云回调、Projection 更新或 Field 更新时重新读取文件。

保存最少数据：

```cpp
struct PriorMapData
{
  bool loaded{false};
  int width{0};
  int height{0};
  double resolution{0.0};

  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};

  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.25};

  std::vector<uint8_t> occupied;
};
```

其中：

```text
occupied[i] = 1：PGM 静态障碍
occupied[i] = 0：自由、未知或地图外
```

不要保存一份 Nav2 Costmap，也不需要创建 `nav_msgs::msg::OccupancyGrid`。

### 6.1 依赖选择

先检查当前工程已有依赖：

1. 优先复用现有的 YAML 解析和地图图像读取能力；
2. 如果已经依赖 `nav2_map_server/map_io`，可复用其标准地图加载逻辑；
3. 如果没有可直接复用的图像读取能力，实现一个局部、简单的 PGM 读取函数；
4. 不要仅为读取 PGM 引入 OpenCV 等大型新依赖。

PGM 至少支持常用二进制 `P5`，可以顺带兼容 `P2`。

PGM 解析必须：

- 跳过 `#` 注释；
- 检查宽、高和最大灰度值；
- 检查像素数量；
- 文件异常时给出明确错误；
- `enable=true` 且加载失败时，按当前配置初始化错误风格直接配置失败，不要静默禁用。

---

## 7. 灰度到障碍的转换

遵守标准 Nav2 地图语义：

```cpp
const double occupancy =
  negate
    ? static_cast<double>(gray) / 255.0
    : static_cast<double>(255 - gray) / 255.0;
```

仅当：

```cpp
occupancy > occupied_thresh
```

时：

```cpp
prior_occupied = true;
```

其余情况全部视为不施加静态障碍：

```cpp
prior_occupied = false;
```

因此：

- PGM 自由区域不清除动态障碍；
- PGM 未知区域不阻挡；
- 阈值之间的灰色区域不阻挡；
- 不额外使用 `free_thresh` 决定融合行为，但仍正确读取和校验该字段。

---

## 8. 坐标系与 TF

PGM/YAML 位于：

```text
prior_map.frame_id
```

默认：

```text
map
```

ProjectionLayer 位于 ROGMap 当前地图 frame，通常为：

```text
camera_init
```

必须将每个 Projection 栅格中心从 ROGMap frame 转换到 `map` frame，再查询 PGM。

### 8.1 ROS 层职责

优先让现有 `ROGMapROS`：

- 加载 YAML/PGM；
- 复用已有 Node 与 TF 能力；
- 每轮更新前获取一次 `T_map_rog`；
- 将地图数据和二维变换快照传给 `ROGMap`。

不要在纯算法层创建新的：

```text
tf2_ros::Buffer
tf2_ros::TransformListener
ROS subscription
ROS timer
```

如果当前 `ROGMapROS` 已经持有 TF 相关对象，直接复用。

如果当前类关系下由 `MincoPlanner` 持有唯一的 `tf_`，沿现有对象构造或配置路径传递该共享指针，禁止再创建第二套 TF。

### 8.2 TF 调用频率

每轮 Projection 融合最多调用一次：

```cpp
lookupTransform(prior_frame, rog_frame, ...)
```

禁止：

```cpp
for each cell:
  lookupTransform(...)
```

将 TF 结果提前转为轻量二维变换：

```cpp
struct Transform2D
{
  double tx;
  double ty;
  double yaw;
};
```

然后在循环中直接计算。

### 8.3 TF 不可用

运行中 TF 暂时不可用时：

1. 本轮 `fused_projection_mask_`、`fused_projection_values_` 退化为纯动态结果；
2. 不沿用上一轮静态融合缓存；
3. 使用节流警告；
4. 不阻塞点云和 ROGMap 在线更新；
5. 后续 TF 恢复后自动重新叠加。

这样可以避免错误坐标下的旧静态障碍残留。

---

## 9. Projection 栅格坐标

当前 Projection 的：

```cpp
layer_->origin()
layer_->resolution()
layer_->width()
layer_->height()
```

用于计算每个栅格中心。

对于局部索引 `(x, y)`：

```cpp
const double rog_x =
  layer_->origin().x() +
  (static_cast<double>(x) + 0.5) * layer_->resolution();

const double rog_y =
  layer_->origin().y() +
  (static_cast<double>(y) + 0.5) * layer_->resolution();
```

通过一次获取的二维 TF：

```cpp
const double c = std::cos(tf_yaw);
const double s = std::sin(tf_yaw);

const double map_x = c * rog_x - s * rog_y + tf_tx;
const double map_y = s * rog_x + c * rog_y + tf_ty;
```

注意确认 `lookupTransform()` 的 source/target 顺序，最终必须得到：

```text
p_map = T_map_rog * p_rog
```

不要凭函数名猜方向，使用一个已知点进行静态验证。

---

## 10. map 坐标查询 PGM

YAML 的：

```yaml
origin: [origin_x, origin_y, origin_yaw]
```

表示图像栅格坐标系原点在 `map` 中的位姿。

先将 `map` 点转到 PGM 局部坐标：

```cpp
const double dx = map_x - prior.origin_x;
const double dy = map_y - prior.origin_y;

const double c = std::cos(prior.origin_yaw);
const double s = std::sin(prior.origin_yaw);

const double local_x =  c * dx + s * dy;
const double local_y = -s * dx + c * dy;
```

然后计算从地图左下角开始的格子索引：

```cpp
const int mx =
  static_cast<int>(std::floor(local_x / prior.resolution));

const int my_from_bottom =
  static_cast<int>(std::floor(local_y / prior.resolution));
```

PGM 像素行从图像顶部向下，因此：

```cpp
const int image_col = mx;
const int image_row = prior.height - 1 - my_from_bottom;
```

边界检查：

```cpp
if (image_col < 0 || image_col >= prior.width ||
    image_row < 0 || image_row >= prior.height)
{
  return false;
}
```

地图外固定为：

```text
无静态障碍约束
```

不要把 PGM 外部区域当作 occupied 或 unknown obstacle。

---

## 11. 融合函数

增加一个职责单一的函数，名称可按现有代码风格调整：

```cpp
void ROGMap::rebuildFusedProjection(
  const PriorMapTransform2D * transform);
```

职责：

1. 复制当前动态 Projection 输出；
2. 检查先验开关、地图加载状态和 TF 状态；
3. 遍历完整 Projection 网格；
4. 查询对应 PGM 像素；
5. 仅将静态障碍写入融合 mask/value。

概念代码：

```cpp
void ROGMap::rebuildFusedProjection(
  const PriorMapTransform2D * transform)
{
  fused_projection_mask_ = layer_->mask();
  fused_projection_values_ = layer_->values();

  if (!cfg_.prior_map_enable ||
      !prior_map_.loaded ||
      transform == nullptr)
  {
    return;
  }

  const int width = layer_->width();
  const int height = layer_->height();
  const double resolution = layer_->resolution();
  const Eigen::Vector2d origin = layer_->origin();

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t index =
        static_cast<size_t>(y) * static_cast<size_t>(width) +
        static_cast<size_t>(x);

      const Eigen::Vector2d p_rog(
        origin.x() + (static_cast<double>(x) + 0.5) * resolution,
        origin.y() + (static_cast<double>(y) + 0.5) * resolution);

      const Eigen::Vector2d p_map =
        transformPoint(*transform, p_rog);

      if (!priorMapOccupied(p_map)) {
        continue;
      }

      fused_projection_mask_[index] = 0U;
      fused_projection_values_[index] =
        nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  }
}
```

如果当前 `rog_map` 底层不应该依赖 `nav2_costmap_2d` 常量，则直接使用当前已有的：

```cpp
254U
```

不要为了一个常量新增依赖。

---

## 12. 融合语义

当前 mask 语义：

```text
0 = obstacle
1 = free/passable
```

正确语义为：

```cpp
const bool dynamic_obstacle =
  dynamic_mask[i] == 0U;

const bool prior_obstacle =
  queryPriorObstacle(...);

const bool fused_obstacle =
  dynamic_obstacle || prior_obstacle;

fused_mask[i] =
  fused_obstacle ? 0U : 1U;
```

PGM 静态障碍只执行：

```cpp
fused_mask[i] = 0U;
fused_value[i] = 254U;
```

禁止以下错误行为：

```cpp
if PGM is free:
  fused_mask[i] = 1U;   // 错误，会清除动态障碍
```

虽然在两个 mask 都采用 `0=障碍、1=自由` 时可以写：

```cpp
fused_mask[i] = dynamic_mask[i] & prior_mask[i];
```

但实际实现优先使用布尔障碍表达，避免语义混淆。

---

## 13. `refreshLayers()` 修改

当前 `refreshLayers()` 在 Projection 更新前保存：

```cpp
const std::vector<uint8_t> old_mask = layer_->mask();
```

改为保存上一轮最终融合结果：

```cpp
const std::vector<uint8_t> old_fused_mask =
  fused_projection_mask_;
```

在以下任一分支完成后：

```text
updateFull()
updateDirty()
no_dirty
```

都必须调用一次：

```cpp
rebuildFusedProjection(current_transform_or_null);
```

即使没有 dynamic dirty column，也要重新构建融合结果，因为：

- `map → rog_frame` TF 可能变化；
- ROGMap 滑动窗口 origin 可能变化；
- 上一轮 TF 可能不可用，本轮恢复；
- 不能只依赖动态 Projection dirty。

之后 mask diff 改为比较：

```cpp
old_fused_mask
与
fused_projection_mask_
```

概念逻辑：

```cpp
size_t mask_diff_count = 0;

if (old_fused_mask.size() != fused_projection_mask_.size()) {
  mask_diff_count = fused_projection_mask_.size();
} else {
  for (size_t i = 0; i < fused_projection_mask_.size(); ++i) {
    if (old_fused_mask[i] != fused_projection_mask_[i]) {
      ++mask_diff_count;
    }
  }
}

const bool mask_changed = mask_diff_count > 0;
```

保持现有：

```text
mask_sequence_
field_dirty_
field_update_rate
```

调度结构不变，只将判定对象改成融合 mask。

不要修改：

```text
projection_sequence_
dirty column
force_full_refresh
Field update rate
```

的原有语义。

---

## 14. Field 输入修改

当前：

```cpp
field_->updateFromMask(
  ...,
  layer_->mask(),
  ...);
```

改为：

```cpp
field_->updateFromMask(
  layer_->width(),
  layer_->height(),
  layer_->resolution(),
  layer_->origin(),
  fused_projection_mask_,
  cfg_.field_inflation_radius,
  cfg_.field_max_distance,
  cfg_.field_min_distance,
  cfg_.field_clamp_distance_en,
  parseInterpolationMode(cfg_.field_interpolation),
  &field_stats);
```

DynamicLayer 和 EDT 算法完全不改。

不要单独对 PGM 膨胀，否则会与 Field inflation 重复。

---

## 15. Query 快照修改

当前：

```cpp
snapshot->values = layer_->values();
```

改为：

```cpp
snapshot->values = fused_projection_values_;
```

以下数据继续来自动态 Projection cells：

```cpp
snapshot->types
snapshot->height_deltas
snapshot->confidence
```

即：

```text
types / height / confidence：
表示在线三维感知分类。

values / distances：
表示动态障碍和 PGM 静态障碍融合后的规划结果。
```

不要为了 PGM 障碍伪造：

```text
CellType::OCCUPIED
raw_reason
height_delta
confidence
```

---

## 16. 初始化和尺寸一致性

完成 `ProjectionLayer` 更新后，融合缓存尺寸必须始终满足：

```cpp
fused_projection_mask_.size() ==
  static_cast<size_t>(layer_->width()) *
  static_cast<size_t>(layer_->height());

fused_projection_values_.size() ==
  fused_projection_mask_.size();
```

当：

```text
Projection disabled
layer empty
网格几何改变
先验 disabled
TF 不可用
```

时都不能留下旧尺寸或旧融合障碍。

当 layer 为空时清空融合缓存。

---

## 17. 配置加载位置

在当前 ROGMap `Config` 中增加最少字段：

```cpp
bool prior_map_enable{false};
std::string prior_map_yaml_path;
std::string prior_map_pgm_path;
std::string prior_map_frame{"map"};
```

沿用现有 `Config::loadFromRosNode()` 风格加载：

```text
projection.prior_map.enable
projection.prior_map.yaml_path
projection.prior_map.pgm_path
projection.prior_map.frame_id
```

不要保留重复命名或兼容别名。

配置示例加入当前参数文件：

```yaml
projection:
  enable: true

  prior_map:
    enable: true
    yaml_path: /home/rm/2027-sentry-navi/maps/rm_map.yaml
    pgm_path: /home/rm/2027-sentry-navi/maps/rm_map.pgm
    frame_id: map
```

关闭时：

```yaml
prior_map:
  enable: false
```

不得要求用户删除其他参数。

---

## 18. 构建系统

根据实际使用的依赖更新对应：

```text
CMakeLists.txt
package.xml
```

要求：

- 只增加真正需要的依赖；
- 不为读取 PGM 引入 OpenCV；
- 不重复链接已有依赖；
- 保持编译警告干净；
- 不修改无关 target。

---

## 19. 测试要求

不要启动完整导航或仿真。

至少完成以下静态测试或单元测试。

### 19.1 PGM 解析

构造一个很小的 PGM：

```text
3 × 3
```

验证：

- 黑色障碍；
- 白色自由；
- 灰色未知；
- `negate=0`；
- `negate=1`；
- P5；
- 注释行；
- 尺寸检查。

### 19.2 图像上下翻转

使用左上和左下不同像素的测试地图，验证：

```text
map 左下栅格
正确对应
PGM 最后一行
```

不能出现上下镜像。

### 19.3 YAML origin

验证：

```text
origin_x
origin_y
origin_yaw
resolution
```

至少测试：

```text
yaw = 0
yaw = π/2
```

### 19.4 TF 对齐

使用已知二维变换验证：

```text
p_map = T_map_rog * p_rog
```

确保 source/target 没有写反。

### 19.5 融合真值表

验证：

| 动态 | PGM | 融合 |
|---|---|---|
| free | free | free |
| obstacle | free | obstacle |
| free | obstacle | obstacle |
| obstacle | obstacle | obstacle |

### 19.6 地图外行为

Projection 栅格落在 PGM 外时：

```text
保持动态结果
```

### 19.7 TF 暂不可用

验证：

```text
上一轮有静态障碍
本轮 TF 不可用
→ 本轮退化为纯动态结果
→ 不残留上一轮静态障碍
```

### 19.8 编译

只编译涉及的包和直接依赖包。

不要通过修改无关代码绕过编译错误。

---

## 20. 验收标准

改造完成后必须满足：

1. 用户只需配置 PGM、YAML 和 frame。
2. 不需要 AABB 或任何裁剪配置。
3. PGM 只加载一次。
4. 每轮 Projection 生成后，完整 PGM 障碍按坐标叠加。
5. 动态障碍永远不会被 PGM 自由区域清除。
6. PGM 地图外区域不产生约束。
7. `layer_->cells()` 保持在线感知语义。
8. Field 使用融合 mask。
9. Query 使用融合 values 和融合 ESDF。
10. MINCO、Corridor、SafetyChecker 无需额外修改即可获得先验障碍。
11. dirty update、field update rate 和现有调度不被重构。
12. TF 失败时安全退化为纯动态地图。
13. 关闭先验开关后行为与改造前一致。
14. 不存在图像上下翻转、origin yaw 忽略或 TF 方向错误。
15. 没有新增 AABB、独立 prior ESDF 或第二套地图查询接口。

---

## 21. 禁止事项

禁止：

- 创建 `PriorESDFMap`；
- 修改 MINCO 障碍代价；
- 修改 CorridorGenerator；
- 修改 TrajectorySafetyChecker；
- 修改 SMAC/A*；
- 修改 ProbMap；
- 修改 Projection 的分类规则；
- 将静态障碍塞入 `CellType`；
- 使用 Nav2 已膨胀后的 master costmap；
- 对 PGM 进行第二次 inflation；
- 增加 AABB；
- 增加运行时地图热加载；
- 新建独立 ROS 节点；
- 新建第二个 TF Buffer；
- 每格执行 TF 查询；
- 进行无关重构；
- 增加大量性能日志；
- 启动仿真；
- 执行 Git 提交操作。

---

## 22. 完成报告要求

完成修改后，输出简洁报告，必须包含：

1. 修改了哪些文件；
2. PGM/YAML 在哪里加载；
3. TF 从哪里复用；
4. 融合函数位于哪里；
5. Field 输入如何改为融合 mask；
6. Query values 如何改为融合 values；
7. 如何保证 PGM 自由区域不清除动态障碍；
8. 如何处理 PGM 上下翻转和 YAML origin yaw；
9. TF 不可用时如何退化；
10. 完成了哪些编译或单元测试；
11. 是否存在仍需真实地图运行验证的部分。

不要提交代码，不要启动完整导航。
