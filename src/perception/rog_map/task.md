你是一个长期执行任务的高级 C++/ROS2 机器人开发助手。请在当前代码仓库中持续工作，直到完成目标或遇到真正无法继续的阻塞。不要只给建议，要直接修改代码、补全接口、更新 CMake/package、尽可能编译验证，并在每一阶段留下清晰的实现说明和 TODO。

# 总目标

将现有 ROGMap 改造成规划器可直接使用的动态 3D 感知 + 2D 地形距离场系统，用于替代 Nav2 costmap。最终 planner 不再依赖 Nav2 costmap，而是通过一个泛化的只读地图接口访问 ROGMap 内部生成的二维可通行层和 2D ESDF。

目标效果参考中科大哨兵方案：

1. ROGMap 维护机器人局部 3D Occupancy Grid。
2. 基于 3D Occupancy Grid 做 z-column 高程分析，生成 2D 地形语义层。
3. 2D 地形层输出 cost buffer 和 obstacle/free mask。
4. 将现有 planner 中的 DynamicLayer / ESDFUtils 引入 ROGMap，作为 2D ESDF layer。
5. ROGMap 内部生成 2D signed distance field，并提供 distance + gradient 查询。
6. Planner 通过泛化接口读取 ROGMap，不直接访问 ROGMap 内部 buffer。
7. 暂时不做动态障碍聚类，不做轮速接口，不做完整 Batch-LIWO。
8. 第一阶段优先保证架构正确、接口稳定、可编译、可调试。

# 重要设计约束

## 1. Planner 只通过接口读取地图

不要让 planner 直接持有 ROGMap 本体，也不要让 planner 访问 ROGMap 的 occupancy_buffer_、hash、SlidingMap 内部索引等细节。

应该新增一个泛化只读接口，例如：

```cpp
class MapQueryInterface {
public:
  virtual ~MapQueryInterface() = default;

  virtual bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const = 0;
  virtual void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const = 0;

  virtual unsigned int sizeX() const = 0;
  virtual unsigned int sizeY() const = 0;
  virtual double resolution() const = 0;
  virtual double originX() const = 0;
  virtual double originY() const = 0;

  virtual uint8_t value(unsigned int mx, unsigned int my) const = 0;
  virtual const unsigned char * values() const = 0;

  virtual bool isValid(unsigned int mx, unsigned int my) const = 0;
  virtual bool isFree(unsigned int mx, unsigned int my) const = 0;

  virtual bool evaluate(
    const Eigen::Vector3d & pos,
    double & dist,
    Eigen::Vector3d & grad) const = 0;
};
```

注意：函数名要泛化，不要写成 `getTerrainCost2D`、`evaluateESDF2D`、`isTraversable2D` 这种过于绑定实现的名字。接口表达“二维地图查询能力”，不要暴露“ROGMap/ESDF/TerrainLayer”的内部概念。

允许根据现有代码风格微调命名，例如：

```cpp
value()
values()
inside()
free()
evaluate()
```

或者：

```cpp
getValue()
getValues()
contains()
isFree()
evaluate()
```

但保持统一、简洁、泛化。

## 2. ROGMap 内部可以持有 DynamicLayer

当前 planner 已有 DynamicLayer、ESDFUtils、HybridESDFMap 等代码。需要将 DynamicLayer 或其核心能力引入 ROGMap 内部，作为 ROGMap 的二维距离场层。

第一阶段建议：

* 保留 ESDFUtils 的 computeEDT2D。
* 保留 DynamicLayer 的 signed distance + evaluate 逻辑。
* 给 DynamicLayer 增加从 occupancy mask 更新的接口。
* ROGMap 的地形层输出 occ/free mask 后，直接调用 DynamicLayer 更新 ESDF。
* 不要再让 DynamicLayer 订阅 `/global_costmap/voxel_grid`。
* 不要再依赖 Nav2 costmap 作为 ESDF 输入。

需要新增类似接口：

```cpp
void updateFromMask(
  int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const std::vector<uint8_t> & mask,
  double inflation_radius);
```

函数名可以是 `updateFromMask`、`update`、`resetFromMask`，保持泛化。不要写成 `updateFromTerrainLayer2D` 这种过细名称。

mask 语义建议：

```text
mask[idx] = 0: obstacle seed
mask[idx] = 1: free cell
```

## 3. 暂时不做动态障碍聚类

不要实现 dynamic cluster、Kalman filter、track id、速度估计等内容。

当前阶段只做：

* Occupancy timestamp。
* 可选 fading。
* 高程分析。
* 2D ESDF。
* planner 查询接口。

动态聚类可以留下 TODO，但不要占用主线。

## 4. 暂时不做轮速接口

不要添加 wheel odometry measurement。
不要改 Estimator 状态量以加入轮速。
不要实现 Batch-LIWO 中的 wheel covariance。
当前里程计只要求后续能给 ROGMap 提供：

* odom pose。
* dense undistorted world cloud。

如果本次任务范围主要是 ROGMap，就不要深入修改 LIO EKF。

# 代码风格要求

1. 使用现代 C++17。
2. 遵循当前仓库代码风格，优先和 ROGMap 原有命名、Eigen 类型、PCL 类型保持一致。
3. 新类职责清晰，不要把所有逻辑塞进 ROGMap::updateMap。
4. 尽量避免全局变量。
5. 对共享数据读写要有明确线程安全策略。
6. Planner 读地图时不能长时间持有 ROGMap 更新锁。
7. 接口只读，不允许 planner 通过接口修改地图。
8. 所有新增参数需要有默认值。
9. 新增文件要加入 CMakeLists.txt 和 package.xml 依赖。
10. 编译错误优先修复，不要留下明显不可编译代码。
11. 不要大规模重构无关模块。
12. 每完成一个阶段，更新一段简短实现记录，说明改了哪些文件、下一步是什么。

# 推荐架构

最终 ROGMap 内部结构如下：

```text
ROGMap
  ├── ProbMap / InfMap / SlidingMap 原有 3D occupancy
  ├── timestamp / fading 扩展
  ├── GridLayer 或 ProjectionLayer：z-column 高程分析
  ├── DynamicLayer：2D signed ESDF
  └── MapQueryInterface adapter：给 planner 只读查询
```

注意命名：

* 内部类可以叫 `GridLayer`、`ProjectionLayer`、`PlanarLayer`、`MapLayer`。
* 不建议对外接口函数写成过细的 `getTerrainLayer2D`、`getESDF2D`。
* 对外查询接口尽量命名为 `queryInterface()`、`mapInterface()`、`getMap()` 或 `getQuery()`。
* 如果需要保留语义枚举，可以在内部用 `CellType`，而不是对 planner 暴露太多 terrain 细节。

# 阶段 1：整理并引入 DynamicLayer 到 ROGMap

## 目标

将当前 planner 中的 DynamicLayer / ESDFUtils 迁入或复用到 ROGMap 包，使 ROGMap 能够从一个 2D mask 构建 signed distance field，并提供：

```cpp
evaluate(pos, dist, grad)
```

## 需要做的事

1. 找到当前 DynamicLayer、ESDFUtils、HybridESDFMap 的头文件和源文件。
2. 判断这些文件属于哪个 package。
3. 将 DynamicLayer 和 ESDFUtils 移动到 ROGMap package，或者在 CMake 中让 ROGMap 正确依赖它们。
4. 给 DynamicLayer 增加泛化输入接口：

```cpp
void updateFromMask(
  int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const std::vector<uint8_t> & mask,
  double inflation_radius);
```

5. 保留原来的 point cloud 更新接口，但不要让 ROGMap 使用点云投影接口。
6. 将 EDT 构造逻辑抽成内部函数，避免重复代码：

```cpp
void rebuild(
  int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const std::vector<uint8_t> & mask,
  double inflation_radius);
```

7. 保持 `evaluate()` 接口不变或做最小改动，因为 planner 优化器已经依赖该模式。

## 验收标准

* DynamicLayer 可以在不订阅 ROS topic 的情况下，由 ROGMap 主动传入 mask 更新。
* ESDFUtils 可以正常编译并被 ROGMap 调用。
* DynamicLayer 的 evaluate 能返回距离和梯度。
* 不再强依赖 `/global_costmap/voxel_grid`。

# 阶段 2：为 ROGMap 增加 2D 投影/地形层

## 目标

新增一个内部 layer，从 ROGMap 的 3D occupancy 中生成二维地图。

不要求名字必须是 TerrainLayer2D。优先使用泛化名称，例如：

```text
GridLayer
PlanarLayer
ProjectionLayer
MapLayer
```

但内部注释要说明其作用是 z-column 高程分析。

## 数据结构建议

```cpp
enum class CellType : uint8_t {
  UNKNOWN = 0,
  FREE = 1,
  PASSABLE = 2,
  OCCUPIED = 3
};

struct CellData {
  CellType type{CellType::UNKNOWN};

  float min_z{0.0f};
  float max_z{0.0f};
  float height{0.0f};
  float ratio{0.0f};

  float last_hit_time{0.0f};
  float last_update_time{0.0f};

  uint8_t value{255};
  uint8_t mask{0};
};
```

其中：

```text
value: 给 planner/A* 用，类似 cost buffer
mask: 给 DynamicLayer/ESDF 用，0 obstacle，1 free
```

建议映射：

```text
UNKNOWN:
  value = unknown_as_occupied ? 254 : 255
  mask  = unknown_as_occupied ? 0 : 1

FREE:
  value = 0
  mask  = 1

PASSABLE:
  value = passable_cost，例如 30~80
  mask  = 1

OCCUPIED:
  value = 254
  mask  = 0
```

## 高程分析方法

对每个 `(x, y)` column，扫描指定 `z` 范围：

```text
terrain_min_z ~ terrain_max_z
```

统计：

```text
n_occ: occupied voxel 数量
n_obs: observed voxel 数量，包括 occupied 和 known free
min_z: 最低 occupied voxel 高度
max_z: 最高 occupied voxel 高度
H = max_z - min_z
ratio = ((n_occ + 1) * resolution) / max(H, resolution)
```

分类逻辑第一版：

```text
if n_obs < min_observed_voxels:
    UNKNOWN

else if n_occ == 0:
    FREE

else if H <= low_obstacle_height:
    PASSABLE

else:
    OCCUPIED
```

第二版可以加入：

```text
if H > wall_height && ratio > min_ratio:
    OCCUPIED
```

但第一版不必过拟合。

## 配置参数

在 config 中加入：

```cpp
bool layer_en = true;
double layer_min_z = -0.20;
double layer_max_z = 0.80;

double low_obstacle_height = 0.07;
double obstacle_height = 0.14;
double min_ratio = 0.35;
int min_observed_voxels = 2;

bool unknown_as_occupied = true;
uint8_t passable_cost = 50;

double inflation_radius = 0.33;
```

命名可以根据仓库风格微调，但不要太具体。

## 验收标准

* ROGMap 更新后可以生成一个 2D value buffer。
* 可以生成一个 mask buffer 给 DynamicLayer。
* RViz/debug 能看到 value map 或 height map。
* 低矮障碍能被标记为 PASSABLE，而高障碍标记为 OCCUPIED。
* 未观测区域按参数决定是否作为障碍。

# 阶段 3：扩展 ProbMap 时间戳和 fading

## 目标

让 ROGMap 具备基本动态遗忘能力，减少动态物体尾迹。暂时不做动态障碍聚类。

## 修改 ProbMap

在 ProbMap 增加：

```cpp
std::vector<float> last_hit_time_;
std::vector<float> last_update_time_;
```

可选增加 active list：

```cpp
std::vector<int> active_ids_;
std::vector<uint8_t> active_flags_;
```

## 更新规则

当 voxel hit：

```text
occupancy += l_hit
last_hit_time = now
last_update_time = now
加入 active list
标记对应 column dirty
```

当 voxel miss：

```text
occupancy += l_miss
last_update_time = now
标记对应 column dirty
```

当 map sliding/reset：

```text
timestamp reset
相关 column dirty
```

## fading 规则

新增函数：

```cpp
void applyDecay(double now);
```

泛化命名即可，不要写成过细的 `applyDynamicObstacleFading`。

逻辑：

```text
遍历 active occupied voxel
如果 now - last_hit_time < keep_time:
    保留
否则:
    occupancy 按 decay_rate 衰减
    如果状态从 occupied 变成 free/unknown:
        更新 inflation/counter 状态
        标记 column dirty
```

参数：

```cpp
bool decay_en = true;
double keep_time = 0.4;
double decay_time = 1.2;
```

内部计算：

```cpp
decay_rate = (l_occ - l_free) / decay_time;
```

## 关键注意

ROGMap 原有 InfMap/CounterMap 依赖 voxel 状态变化。不要只改 occupancy_buffer_ 而不通知 inflation/counter map。

需要找到原有概率更新中从 old state 到 new state 的更新路径，复用它。如果没有统一入口，新增一个内部函数，例如：

```cpp
void updateCellState(const Vec3i & id, GridType old_type, GridType new_type);
```

名称可根据原代码风格调整。

## 验收标准

* 动态障碍移走后，occupied cell 能在 keep_time + decay_time 后逐渐消失。
* inflation map 不残留旧障碍。
* 2D layer 对应区域会重新变为 free/unknown。
* 不实现动态聚类。

# 阶段 4：ROGMap 主流程集成 2D layer 和 DynamicLayer

## 目标

让 ROGMap::updateMap 完成以下流程：

```text
1. 更新机器人状态
2. 更新 3D occupancy
3. 执行 fading/decay
4. 更新 2D layer
5. 用 2D layer 的 mask 更新 DynamicLayer ESDF
6. 发布/刷新只读接口
```

## 修改 ROGMap 类

新增成员，例如：

```cpp
std::shared_ptr<ProjectionLayer> layer_;
std::shared_ptr<DynamicLayer> field_;
std::shared_ptr<MapQueryAdapter> query_;
```

命名可根据代码风格调整，保持泛化：

* `layer_` 表示二维投影/地形层。
* `field_` 表示距离场。
* `query_` 表示对 planner 的只读接口。

不要命名成：

```cpp
terrain_layer_2d_
esdf_2d_dynamic_layer_
```

除非当前项目风格就是这样。

## updateMap 伪代码

```cpp
void ROGMap::updateMap(const PointCloud & cloud, const Pose & pose)
{
  if (cloud.empty()) {
    return;
  }

  const double now = getSystemWalltimeNow();

  std::unique_lock<std::shared_mutex> lock(update_mtx_);

  updateRobotState(pose);
  setUpdateTime(now);

  updateProbMap(cloud, pose);

  if (cfg_.decay_en) {
    applyDecay(now);
  }

  if (cfg_.layer_en) {
    layer_->update(*this);
  }

  if (cfg_.field_en) {
    field_->updateFromMask(
      layer_->width(),
      layer_->height(),
      layer_->resolution(),
      layer_->origin(),
      layer_->mask(),
      cfg_.inflation_radius);
  }

  refreshQuery();
}
```

## Query refresh

第一阶段可以让 query adapter 直接读 ROGMap 内部 layer 和 field，但必须加读写锁。

更推荐第一阶段也实现 snapshot：

```cpp
struct MapSnapshot {
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;

  std::vector<uint8_t> values;
  std::vector<uint8_t> types;
  std::vector<double> distances;
};
```

ROGMap 每次更新完后生成 shared snapshot。

Planner 的 MapQueryInterface 只读 snapshot，不直接读更新中的 buffer。

如果实现 snapshot 需要时间，先用 shared_mutex 保证安全，但代码中留 TODO：后续切换双缓冲 snapshot。

## 验收标准

* 一次 ROGMap 更新后，2D value buffer、mask、distance field 都同步刷新。
* Planner 可以通过接口读取 value 和 evaluate。
* 不需要 ROS topic 传大地图。
* 可以通过 debug topic 可视化 2D layer 和 ESDF。

# 阶段 5：接口注册与进程内通信

## 目标

支持 planner 在同一进程中获取 ROGMap 的查询接口。

这不是 ROS topic 通信，而是 C++ 进程内共享对象。可以理解为 dependency injection。

## 推荐实现

在 ROGMap ROS2 wrapper 中提供：

```cpp
std::shared_ptr<MapQueryInterface> getQueryInterface() const;
```

Planner 节点提供：

```cpp
void setMap(const std::shared_ptr<MapQueryInterface> & map);
```

或者如果当前是插件结构，则提供一个简单的 registry：

```cpp
class MapRegistry {
public:
  static void set(const std::shared_ptr<MapQueryInterface> & map);
  static std::shared_ptr<MapQueryInterface> get();
};
```

注意：

* Registry 只作为过渡方案。
* 更好的长期方案是同一 main/component 中构造 ROGMap 和 planner，然后把接口指针注入 planner。
* 不要通过 ROS topic 发布完整 ESDF 给 planner 作为主链路。
* Debug topic 可以保留。

## 验收标准

* Planner 可以拿到 MapQueryInterface 指针。
* Planner 不 include ROGMap 内部头文件。
* Planner 不访问 ROGMap 私有 buffer。
* Planner 只调用 `worldToMap / value / values / evaluate` 等泛化接口。

# 阶段 6：Planner 最小适配

虽然本任务重点是 ROGMap，但为了验证闭环，需要最小改 planner。

## 修改内容

将 planner 中对 Nav2 costmap 的查询替换为 MapQueryInterface：

```text
costmap_->worldToMap      → map_->worldToMap
costmap_->mapToWorld      → map_->mapToWorld
costmap_->getCost         → map_->value
costmap_->getCharMap      → map_->values
esdf_map_->evaluate       → map_->evaluate
```

保留现有 Smac/A*/MINCO 主体逻辑。

如果某些地方需要地图尺寸：

```cpp
map_->sizeX()
map_->sizeY()
map_->resolution()
```

## 注意

不要在本阶段彻底重写 planner。
不要改优化器数学逻辑。
不要改控制器。

## 验收标准

* planner 编译通过。
* planner 可以在无 Nav2 costmap 的情况下进行路径搜索和 MINCO 优化。
* collision check 只依赖 MapQueryInterface。
* 原有 DynamicLayer 不再从 `/global_costmap/voxel_grid` 更新。

# 阶段 7：Debug 与可视化

为调试增加 ROS topic，但这些 topic 不是 planner 主链路。

建议发布：

```text
/rog_map/layer/value
/rog_map/layer/type
/rog_map/layer/height
/rog_map/field/points
/rog_map/occ_cloud
```

可视化建议：

* value map：用 OccupancyGrid。
* type map：用不同整数表示 UNKNOWN/FREE/PASSABLE/OCCUPIED。
* height map：可用 PointCloud2，高度或 intensity 表示 height。
* field：可用 PointCloud2，intensity 表示 dist。

# 最终验收标准

完成后应满足：

1. ROGMap 可以接收世界系 dense cloud 和 odom pose 更新 3D occupancy。
2. ROGMap 内部可以从 3D occupancy 生成二维 layer。
3. 二维 layer 可以区分 unknown、free、passable、occupied。
4. DynamicLayer 被 ROGMap 调用，从 layer mask 生成 2D signed ESDF。
5. Planner 通过泛化 MapQueryInterface 读取地图。
6. Planner 不直接依赖 Nav2 costmap。
7. Planner 不直接访问 ROGMap 内部 buffer。
8. 暂时没有动态障碍聚类代码。
9. 暂时没有轮速接口代码。
10. 代码可以编译，关键路径有注释，新增参数有默认值。
11. Debug topic 可以帮助检查 layer 和 ESDF 是否正确。
12. ROGMap sliding 时不会让 planner 读到错位地图。

# 执行策略

请按阶段执行。每一阶段完成后：

1. 列出修改文件。
2. 简述实现。
3. 运行编译或静态检查。
4. 修复编译错误。
5. 继续下一阶段。

不要因为某个局部函数名或文件位置不确定就停止。请先搜索仓库，理解当前结构，再选择最小侵入实现。遇到现有代码接口不匹配时，优先写 adapter，而不是大规模重构。

如果必须做设计取舍，优先级如下：

```text
可编译 > 接口稳定 > 功能闭环 > 性能优化 > 命名完美
```

# 禁止事项

1. 不要实现轮速融合。
2. 不要实现动态障碍聚类。
3. 不要将 planner 与 ROGMap 私有实现强耦合。
4. 不要用 ROS topic 作为 planner 查询 ESDF 的主链路。
5. 不要继续依赖 `/global_costmap/voxel_grid` 作为 DynamicLayer 输入。
6. 不要把函数名写得过度具体，例如 `evaluateTerrainESDF2DForPlanner`。
7. 不要在未理解原 ROGMap inflation/counter 更新机制前直接修改 occupancy_buffer_。
8. 不要把所有代码写进一个大文件。
