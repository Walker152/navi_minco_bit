# 哨兵车 MID360 可选 LIO 方案对比

本文档讨论四套候选 LIO：

1. FAST-LIO2
2. RKO-LIO
3. DLIO，也就是 `vectr-ucla/direct_lidar_inertial_odometry`
4. D-LIO，也就是 `robotics-upo/D-LIO`

目标是判断它们是否适合当前哨兵车的硬件、ROS 2 工程链路和 RoboMaster 场地环境，尤其关注高速旋转、颠簸、碰撞冲击和动态障碍物对定位建图的影响。

当前工程已有一套 LIO 作为基线，本文不展开基线方案本身，只讨论上面三个候选。

当前相关配置：

- `src/perception/Point-LIO/config/mid360.yaml`
- `src/perception/lidar_merger/config/dual_lidar_merger.yaml`

## 1. 当前车体与环境特点

当前系统大致条件如下：

- 传感器：Livox MID360
- 当前启动方式：双 MID360 通过 `lidar_merger` 合并到 `/livox/lidar`
- 当前基线里程计使用的 IMU：`/livox/imu_192_168_1_135`
- 软件框架：ROS 2
- 下游模块：Nav2、重定位、局部代价地图、路径规划
- 运动特点：高速 yaw 旋转、急停急启、地面颠簸、碰撞冲击
- 环境特点：RoboMaster 场地，大平面、长墙、低纹理区域、反光结构、动态机器人较多

这类场景对 LIO 的关键要求：

1. 高频输出和低延迟，避免下游 Nav2 里程计滞后。
2. 正确使用 MID360 点内时间戳，快速旋转时要能去畸变。
3. 对短时 IMU 异常、颠簸和冲击有一定容忍能力。
4. 对动态物体不能过度敏感。
5. 接入 ROS 2 的工作量不能过大。
6. 能稳定输出标准 odom、tf 和点云，便于替换当前 `/aft_mapped_to_init`、`/cloud_registered` 链路。

需要特别注意：碰撞鲁棒性不完全由算法决定。如果 LiDAR 和 IMU 的安装结构在碰撞后发生微小相对位移，外参会变成时变量，任何 LIO 都会明显变差。

注意：`DLIO` 和 `D-LIO` 名字很像，但不是同一个项目。本文会分别讨论。

## 2. 四套方案总体结论

| 方案 | 推荐定位 | 当前适配度 | 主要优势 | 主要风险 |
| --- | --- | --- | --- | --- |
| FAST-LIO2 | 成熟稳定对照组 | 高 | Livox 适配成熟，资料多，实时性强 | 官方主线偏 ROS 1，ROS 2 需选可靠移植版 |
| RKO-LIO | 新 ROS 2 通用路线 | 中高 | ROS 2 接口现代，传感器特化少 | MID360 CustomMsg 和点内时间戳适配需确认 |
| DLIO | 去畸变与连续时间对照组 | 中 | 连续时间轨迹和直接法，适合验证高速运动 deskew | 工程接入和 ROS 2 适配成本较高 |
| D-LIO | ROS 2 + Fast-TDF 直接法新方案 | 中高 | ROS 2 Humble，明确面向 Livox 等雷达，参数接口现代 | 项目较新，比赛实车案例需要验证 |

建议测试顺序：

```text
单雷达 FAST-LIO2
  -> 双雷达 FAST-LIO2
  -> RKO-LIO 离线 bag
  -> RKO-LIO 在线
  -> D-LIO 离线 bag
  -> D-LIO 在线
  -> DLIO 离线 bag
  -> DLIO 在线
```

不要一开始就用双雷达测试所有算法。建议先用单 MID360 跑通，确认算法本体表现后，再加入双雷达合并。否则很难区分问题来自算法、双雷达外参，还是时间同步。

## 3. FAST-LIO2

参考：

- [FAST-LIO GitHub](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO2 arXiv](https://arxiv.org/abs/2107.06829)

### 3.1 基本原理

FAST-LIO2 是直接法 LiDAR-Inertial Odometry。它不再像传统 LOAM 系列那样先显式提取边缘点和平面点，而是直接使用原始点云参与 scan-to-map 匹配。

其核心流程可以理解为：

1. IMU 预测当前时刻位姿。
2. 根据点内时间戳对 LiDAR 点云进行运动补偿。
3. 将当前点云投影到地图坐标系。
4. 在增量地图中寻找最近邻点。
5. 构造点到局部平面的几何残差。
6. 使用迭代卡尔曼滤波更新状态。
7. 将当前有效点加入增量地图。

FAST-LIO2 使用 ikd-Tree 维护增量地图，使最近邻搜索和地图更新都能实时运行。它对 Livox 这类非重复扫描雷达比较友好，因为它不强依赖固定线束结构或传统特征提取。

### 3.2 对 MID360 的适配程度

适配程度：高。

原因：

- FAST-LIO2 原本就面向 Livox 系列做过适配。
- MID360 与 FAST-LIO2 的输入需求匹配度较高，关键是保留点内时间戳。
- 社区里 FAST-LIO2 + MID360 的使用案例较多。

需要确认：

1. 使用的 ROS 2 移植版是否支持 `livox_ros_driver2::msg::CustomMsg`。
2. 如果转换为 `sensor_msgs::msg::PointCloud2`，点内时间字段是否保留。
3. LiDAR frame、IMU frame、base frame 的外参是否和当前系统一致。

对于 MID360，高速旋转时最关键的是点内时间戳。如果某个移植版只把整帧点云当作同一时刻处理，那么 FAST-LIO2 的实际表现会明显下降。

### 3.3 对当前双雷达配置的适配

适配程度：中高。

当前系统中双雷达先由 `lidar_merger` 合并，再送入里程计。FAST-LIO2 理论上也可以吃合并后的点云，但要满足两个条件：

1. 合并后的点云仍然保留每个点的 offset time。
2. 合并点云的参考坐标系与所使用的 IMU 外参一致。

如果双雷达合并点云已经出现前后墙面错位，FAST-LIO2 会把错位点当成真实环境参与匹配，结果通常是地图变厚、局部位姿抖动、快速转弯后漂移。

建议测试流程：

1. 先只用前雷达和对应 IMU 跑 FAST-LIO2。
2. 确认单雷达原地旋转地图不厚。
3. 再加入后雷达合并。
4. 如果加入后雷达后墙面变双层，优先调 `extrinsic_back_to_front`，不要先调 FAST-LIO2 滤波参数。

### 3.4 对高速旋转和颠簸碰撞的适配

适配程度：高，但不如专门逐点更新的高带宽 LIO。

FAST-LIO2 对快速运动的鲁棒性已经很强，论文和工程实践中都证明它能处理较高角速度场景。但它本质上仍是以 LiDAR scan 为核心的 scan-to-map 更新。对哨兵车这种高速自旋、急停、碰撞冲击频繁的场景，它很适合作为成熟对照组。

碰撞场景下，FAST-LIO2 主要风险：

- IMU 短时饱和导致预测异常。
- 碰撞后 LiDAR-IMU 外参发生物理变化。
- 车体结构件进入近距离点云，形成错误地图约束。

这些风险无法单靠算法参数完全解决。

### 3.5 对 RoboMaster 场地的适配

适配程度：高。

优点：

- 对大部分室内结构化环境表现稳定。
- 低延迟、实时性好。
- 对 Livox 固态雷达友好。
- 适合作为比赛工程基线。

弱点：

- 对动态机器人没有语义剔除，动态物体进入局部地图可能带偏匹配。
- 长墙、开阔平面、低纹理区域中，几何约束退化时仍可能漂。
- 若双雷达合并外参不准，地图会明显变厚。

### 3.6 接入当前工程的工作量

工作量：中。

主要工作：

1. 选择一个可靠 ROS 2 版本。
2. 对接 `/livox/lidar` 和 `/livox/imu_192_168_1_135`。
3. 统一输出 odom 和 tf frame。
4. 将输出 remap 到当前下游需要的话题，例如替代 `/aft_mapped_to_init`。
5. 确认输出点云可替代 `/cloud_registered` 或调整下游订阅。

### 3.7 推荐测试指标

测试时重点看：

- `/odom` 输出频率是否稳定。
- 快速原地旋转时墙面是否变厚。
- 急刹急启后是否出现跳变。
- 和当前基线在同一条路线、同一段 bag 上的轨迹差异。
- Nav2 局部代价地图是否抖动。

## 4. RKO-LIO

参考：

- [RKO-LIO GitHub](https://github.com/PRBonn/rko_lio)
- [RKO-LIO ROS package](https://index.ros.org/p/rko_lio/)
- [RKO-LIO usage](https://prbonn.github.io/rko_lio/pages/ros/usage.html)

### 4.1 基本原理

RKO-LIO 的目标是做一个鲁棒、较少依赖传感器特定建模的 LiDAR-Inertial Odometry。它强调通用性和 ROS 2 友好接口。相比很多针对 Livox 或特定线束雷达深度适配的 LIO，RKO-LIO 更像是一个现代 ROS 2 环境下的通用 LIO 框架。

它的典型启动方式比较直接：

```bash
ros2 launch rko_lio odometry.launch.py imu_topic:=<imu_topic> lidar_topic:=<lidar_topic> base_frame:=base_link
```

这种接口对当前仓库很有吸引力，因为可以更快用 rosbag 做离线对照。

### 4.2 对 MID360 的适配程度

适配程度：中高，需要验证输入格式。

优点：

- ROS 2 接口友好。
- 不强依赖特定 LiDAR 模型。
- 更适合快速替换、离线 bag 对照和工程化实验。

需要重点确认：

1. 是否能直接读取 Livox CustomMsg。
2. 如果需要 PointCloud2，是否支持 MID360 点内时间字段。
3. 输入点云的时间戳字段名称是否和 RKO-LIO 预期一致。
4. 是否需要额外配置 LiDAR-IMU 外参和 frame。

如果 RKO-LIO 只能读取普通 PointCloud2 且没有 per-point time，高速旋转测试会不公平，因为它会缺少 MID360 的关键时间信息。

### 4.3 对当前双雷达配置的适配

适配程度：中。

RKO-LIO 的通用性有利于接入标准 PointCloud2，但当前双雷达合并链路使用的是 Livox 风格数据。若要测试 RKO-LIO，建议两种方式：

方式一：单雷达测试。

```text
/livox/lidar_192_168_1_135 + /livox/imu_192_168_1_135
```

方式二：合并后转标准 PointCloud2，并保留点内时间字段。

第二种方式需要额外确认字段保留情况，否则快速旋转下 deskew 可能失效。

### 4.4 对高速旋转和颠簸碰撞的适配

适配程度：中高。

RKO-LIO 的优势是鲁棒和通用，但它不是专门为极端高带宽逐点更新设计的方案。对当前车来说，它最适合回答下面这个问题：

> 如果不用 Livox 特化路线，一个新的 ROS 2 通用 LIO 在相同 bag 上是否更稳、更容易接入？

高速旋转场景下，RKO-LIO 的表现很大程度取决于：

- 点内时间戳是否正确输入。
- IMU 与 LiDAR 外参是否准确。
- deskew 模块是否充分使用 per-point time。

颠簸和碰撞场景下，它仍然需要依赖稳定机械外参。若碰撞导致雷达支架微变形，RKO-LIO 同样会退化。

### 4.5 对 RoboMaster 场地的适配

适配程度：中高。

优点：

- ROS 2 接入成本低。
- 适合快速做横向测试。
- 对传感器泛化能力更强，后续换雷达也更方便。

弱点：

- MID360/Livox 专项资料少于 FAST-LIO2。
- 极端高速旋转下需要和当前基线做同 bag 对照。
- 对动态障碍物同样没有天然语义过滤。

### 4.6 接入当前工程的工作量

工作量：中低到中，取决于点云格式。

如果能直接读取当前话题，工作量较低。若需要转换 CustomMsg 到 PointCloud2 并保留点内时间，工作量会上升。

建议先离线验证：

```bash
ros2 bag record /livox/lidar_192_168_1_135 /livox/imu_192_168_1_135 /tf /tf_static
```

然后用单雷达 bag 跑 RKO-LIO，确认：

- 是否正常初始化。
- 输出轨迹是否连续。
- 快速旋转时地图是否厚。
- 是否存在明显时间戳警告。

### 4.7 推荐测试指标

重点看：

- 初始化是否稳定。
- 对同一段 bag 的轨迹是否比 FAST-LIO2 更平滑。
- 高频旋转时是否丢失跟踪。
- 接入 Nav2 时 TF 是否更清晰。
- 是否需要大量额外参数调试。

## 5. DLIO

参考：

- [DLIO GitHub](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)
- [DLIO paper](https://arxiv.org/abs/2203.03749)

### 5.1 基本原理

DLIO，全称 Direct LiDAR-Inertial Odometry，是直接法 LiDAR-IMU 里程计。它的一个重要特点是使用连续时间轨迹思想，对 LiDAR 扫描期间的运动进行建模，从而更好地处理运动畸变。

可以把它理解为：

1. IMU 提供扫描期间的连续运动先验。
2. LiDAR 点云不被简单看成一个静态整帧。
3. 系统根据连续时间状态对点云进行 deskew。
4. deskew 后的点云再与地图进行直接匹配。

这类思路对高速旋转、快速平移和颠簸时的点云畸变比较有意义。

### 5.2 对 MID360 的适配程度

适配程度：中。

DLIO 官方仓库主要是通用 LIO 风格，并非专门面向 MID360。它可以作为“连续时间 deskew 能力”的对照方案，但接入 MID360 时需要重点确认：

1. 是否支持 Livox CustomMsg。
2. 是否需要把 MID360 转成 PointCloud2。
3. PointCloud2 中是否保留 per-point timestamp。
4. IMU 频率、单位、噪声参数是否与 MID360 内置 IMU 匹配。

如果点内时间戳处理不正确，DLIO 的核心优势会被削弱。

### 5.3 对当前双雷达配置的适配

适配程度：中低。

DLIO 更适合先做单雷达测试。双雷达合并后，如果每个点来自不同雷达，且每个雷达有自己的空间外参和时间戳，连续时间 deskew 会更复杂。

建议：

1. 不要第一轮就用合并 `/livox/lidar` 测 DLIO。
2. 先用单 MID360 和对应 IMU 测。
3. 单雷达稳定后，再考虑合并点云。
4. 如果双雷达合并后轨迹变差，优先怀疑合并外参和点时间字段。

### 5.4 对高速旋转和颠簸碰撞的适配

适配程度：理论上高，工程上需验证。

DLIO 的连续时间和直接法思路对高速运动很有吸引力。对哨兵车来说，它可以重点验证：

- 快速旋转时点云去畸变是否比 FAST-LIO2 更好。
- 过坡、颠簸时局部地图是否更薄。
- 急停急启后轨迹是否更连续。

但碰撞冲击本身仍然有硬件边界：

- IMU 饱和会影响连续状态估计。
- LiDAR-IMU 外参变化会使 deskew 失效。
- 短时间强振动可能让 IMU 积分和 LiDAR 匹配都变差。

因此 DLIO 更适合用于“deskew 对照实验”，不应默认认为它一定比 FAST-LIO2 或当前基线更适合比赛在线运行。

### 5.5 对 RoboMaster 场地的适配

适配程度：中。

优点：

- 对运动畸变建模更有针对性。
- 适合高速运动数据集对照。
- 能帮助判断当前系统地图变厚是否由 deskew 不足导致。

弱点：

- MID360 工程资料少于 FAST-LIO2。
- ROS 2 接入可能需要额外工作。
- 动态障碍物同样会影响直接匹配。
- 双雷达合并场景需要额外处理。

### 5.6 接入当前工程的工作量

工作量：中高。

主要工作：

1. 确认 ROS 2 可用版本或完成移植。
2. 适配 MID360 点云输入。
3. 保留 per-point timestamp。
4. 配置 LiDAR-IMU 外参和 IMU 噪声。
5. 将输出 odom/tf 接到当前 Nav2 和重定位链路。

建议先用离线 bag 做评估，不要直接在线替换当前主定位。

### 5.7 推荐测试指标

DLIO 测试重点应该放在运动畸变：

- 快速原地旋转时墙面厚度。
- 高速 S 弯时轨迹是否连续。
- 急停急启时局部地图是否撕裂。
- 过颠簸区域后是否能快速恢复。
- deskew 后点云是否比 FAST-LIO2 更薄。

## 6. D-LIO

参考：

- [D-LIO GitHub](https://github.com/robotics-upo/D-LIO)

### 6.1 和 DLIO 的关系

`robotics-upo/D-LIO` 与上一节的 `vectr-ucla/direct_lidar_inertial_odometry` 不是同一个项目，也不是直接 fork 关系。它们名字接近，且都属于 direct LiDAR-inertial odometry，但核心实现路线不同。

简单区分：

| 项目 | 仓库 | 核心特点 |
| --- | --- | --- |
| DLIO | `vectr-ucla/direct_lidar_inertial_odometry` | 连续时间运动校正，轻量级 direct LIO |
| D-LIO | `robotics-upo/D-LIO` | 基于 Fast-TDF / Truncated Distance Field 地图的 direct LIO |

因此文档中出现 `DLIO` 时，指 vectr-ucla 的项目；出现 `D-LIO` 时，指 robotics-upo 的项目。

### 6.2 基本原理

D-LIO 的全称是：

```text
D-LIO: 6DoF Direct LiDAR-Inertial Odometry based on Simultaneous Truncated Distance Field Mapping
```

它的核心思路是用 Truncated Distance Field，也就是截断距离场，维护局部地图，并在这个地图上做直接点云配准。和传统点到平面最近邻匹配相比，TDF 地图把空间中的距离信息组织成场，可以让点云和地图之间的残差构造更直接。

可以粗略理解为：

1. IMU 提供运动预测。
2. LiDAR 点云根据时间戳做 deskew 或 unwarp。
3. 当前点云与 Fast-TDF 地图进行直接配准。
4. 使用非线性优化估计当前位姿。
5. 同步更新 TDF 地图。

这种路线的吸引力在于，它不是单纯依赖若干最近邻点拟合平面，而是利用局部距离场表达地图。对几何结构连续、点云密集的室内场景，理论上有不错的稳定性。

### 6.3 对 MID360 的适配程度

适配程度：中高。

D-LIO 仓库是 ROS 2 Humble 工程，并且配置中能看到与 Livox、点云时间戳、IMU 单位、辅助 LiDAR 相关的参数。对当前仓库而言，这是它比 vectr-ucla DLIO 更有吸引力的地方。

重点需要确认：

1. 它是否能直接读取当前 Livox CustomMsg。
2. 如果使用 PointCloud2，MID360 的点内时间戳字段是否能正确进入 D-LIO。
3. `timestamp_mode`、`lidar_type` 等参数是否需要针对 MID360 调整。
4. MID360 内置 IMU 的加速度单位是否和 `imu_acc_in_ms2` 等参数一致。

如果它能稳定读取 MID360 点云并正确 unwarp，那么它会比原 DLIO 更适合当前 ROS 2 工程链路。

### 6.4 对当前双雷达配置的适配

适配程度：中高，但需要实测。

D-LIO 的配置中存在辅助点云相关思路，例如辅助 LiDAR 输入或 aux cloud。对当前双 MID360 车来说，这是一个值得注意的点。

但是不要直接假设它天然支持当前 `lidar_merger` 的双雷达合并结果。需要分两步验证：

1. 单 MID360 输入是否稳定。
2. 双 MID360 合并或辅助点云输入是否稳定。

建议第一轮仍然只用前雷达：

```text
/livox/lidar_192_168_1_135
/livox/imu_192_168_1_135
```

单雷达稳定后，再测试：

```text
/livox/lidar
```

或者进一步研究 D-LIO 的辅助点云输入是否能直接接后雷达点云。

### 6.5 对高速旋转和颠簸碰撞的适配

适配程度：中高。

D-LIO 的直接法和 TDF 地图路线，理论上适合高频局部定位和室内连续结构环境。对哨兵车来说，它值得测试的点在于：

- TDF 地图是否能让墙面、地面等连续结构匹配更稳定。
- 快速旋转时 unwarp 后地图是否比 FAST-LIO2 更薄。
- 颠簸后局部地图是否更容易恢复。
- 双雷达输入是否能让低纹理区域更稳定。

但碰撞冲击仍然有硬件边界。若 LiDAR 和 IMU 外参在撞击后变化，D-LIO 也会退化。

### 6.6 对 RoboMaster 场地的适配

适配程度：中高。

优点：

- ROS 2 Humble 工程，对当前系统更贴近。
- 直接法 + TDF 地图对室内连续结构有吸引力。
- 明确关注 Livox 等雷达输入。
- 辅助点云相关接口可能对双 MID360 有价值。

风险：

- 项目较新，比赛实车案例少于 FAST-LIO2。
- TDF 地图参数可能需要较多实测调节。
- 动态机器人进入局部地图时，仍可能影响直接配准。
- 若点云时间戳处理不正确，高速旋转下优势会消失。

### 6.7 接入当前工程的工作量

工作量：中。

比 FAST-LIO2 的 ROS 1 主线更接近当前工程，但仍需确认输入消息格式。主要工作：

1. 确认 D-LIO 使用的点云输入类型。
2. 将 MID360 点云转换到它需要的字段格式。
3. 配置 LiDAR-IMU 外参、IMU 单位和噪声。
4. 将输出 odom/tf 接入当前 Nav2 链路。
5. 确认是否能替代当前 `/aft_mapped_to_init` 和 `/cloud_registered`。

建议先离线 bag 验证，再在线实车。

### 6.8 推荐测试指标

D-LIO 测试重点：

- 初始化是否稳定。
- 原地快速旋转时墙面是否变厚。
- TDF 地图是否吃动态物体。
- 地面颠簸后是否能快速恢复。
- 与 FAST-LIO2 在同一 bag 上的轨迹差异。
- 双雷达合并输入是否比单雷达更稳定。

## 7. 四套方案实车测试方案

### 7.1 数据采集

建议先录制单雷达 bag：

```bash
ros2 bag record \
  /livox/lidar_192_168_1_135 \
  /livox/imu_192_168_1_135 \
  /tf \
  /tf_static
```

再录制双雷达合并 bag：

```bash
ros2 bag record \
  /livox/lidar \
  /livox/lidar_192_168_1_135 \
  /livox/lidar_192_168_1_122 \
  /livox/imu_192_168_1_135 \
  /tf \
  /tf_static
```

每套算法都先跑单雷达 bag，再跑双雷达 bag。

### 7.2 测试路线

建议固定几段动作：

1. 静止 10 秒，用于初始化。
2. 原地慢速旋转 360 度。
3. 原地快速旋转 360 度。
4. 直线加速后急停。
5. S 弯高速行驶。
6. 经过轻微颠簸区域。
7. 靠近动态机器人或移动障碍。
8. 回到起点附近，观察局部闭合误差。

### 7.3 对比指标

| 指标 | 观察内容 |
| --- | --- |
| 输出频率 | odom 是否稳定，高速运动时是否掉频 |
| 地图厚度 | 墙面、地面、角点是否变厚或撕裂 |
| 快速旋转 | 原地转圈后是否漂移、是否双层墙 |
| 急停急启 | 位姿是否跳变 |
| 颠簸恢复 | 过颠簸后是否能在 1 到 2 秒内恢复 |
| 动态障碍 | 动态物体是否明显带偏轨迹 |
| Nav2 兼容 | costmap 是否抖动，tf 是否连续 |
| 工程复杂度 | 是否需要大量转换、移植和改 frame |

## 8. 对当前车的推荐结论

### 8.1 如果目标是尽快比赛可用

优先测 FAST-LIO2。

原因：

- Livox 适配成熟。
- 工程资料最多。
- 作为当前基线之外的成熟对照最有价值。
- 如果 FAST-LIO2 都不稳，通常说明时间同步、外参或输入点云有问题。

### 8.2 如果目标是找新的 ROS 2 方案

优先测 RKO-LIO。

原因：

- ROS 2 接口更现代。
- 适合离线 bag 快速对照。
- 接入 Nav2/TF 链路可能更清爽。

但要先确认它是否正确使用 MID360 的点内时间戳。

### 8.3 如果目标是测试 ROS 2 + Livox 新直接法

优先测 D-LIO。

原因：

- 它是 ROS 2 Humble 工程，对当前系统更贴近。
- 明确涉及 Livox、点云时间戳和辅助 LiDAR 输入。
- Fast-TDF 地图路线和 FAST-LIO2、RKO-LIO 都不同，可以形成有价值的横向对照。

但 D-LIO 项目较新，建议先离线 bag 测试，不要直接替换当前在线定位。

### 8.4 如果目标是研究高速运动畸变

优先测 DLIO。

原因：

- 连续时间和直接法对 deskew 很有价值。
- 可以验证地图变厚是否来自帧内运动畸变。
- 适合做高速旋转和颠簸数据的专项对比。

但 DLIO 不建议作为第一套在线替换方案，因为工程接入成本较高。

## 9. 最终建议

四套方案的推荐定位如下：

```text
FAST-LIO2：成熟实用对照，第一优先级
RKO-LIO：ROS 2 新方案对照，第二优先级
D-LIO：ROS 2 + Fast-TDF 直接法对照，第三优先级
DLIO：高速运动 deskew 对照，第四优先级
```

实车落地顺序：

```text
单 MID360 + FAST-LIO2
  -> 双 MID360 + FAST-LIO2
  -> 单 MID360 + RKO-LIO 离线 bag
  -> 单 MID360 + RKO-LIO 在线
  -> 单 MID360 + D-LIO 离线 bag
  -> 必要时 D-LIO 在线
  -> 单 MID360 + DLIO 离线 bag
  -> 必要时 DLIO 在线
```

最重要的前置检查：

```text
点内时间戳正确
  > LiDAR-IMU 外参准确
  > 双雷达外参准确
  > 时间同步稳定
  > 单雷达基线稳定
  > 双雷达合并稳定
  > 换新算法
```
