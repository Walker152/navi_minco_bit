# 单 MID360 与 Point-LIO Intra-Process 启动设计

## 背景

当前 `src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py` 在同一个
`component_container_mt` 中加载 Livox driver 与 Point-LIO，并默认使用双雷达 driver
参数。单雷达需要独立启动入口和 driver 参数，但不应复制整套 container 逻辑，也不应
修改当前双雷达使用的 Point-LIO 参数文件。

## 目标

- 在 Point-LIO 现有 launch 同目录新增单雷达 wrapper launch。
- 在 `livox_ros_driver2/config` 新增单 MID360 component 参数文件。
- 单雷达仍使用现有 composable node container 与 intra-process 通信。
- 单雷达 Point-LIO 订阅 `livox/lidar` 和 `livox/imu`。
- 保持现有双雷达默认 IMU 接口 `livox/imu_192_168_1_135` 不变。

## 范围

本次属于 `launch / communication` 与 `parameter / yaml` 模块，目标是新增单雷达启动入口。
允许新增两个文件，并对通用 intra-process launch 做一个局部参数扩展。

本次不修改：

- Point-LIO C++ 源码及 `config/mid360.yaml`。
- Livox driver C++ 点云、IMU 或内部融合逻辑。
- 双雷达 driver YAML、雷达 JSON、topic 命名、QoS、callback group 与 container 类型。
- planner、controller、ROGMap 或比赛策略。

## 方案选择

采用轻量 wrapper，而非复制完整 container launch。wrapper 只为通用 launch 提供单雷达
默认值，Livox driver 与 Point-LIO 的 component 定义仍只有一份。

为避免复制 Point-LIO YAML，通用 launch 新增 `pointlio_imu_topic` 参数，并把它作为
`common.imu_topic` 的 ROS 参数覆盖传给 `LaserMappingNode`。通用 launch 的默认值保持
`livox/imu_192_168_1_135`，因此现有双雷达直接启动行为不变；单雷达 wrapper 显式传入
`livox/imu`。

## 文件设计

### 通用 launch

修改：

`src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`

- 声明 `pointlio_imu_topic` launch 参数。
- 默认值为当前生效接口 `livox/imu_192_168_1_135`。
- 在 Point-LIO component 的参数列表中，于 YAML 后追加
  `{"common.imu_topic": pointlio_imu_topic}`，使显式 launch 参数优先。
- 不改变 driver 参数入口、container、component、intra-process 或日志设置。

### 单雷达 wrapper

新增：

`src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py`

wrapper 使用 `IncludeLaunchDescription` 加载同目录的通用 launch，并传入：

- `driver_params_file`：`single_MID360_component.yaml`
- `pointlio_params_file`：现有 `point_lio/config/mid360.yaml`
- `pointlio_imu_topic`：`livox/imu`
- `use_intra_process`：`true`
- `container_name`：`single_livox_pointlio_container`
- `log_level`：`info`

启动命令为：

```bash
ros2 launch point_lio single_livox_pointlio_intra_process.launch.py
```

### 单雷达 driver 参数

新增：

`src/perception/livox_ros_driver2/config/single_MID360_component.yaml`

以现有 `mixed_MID360_component.yaml` 为结构参考，保留传输格式、频率、frame 和输出类型，
只把雷达拓扑相关项调整为：

```yaml
multi_topic: 0
user_config_path: "$(find-pkg-share livox_ros_driver2)/config/MID360_config.json"
enable_internal_lidar_merge: false
enable_merge_debug: false
```

保留现有 merge 参数作为兼容参数，但单雷达模式不依赖它们。单雷达实际 IP 继续由
`MID360_config.json` 的 `lidar_configs` 决定。

## 数据流与行为保持

单雷达启动后的链路为：

```text
MID360 -> livox_ros::DriverNode -> livox/lidar -> point_lio::LaserMappingNode
                              -> livox/imu   -> point_lio::LaserMappingNode
```

两个 component 仍位于同一 `component_container_mt`，并继续对两者设置
`use_intra_process_comms=true`。

现有通用 launch 不传新参数时，Point-LIO 仍订阅 `livox/imu_192_168_1_135`，因此双雷达
默认行为保持不变。单雷达 wrapper 只在本次启动实例中覆盖为 `livox/imu`。

## 错误处理

- wrapper 通过包 share 路径定位通用 launch 和参数文件，不使用工作空间绝对路径。
- 参数文件继续使用 `ParameterFile(..., allow_substs=True)`，保证 `find-pkg-share` 替换生效。
- 不增加运行时 fallback；缺失文件或无效 YAML 由 ROS 2 launch/参数加载阶段明确报错。

## 验证

未经用户明确许可不执行构建或 ROS 运行测试。本次先执行以下静态验收：

- Python AST 解析两个 launch 文件。
- YAML 解析单雷达参数文件。
- 检查 wrapper 参数完整传入通用 launch。
- 检查通用 launch 的两个 component 仍处于同一个 container，且均保留 intra-process。
- 检查单雷达 YAML 使用 `multi_topic: 0`、`MID360_config.json` 并关闭内部融合。
- 检查 `mid360.yaml` 与双雷达 YAML 没有被修改。
- 使用 `git diff` 与 `rg` 审计修改边界。

真机验收需在用户后续允许并具备雷达环境时执行：

```bash
ros2 launch point_lio single_livox_pointlio_intra_process.launch.py
ros2 component list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
```

## 风险

- 本设计只新增单雷达入口，不改变 driver 内部融合实现。
- `pointlio_imu_topic` 的默认值必须保持现状，否则会意外改变双雷达链路。
- 单雷达真机 IP、主机网卡地址及端口仍由现有 `MID360_config.json` 决定，本次不调比赛参数。
