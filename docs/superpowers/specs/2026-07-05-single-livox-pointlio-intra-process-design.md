# Livox Driver 单/双 MID360 自适应与 Point-LIO Intra-Process 启动设计

## 背景

仓库已经具备 Livox driver 内部双 MID360 点云融合和一个通用 intra-process launch，
但当前实现仍直接使用 `enable_internal_lidar_merge`，缺少有效融合组合判定、完整配置校验，
以及 component unload 所需的线程和空指针保护。当前也没有
单雷达 wrapper 与单雷达 component YAML。

用户明确选择按实际接口区分 Point-LIO 的 IMU topic：

- 单雷达：`livox/imu`
- 双雷达：`livox/imu_192_168_1_135`

这个选择覆盖原始提示词中“Point-LIO 始终订阅 `/livox/imu`、双雷达由 driver 发布 front
IMU alias”的要求。Point-LIO C++ 不改，topic 差异只由 launch 参数覆盖。

## 目标

- driver 根据 `multi_topic` 与 `enable_internal_lidar_merge` 安全选择单雷达直接链路、
  双雷达融合链路或多 topic 调试链路。
- 单雷达发布 `livox/lidar` 与 `livox/imu`，不创建或进入融合逻辑。
- 双雷达融合点云发布到 `livox/lidar`，两台雷达的原始 IMU topic 保持不变。
- Point-LIO 单雷达时订阅 `livox/imu`，双雷达时订阅 front 原始 IMU topic。
- driver 融合配置错误在启动阶段失败，确认现有缓存有界，线程退出不解引用空对象。
- 单、双雷达都通过同一套 composable container 逻辑启动，并保持 intra-process。

## 任务边界

本次涉及：

- `odom / pointcloud`
- `launch / communication`
- `parameter / yaml`
- driver bug 修复、链路适配与退出安全

允许小幅改变 driver 的错误参数、初始化失败和 component unload 行为。
正常单雷达直出与现有双雷达融合算法、外参、topic 命名、发布频率、QoS 和 component
划分保持不变。

本次涉及比赛验证过的 Livox/Point-LIO 链路，采用最小改动策略。

本次不修改：

- Point-LIO C++、`config/mid360.yaml`、点云去畸变和 odom 输出逻辑。
- planner、controller、MPC、ROGMap、行为树或裁判系统。
- 雷达 JSON 中的 IP、主机网卡地址、端口和已验证外参。
- ROS topic 名、frame、QoS、timer、callback group 或 container 类型。
- 内部融合的坐标变换和消息构造算法。

## 方案选择

采用现有 `DriverNode -> Lddc -> InternalLidarMerger` 链路的局部补强，不增加外部融合节点，
不把逻辑移入 Point-LIO。

launch 采用轻量 wrapper。单、双 wrapper 与 Point-LIO 现有通用 launch 同目录，只传不同
的 driver YAML、Point-LIO IMU topic 和 container 名称，不复制 container/component 定义。

不实现双雷达 `/livox/imu` alias。双雷达 Point-LIO 直接使用现有 front IMU topic，避免
额外消息复制；这是用户对原始任务的明确覆盖。

## Driver 有效融合与配置校验

`DriverNode` 在读取参数后计算：

```cpp
const bool effective_internal_merge =
  enable_internal_lidar_merge && (multi_topic == 1);
```

只有 `effective_internal_merge` 为 true 时才校验、构造和配置 `InternalLidarMerger`。

四种组合行为：

| `multi_topic` | `enable_internal_lidar_merge` | 行为 |
|---:|---:|---|
| 0 | false | 单雷达直接发布 `livox/lidar` 与 `livox/imu` |
| 0 | true | 打一次明确 warning，忽略融合并走直接链路 |
| 1 | true | 双雷达原始多 topic + 融合点云 `livox/lidar` |
| 1 | false | 原始多 topic 调试链路，不保证 `livox/lidar` |

有效融合模式下逐项校验：

- `xfer_format` 为 0 或 1。
- 外参数组恰好为 6 项。
- `merge_max_interval_ms > 0`。
- front/back IP 非空且不同。
- `merge_output_topic` 非空。
- `merge_frame_id` 为空时使用 `frame_id`。

校验失败先记录具体错误，再抛出 `std::invalid_argument`，不启动 poll thread。单雷达或
多 topic 调试模式不校验、不解析、不使用 `merge_*` 参数。

## 融合队列审计边界

不修改底层融合队列结构、容量算法或生产端。现有 `LidarDataQueue` 在初始化时分配固定容量
的环形数组；队列满后生产端不再写入，因此缺一路雷达时不会无限分配内存。20 Hz 配置下
申请容量为 21，并向上取整为 32 帧。

保留现有匹配行为：两侧队列都可用时，时间差超限会弹出较旧一侧；融合成功后立即弹出
两侧已使用帧。不新增 `merge_max_queue_size`、`QueuePeekNewest()` 或孤帧年龄清理。

只修正与现有消费逻辑直接相关的生命周期风险：融合前复制日志和统计需要的时间、点数等
标量，弹出队列后不再访问 `QueuePeek()` 返回的元素指针。融合消息继续使用
`BuildMergedCustomMsg()` / `BuildMergedPointCloud2()` 返回的 `std::unique_ptr` 发布，publish
后不再访问该指针。

这是用户基于现有固定容量实现，对原提示词中额外队列上限和孤帧清理要求的明确覆盖。

## 空指针、异常与线程退出

`DriverNode` 增加 `std::atomic_bool stop_requested_{false}`，析构顺序为：

1. 设置 stop 标志。
2. 若 `lddc_ptr_` 及其 `lds_` 有效，请求数据源退出。
3. 安全满足 `exit_signal_`，忽略已满足 promise 的 `std::future_error`。
4. 仅对存在且 `joinable()` 的两个线程执行 `join()`。
5. join 完成后释放 `lddc_ptr_`。

构造期间只有 driver、数据源与配置均初始化成功后才启动 poll thread。数据源初始化失败
抛出明确异常，不留下半初始化后台线程。

两个 poll thread 在初始等待后和每次循环前检查 `rclcpp::ok()`、stop 标志及
`lddc_ptr_`。分发异常由线程入口捕获并节流记录，随后继续或在 stop/空对象时退出，避免
异常越过线程入口触发 `std::terminate`。

`Lddc` 的 publisher、node、merger 与队列使用点继续做局部判空；配置阶段要求 `cur_node_`
已经设置。普通单雷达路径不创建 merge publisher，publisher 仍按首次实际发布惰性创建。

## Launch 设计

### 通用 launch

修改：

`src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`

保留原有参数：

- `driver_params_file`
- `pointlio_params_file`
- `use_intra_process`
- `container_name`
- `log_level`

新增 `pointlio_imu_topic`，默认保持当前双雷达接口
`livox/imu_192_168_1_135`。在 Point-LIO YAML 后追加：

```python
{"common.imu_topic": pointlio_imu_topic}
```

两个 component 仍位于同一个 `component_container_mt`，并继续对两者设置
`use_intra_process_comms`。

### 单雷达 wrapper

新增：

`src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py`

传入：

- `single_MID360_component.yaml`
- 现有 `point_lio/config/mid360.yaml`
- `pointlio_imu_topic=livox/imu`
- `use_intra_process=true`
- `container_name=single_livox_pointlio_container`

启动命令：

```bash
ros2 launch point_lio single_livox_pointlio_intra_process.launch.py
```

### 双雷达 wrapper

新增：

`src/perception/Point-LIO/launch/mixed_livox_pointlio_intra_process.launch.py`

传入：

- `mixed_MID360_component.yaml`
- 现有 `point_lio/config/mid360.yaml`
- `pointlio_imu_topic=livox/imu_192_168_1_135`
- `use_intra_process=true`
- `container_name=mixed_livox_pointlio_container`

启动命令：

```bash
ros2 launch point_lio mixed_livox_pointlio_intra_process.launch.py
```

## 参数文件

新增 `src/perception/livox_ros_driver2/config/single_MID360_component.yaml`，以现有双雷达
component YAML 为格式参考，关键值为：

```yaml
multi_topic: 0
user_config_path: "$(find-pkg-share livox_ros_driver2)/config/MID360_config.json"
enable_internal_lidar_merge: false
enable_merge_debug: false
```

兼容性保留的 `merge_*` 参数在 effective merge 为 false 时必须完全忽略。单雷达 IP 继续
由现有 `MID360_config.json` 决定。

更新并核对 `mixed_MID360_component.yaml`：

```yaml
multi_topic: 1
enable_internal_lidar_merge: true
merge_front_ip: 192.168.1.135
merge_back_ip: 192.168.1.122
merge_output_topic: livox/lidar
merge_frame_id: livox_frame
```

front/back IP 必须继续存在于 `mixed_MID360_config.json`。不调整现有外参、频率或 frame。

## 数据流与行为保持

单雷达：

```text
MID360 -> DriverNode -> livox/lidar -> Point-LIO
                    -> livox/imu   -> Point-LIO
```

双雷达：

```text
front/back lidar -> driver 内部同步与 back-to-front 变换 -> livox/lidar -> Point-LIO
front IMU --------------------------------> livox/imu_192_168_1_135 -> Point-LIO
back IMU  --------------------------------> livox/imu_192_168_1_122
```

保持不变：

- Point-LIO 点云始终订阅 `livox/lidar`。
- 双雷达原始点云和两路 IMU topic 保留。
- Point-LIO 不同时订阅两路 IMU，也不订阅两路未融合点云。
- `/aft_mapped_to_init`、`/cloud_registered`、`/cloud_registered_full` 逻辑不改。
- 点云融合外参、消息字段、时间基准与 unique_ptr 发布策略不改。

有意改变：

- Point-LIO IMU topic 由单/双 wrapper 按实际接口覆盖。
- `multi_topic=0 && enable_internal_lidar_merge=true` 不再进入融合，而是 warning 后直出。
- 无效融合参数在启动阶段失败。
- component unload 与初始化失败不再盲目解引用或 join 空对象。

## 测试与验收

恢复历史 `internal_lidar_merger_test` 测试接入，并在实现前添加失败用例，覆盖：

- 传输格式支持判断。
- 时间窗匹配和较旧侧选择。
- CustomMsg / PointCloud2 合并、坐标变换和相对时间。
- effective merge 四组合和参数校验尽可能提取为无 ROS 运行时依赖的纯逻辑测试。

launch 与配置执行静态检查：

- Python AST 解析三个 launch 文件。
- YAML 与 JSON 语法解析。
- 核对单/双 wrapper 参数、两个 component、container 和 intra-process 设置。
- 核对单/双 YAML、IP、输出 topic 和 frame。
- `rg` 检查 merge 开关、publisher、IMU topic、线程和 Point-LIO 源码修改边界。
- `git diff --check` 与范围审计。

根据 AGENTS.md，未经用户明确许可不运行 `colcon build`、CMake、编译型 gtest、ASan、
ROS launch 或真机测试。因此本次默认只做静态验收；需要构建和真机验收时另行申请。

真机验收分别检查：

- 单雷达 `livox/lidar`、`livox/imu`、odom 和完整点云输出。
- 双雷达融合点数、front IMU、两路原始 IMU、odom 和完整点云输出。
- 断开任一路雷达后无崩溃、缓存不增长、warning 节流、Ctrl-C 正常退出。
- 同一 container 中存在 `livox_driver_node` 与 `laserMapping`。

## 改造记录

实施时创建：

`docs/ai_refactor_records/20260705_driver_pointlio_single_dual_adaptation.md`

记录 Explorer 事实、修改边界、文件变化、行为保持、有意调整、静态检查、未执行构建原因
及最终 `PASS` / `NEEDS_FIX`。

## 风险

- 用户选择的 IMU topic 分模式策略不再提供原提示词要求的统一 Point-LIO IMU 接口；未来
  若改回统一接口，应在 driver 中增加 front IMU alias，并删除 launch 层 IMU 切换。
- 现有队列满时丢弃新帧，单侧雷达恢复后可能先清理最多约 32 帧陈旧数据；本次按用户要求
  保持该比赛验证逻辑，不增加底层队列改造。
- 真机、断线、component unload 和内存安全无法仅凭静态检查证明，未获构建与硬件验收许可
  前最终结果必须明确标注这些验证缺口。
