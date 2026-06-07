# Livox + Point-LIO 同进程验证操作流程

## 1. 改造目的

当前问题是 Livox driver 端发布点云稳定 10Hz，但跨进程订阅和 Point-LIO 内部点云接收频率会在 2~7Hz 跳变。driver 内部双雷达 merge 耗时约 1ms，不是主要瓶颈；更可疑的是大点云跨进程 DDS 传输、QoS、订阅端调度，以及 Point-LIO 原先在 LIO 主循环里调用 `spin_some()` 导致 callback 与主计算耦合。

本次改造提供两个验证路径：

- 最小 C++ subscriber：只订阅、计数、打印消息尺寸和时间间隔，用于定位跨进程订阅本身是否能收到 10Hz。
- component + intra-process：把 `livox_ros::DriverNode` 和 `point_lio::LaserMappingNode` 放进同一个 `component_container_mt`，开启 `use_intra_process_comms`，减少大点云跨进程拷贝，并让 Point-LIO callback 由外部 executor 持续处理、LIO 主计算在 worker thread 中运行。

这里不声称已经验证实机 10Hz。真实性能结果必须以实机按本文步骤记录为准。

## 2. 编译步骤

```bash
cd <workspace>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果只想重编感知相关包，可在实机确认依赖完整后执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select livox_ros_driver2 point_lio
source install/setup.bash
```

## 3. 参数检查

检查 driver 输出 topic 与 Point-LIO `common.lid_topic` 是否一致。当前 Point-LIO `config/mid360.yaml` 里默认是 `livox/lidar`，如果 driver 实际发布 `/livox/lidar`，两者通常可匹配；如果带 namespace，请统一后再测。

检查 driver 参数：

- `xfer_format`: `0` 为 `PointCloud2`，`1` 为 Livox `CustomMsg`。
- `publish_freq`: 应为 `10.0`。
- `user_config_path`: 指向实机 Livox JSON 配置。
- 是否启用 internal lidar merge：若用双雷达合并，确认 driver 配置和日志。

检查 Point-LIO 参数：

- `preprocess.lidar_type`: Livox 使用 `1`。
- `common.lid_topic`: 必须和 driver 点云 topic 一致。
- `common.imu_topic`: 必须和 driver IMU topic 一致。
- `common.print_cloud_input_fps`: 实机诊断时建议临时设为 `True`。
- `publish.scan_publish_en`: 诊断接收频率时建议临时设为 `False`，减少大点云输出压力。
- `pcd_save.pcd_save_en`: 诊断实时性时建议临时设为 `False`。

component launch 的 `driver_params_file` 必须是 ROS2 参数 yaml。若原来只使用 `msg_MID360_launch.py` 的 Python 内联参数，可在实机新建类似：

```yaml
/**:
  ros__parameters:
    xfer_format: 1
    multi_topic: 0
    data_src: 0
    publish_freq: 10.0
    output_data_type: 0
    frame_id: livox_frame
    lvx_file_path: /home/livox/livox_test.lvx
    user_config_path: <workspace>/src/perception/livox_ros_driver2/config/MID360_config.json
    cmdline_input_bd_code: 47MDLC20020096
```

## 4. 测试 A：只测 driver + 最小 C++ subscriber

先启动原 driver：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

如果 driver 发布 `PointCloud2`：

```bash
ros2 run point_lio min_lidar_subscriber --ros-args \
  -p lid_topic:=/livox/lidar \
  -p msg_type:=pointcloud2 \
  -p qos_mode:=sensor \
  -p qos_depth:=5 \
  -p print_period:=1.0
```

如果 driver 发布 Livox `CustomMsg`：

```bash
ros2 run point_lio min_lidar_subscriber --ros-args \
  -p lid_topic:=/livox/lidar \
  -p msg_type:=custom \
  -p qos_mode:=sensor \
  -p qos_depth:=5 \
  -p print_period:=1.0
```

可临时用 reliable 定位 QoS 问题，但它可能带来延迟堆积，不一定适合最终实时导航：

```bash
ros2 run point_lio min_lidar_subscriber --ros-args \
  -p lid_topic:=/livox/lidar \
  -p msg_type:=custom \
  -p qos_mode:=reliable \
  -p qos_depth:=10
```

记录：

| 项目 | 结果 |
|---|---|
| driver 日志发布 Hz | |
| min_sub 接收 Hz | |
| ros2 topic hz | |
| CPU 占用 | |
| 结论 | |

## 5. 测试 B：driver + 原跨进程 Point-LIO

启动原 driver：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

启动原 Point-LIO executable：

```bash
ros2 launch point_lio point_lio.launch.py point_lio_cfg_dir:=<pointlio_yaml> rviz:=False
```

必要时另起最小 subscriber 对照，但它仍是外部跨进程订阅，会增加系统压力：

```bash
ros2 run point_lio min_lidar_subscriber --ros-args -p lid_topic:=/livox/lidar -p msg_type:=custom
```

记录：

| 项目 | 结果 |
|---|---|
| Point-LIO cloud_cb Hz | |
| Point-LIO sync_in Hz | |
| Point-LIO avg/max process_ms | |
| 外部 min_sub Hz | |
| 结论 | |

## 6. 测试 C：component intra-process

启动 component，同一 container 内加载 driver 和 Point-LIO：

```bash
ros2 launch point_lio livox_pointlio_intra_process.launch.py \
  driver_params_file:=<driver_yaml> \
  pointlio_params_file:=<pointlio_yaml> \
  use_intra_process:=true
```

本测试不要启动 RViz、不要录包、不要同时启动 `ros2 topic hz` 或外部 subscriber。只看 Point-LIO 内部日志：

```text
[Point-LIO] rates: cloud_cb=..., sync_in=..., odom_pub=..., pose_update=..., avg_process=..., max_process=...
```

记录：

| 项目 | 结果 |
|---|---|
| driver publish Hz | |
| Point-LIO cloud_cb Hz | |
| Point-LIO sync_in Hz | |
| Point-LIO avg/max process_ms | |
| 是否仍 2~7Hz 跳变 | |
| 结论 | |

## 7. 测试 D：component intra-process + 外部 subscriber 对照

先保持测试 C 的 component 正常运行，再额外启动：

```bash
ros2 run point_lio min_lidar_subscriber --ros-args \
  -p lid_topic:=/livox/lidar \
  -p msg_type:=custom \
  -p qos_mode:=sensor
```

注意：外部 min subscriber 仍是跨进程订阅，不代表 Point-LIO 内部同进程接收频率。它只用于比较“container 内部接收”和“外部 DDS 订阅”是否明显不同。

## 8. 判定表

| 现象 | 判断 |
|---|---|
| driver + min_sub 稳定 10Hz，ros2 topic hz 低 | `ros2 topic hz` 对大点云不可靠 |
| driver + min_sub 也只有 2~7Hz | 普通跨进程 DDS 链路接收不稳 |
| 只开 driver + min_sub 10Hz，开 Point-LIO 后 min_sub 掉 | Point-LIO 抢占 CPU、内存带宽或 DDS 线程 |
| component 后 Point-LIO cloud_cb 接近 10Hz | intra-process 改造有效 |
| component 后 cloud_cb 10Hz，但 sync_in 低 | 瓶颈转移到 Point-LIO 同步、IMU 或处理逻辑 |
| component 后 cloud_cb 仍 2~7Hz | 不是单纯跨进程问题，需要查 callback、锁、worker、CPU、时间戳和 QoS |

## 9. 常见问题

- topic 名不一致：driver 输出 topic 与 Point-LIO `common.lid_topic` 必须一致。
- `use_intra_process_comms` 没生效：确认启动的是 `livox_pointlio_intra_process.launch.py`，两个 node 在同一个 `component_container_mt` 内。
- 同时启动 RViz、rosbag 或 `ros2 topic hz`：这些都会引入额外订阅和序列化压力。
- reliable QoS：只用于定位问题，可能带来延迟堆积。
- worker thread 未启动：Point-LIO 应打印内部 rates；没有日志时检查 `common.print_cloud_input_fps`。
- component plugin 名写错：driver 是 `livox_ros::DriverNode`，Point-LIO 是 `point_lio::LaserMappingNode`。
- 参数文件未加载：确认 launch 命令里的 yaml 路径存在，且 `source install/setup.bash` 后再启动。
- CustomMsg / PointCloud2 类型不匹配：`xfer_format=1` 对应 `msg_type:=custom`；`xfer_format=0` 对应 `pointcloud2`。
- crash monitor 风险：Point-LIO 原有严重异常路径仍会 `exit(EXIT_FAILURE)`，component 模式下会退出整个 container。

## 10. 回滚方式

恢复原普通 launch：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch point_lio point_lio.launch.py point_lio_cfg_dir:=<pointlio_yaml> rviz:=False
```

只运行原 Point-LIO executable：

```bash
ros2 run point_lio pointlio_mapping --ros-args --params-file <pointlio_yaml>
```

关闭 component intra-process：

```bash
ros2 launch point_lio livox_pointlio_intra_process.launch.py \
  driver_params_file:=<driver_yaml> \
  pointlio_params_file:=<pointlio_yaml> \
  use_intra_process:=false
```
