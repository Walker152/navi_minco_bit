# Livox ROS2 Driver 内部双 MID360 融合说明

## 目标

当前改动把前后两台 MID360 的点云融合逻辑下沉到 `livox_ros_driver2` 内部，替代外部 `lidar_merger` 节点的订阅、坐标变换和二次发布流程。

融合后的输出仍使用 `livox/lidar`，Point-LIO 可以继续按原有配置订阅点云；IMU 仍保持每台雷达独立话题，例如 `livox/imu_192_168_1_135`。

## 雷达与坐标系

- 前雷达 IP：`192.168.1.135`
- 后雷达 IP：`192.168.1.122`
- 融合输出坐标系：以前雷达坐标系为基准
- 输出 `frame_id`：`livox_frame`
- 后雷达到前雷达外参：`[0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]`

外参含义为：

```text
[x, y, z, roll, pitch, yaw]
```

- `x/y/z` 单位是米
- `roll/pitch/yaw` 单位是弧度
- 表示 back lidar 到 front lidar 的刚体变换

## 主要参数

参数在 `launch_ROS2/msg_mixed_MID360.launch.py` 中配置：

```python
enable_internal_lidar_merge = True
merge_front_ip = '192.168.1.135'
merge_back_ip = '192.168.1.122'
merge_output_topic = 'livox/lidar'
merge_frame_id = 'livox_frame'
merge_max_interval_ms = 5.0
merge_extrinsic_back_to_front = [0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]
```

默认 launch 使用：

```python
xfer_format = 1
multi_topic = 1
```

这样点云由 driver 内部融合到 `livox/lidar`，IMU 仍按多话题方式发布。

## 数据流

启用 `enable_internal_lidar_merge` 后，`Lddc::DistributePointCloudData()` 不再逐雷达发布点云，而是进入内部融合路径：

1. 按 IP handle 找到 front/back 两个 `LidarDataQueue`
2. 使用 `QueuePeek()` 查看两个队头包，避免融合前深拷贝
3. 比较两帧 `base_time`
4. 时间差小于等于 `merge_max_interval_ms` 时合并发布
5. 时间差超限时丢弃更旧的一帧，并节流打印 warning
6. 发布成功后用 `QueuePopUpdate()` 同时弹出两帧

如果任一队列为空，不发布半帧，等待下一轮点云。

## 输出格式

内部融合支持 ROS2 下两个输出格式：

### `xfer_format = 1`

发布 `livox_ros_driver2/msg/CustomMsg`。

这是 Point-LIO 当前使用路径。合并规则：

- `header.stamp = min(front.base_time, back.base_time)`
- `header.frame_id = merge_frame_id`
- `timebase = min(front.base_time, back.base_time)`
- `point_num = front.points_num + back.points_num`
- 前雷达点直接写入
- 后雷达点先做 back->front 变换再写入
- 每点 `offset_time` 重新计算为相对合并帧 `timebase` 的偏移

### `xfer_format = 0`

发布 `sensor_msgs/msg/PointCloud2`。

字段保持 driver 原有结构：

```text
x, y, z, intensity, tag, line, timestamp
```

其中 `timestamp` 写入每点相对合并帧 `timebase` 的时间偏移。

### `xfer_format = 2`

ROS2 原 driver 已不支持 `pcl::PointCloud` 发布路径。启用内部融合时，如果配置成 `xfer_format = 2`，节点会直接参数校验失败，避免静默运行到错误状态。

## 坐标变换实现

内部融合新增 `InternalLidarMerger`：

- 参数加载时预计算一次旋转矩阵和平移
- 逐点融合时只做标量乘加
- 不在循环里创建 Eigen 对象
- 不重复计算三角函数
- 不引入 Ceres

当前每个后雷达点的变换为：

```text
p_front = R_back_to_front * p_back + t_back_to_front
```

前雷达点不做变换，直接进入输出消息。

暂时没有使用 Eigen/Ceres 批量矩阵乘，原因是当前点云存储是 AoS 结构：

```text
x, y, z, intensity, tag, line, offset_time
```

若要转成 Eigen 的 `3 x N` 连续矩阵，需要先 gather 到连续矩阵，乘完再 scatter 回 ROS 消息。这个额外内存搬运通常比当前逐点标量乘加更贵。Ceres 是优化库，不适合固定外参的点云坐标变换。

## 性能策略

当前优化重点是减少节点间传输和消息复制，而不是过早引入并行化：

- driver 内部直接融合，省掉外部 `lidar_merger` 节点订阅/发布
- `QueuePeek()` 避免融合前整包深拷贝
- 外参矩阵预计算
- 点云输出一次性 `reserve` / `resize`
- `CustomMsg` 使用 `std::unique_ptr` 发布
- 单帧融合耗时超过 20ms 时节流 warning

如果真机持续出现 merge duration warning，再考虑进一步优化：

1. 将后雷达点变换内联到消息填充循环
2. `CustomMsg.points.resize(total)` 后按下标写入
3. 对后雷达点变换加 OpenMP 分块并行
4. 再评估 SIMD 或 SoA 存储改造

## 涉及文件

- `src/internal_lidar_merger.h`
- `src/internal_lidar_merger.cpp`
- `src/lddc.h`
- `src/lddc.cpp`
- `src/livox_ros_driver2.cpp`
- `src/comm/ldq.h`
- `src/comm/ldq.cpp`
- `launch_ROS2/msg_mixed_MID360.launch.py`
- `CMakeLists.txt`
- `package.xml`
- `test/internal_lidar_merger_test.cpp`

仓库根目录的 `start.bash` 已注释外部 `lidar_merger` 启动。

## 构建与测试

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
```

带测试构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DBUILD_TESTING=ON
```

运行内部融合单元测试：

```bash
build/livox_ros_driver2/internal_lidar_merger_test
```

当前单元测试覆盖：

- back->front 外参平移变换
- roll/pitch/yaw 矩阵变换
- `CustomMsg` 合并后的 `timebase/header/point_num/offset_time`
- `PointCloud2` 合并后的字段、点数、timestamp
- 超过同步时间窗时的丢弃方向
- 只支持 `xfer_format = 0/1`

注意：完整 `colcon test --packages-select livox_ros_driver2` 目前仍会因为包内既有 lint 问题失败，主要来自 `3rdparty/rapidjson`、旧 launch 文件、CMake/package.xml 风格检查；这不是内部融合 gtest 或编译失败。

## 真机验收

启动 driver：

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_mixed_MID360.launch.py
```

不要再启动：

```bash
ros2 launch lidar_merger dual_lidar_merger.launch.py
```

检查点云频率：

```bash
ros2 topic hz /livox/lidar
```

期望接近 10Hz。

检查消息内容：

```bash
ros2 topic echo /livox/lidar --once
```

期望：

- `header.frame_id = livox_frame`
- `point_num` 接近两台雷达点数之和
- 不持续出现 `Internal lidar merge dropped stale` warning
- 不持续出现 `Internal lidar merge took ... ms` warning

Point-LIO 仍按原配置订阅 `livox/lidar`。
