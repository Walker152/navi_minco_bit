# Point-LIO Livox Intra-Process 改造结果

## Scope

- Source task: `src/perception/Point-LIO/task.md`
- Result file: `src/perception/Point-LIO/result.md`
- Current conclusion limit: 本机未连接 Livox 雷达，且用户要求后续构建手动运行；因此这里不声称已经验证实机 10Hz。
- Truth source for performance: `docs/livox_pointlio_intra_process_操作流程.md` 中的实机 A/B/C/D 测试。

## Completed Deliverables

### 1. Minimal C++ subscriber

Implemented:

- `src/perception/Point-LIO/src/min_lidar_subscriber.cpp`
- executable: `min_lidar_subscriber`
- supports `PointCloud2` and Livox `CustomMsg`
- supports parameters:
  - `lid_topic`
  - `msg_type`
  - `qos_mode`
  - `qos_depth`
  - `print_period`
- prints callback Hz, count, message size, stamp/timebase interval, wall interval, and delay where safe.
- does not run PCL conversion, publish messages, or save files.

### 2. Point-LIO component

Implemented in `src/perception/Point-LIO/src/laserMapping.cpp`:

- `point_lio::LaserMappingNode : public rclcpp::Node`
- component registration:
  - `RCLCPP_COMPONENTS_REGISTER_NODE(point_lio::LaserMappingNode)`
- CMake registration:
  - `rclcpp_components_register_nodes(point_lio_component "point_lio::LaserMappingNode")`
- original executable retained:
  - `pointlio_mapping`
  - compiled from the same source with `POINT_LIO_BUILD_MAIN`

### 3. Executor / main-loop decoupling

Implemented:

- component no longer creates its own executor.
- `laserMapping.cpp` no longer calls `executor.spin_some()`.
- regular executable creates a `MultiThreadedExecutor` and spins the node externally.
- LIO main processing moved to `LaserMappingNode::processingLoop()` worker thread.
- `stopWorker()` stops the thread, notifies `sig_buffer`, and joins.

Runtime rate logging now includes:

```text
cloud_cb, sync_in, odom_pub, pose_update, avg_process, max_process
```

### 4. Callback and buffer handling

Implemented:

- Point-LIO point cloud subscriptions use `UniquePtr` callbacks.
- `standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr)` and `livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr)` adapt ownership without copying the full point cloud.
- Callback-produced LiDAR/IMU data is written into pending queues under `mtx_buffer`.
- `sync_packages()` moves pending data into worker-owned queues before sync.

Note: the existing Point-LIO preprocessing is still called by the original point cloud callbacks. The LIO map update / Kalman / publish path is not run in subscriber callbacks.

### 5. Livox driver publish path

Checked:

- internal merge path already returns `std::unique_ptr<CustomMsg>` / `std::unique_ptr<PointCloud2>`.
- internal merge path already publishes with `publisher_ptr->publish(std::move(...))`.

Changed for the ROS2-only ordinary non-merge path in `src/perception/livox_ros_driver2/src/lddc.cpp` / `.h`:

- `PointCloud2` ordinary publish uses `std::unique_ptr<PointCloud2>` and `publish(std::move(cloud))`.
- `CustomMsg` ordinary publish uses `std::unique_ptr<CustomMsg>` and `publish(std::move(livox_msg))`.
- `PointCloud2` data is written directly into `cloud.data` per point, avoiding a temporary full-frame `std::vector<LivoxPointXyzrtlt>`.
- Per user confirmation, this optimized ordinary publish path no longer keeps a ROS1 const-ref overload. This repository target is treated as ROS2-only for `livox_ros_driver2`.

### 6. Launch files

Added:

- `src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py`
  - uses `component_container_mt`
  - loads `livox_ros::DriverNode`
  - loads `point_lio::LaserMappingNode`
  - passes `use_intra_process_comms`
  - supports `driver_params_file`, `pointlio_params_file`, `use_intra_process`, `container_name`, `log_level`
  - defaults `driver_params_file` to the dual-lidar merge parameter file below

- `src/perception/Point-LIO/launch/min_lidar_subscriber.launch.py`
  - optional cross-process diagnostic subscriber launch
  - not mixed into the intra-process container

Added Livox driver component parameter yaml:

- `src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml`
  - copied from `livox_ros_driver2/launch_ROS2/msg_mixed_MID360.launch.py`
  - keeps `xfer_format=1`, `multi_topic=1`, `publish_freq=10.0`
  - enables `enable_internal_lidar_merge`
  - keeps front/back IP, output topic, frame id, max interval, and back-to-front extrinsic
  - uses `$(find-pkg-share livox_ros_driver2)/config/mixed_MID360_config.json` for portable JSON config lookup

### 7. Docs

Added:

- `src/perception/Point-LIO/docs/livox_pointlio_intra_process_操作流程.md`
- `src/perception/Point-LIO/docs/livox_pointlio_rate_diagnosis_template.md`

These are installed through Point-LIO CMake along with `config`, `launch`, and `rviz_cfg`.

## Verification Performed

Static / non-build checks:

```bash
python3 -m py_compile \
  src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py \
  src/perception/Point-LIO/launch/min_lidar_subscriber.launch.py
```

Result: passed.

Launch generation check with `ROS_LOG_DIR=/tmp/pointlio_launch_check`:

```text
livox_pointlio_intra_process.launch.py LaunchDescription 6
min_lidar_subscriber.launch.py LaunchDescription 6
```

Result: passed.

Static source checks:

- no `spin_some` remains in `src/perception/Point-LIO/src/laserMapping.cpp`.
- no `std::make_shared<rclcpp::Node>("laserMapping")` remains in `laserMapping.cpp`.
- component registration and CMake registration are present.
- docs and launch artifacts are present.

## Build Status

The required command was attempted before the user asked for builds to be run manually:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

- did not complete.
- failure was from existing symlink-install build artifacts, not a C++ compile error from this change.
- first blocker:
  - `build/ros_interfaces/ament_cmake_python/ros_interfaces/ros_interfaces` existed as a directory, but colcon needed to create a symbolic link there.
- target-package retry hit the same type of blocker in `build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2`.

After the user request "要构建都让我手动运行", no more build commands should be run by Codex.

Manual build command for the user:

```bash
cd <workspace>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If the same symlink artifact blocker appears, clean or move the affected build artifacts manually, then rerun the build.

## Performance Status

Not claimed:

- no real Livox radar was connected here.
- no real driver 10Hz validation was performed here.
- no claim is made that Point-LIO now receives 10Hz on the robot.

Required real validation:

- follow `docs/livox_pointlio_intra_process_操作流程.md`.
- record results in `docs/livox_pointlio_rate_diagnosis_template.md`.
