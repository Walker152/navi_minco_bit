# Dual LiDAR Calibration

Offline secondary-to-main extrinsic calibration for two synchronized Livox LiDARs and their
`sensor_msgs/msg/Imu` streams. The tool reads rosbag2 directly, estimates the rotation from IMU
vector alignment, rotation-deskews each scan, runs multi-frame GICP, rejects inconsistent frames,
and aggregates the accepted transforms on SE(3).

For a recording that only excites one rotation axis, an optional static bag can provide gyro bias
and gravity-plane constraints. In that mode gravity corrects roll/pitch relative to the configured
initial rotation, while its unobservable rotation about gravity keeps the initial value and is then
refined by GICP.

The output convention is always:

```text
p_main = R_secondary_to_main * p_secondary + t_secondary_to_main
```

## Record data

Record the two unmerged `livox_ros_driver2/msg/CustomMsg` topics and both IMU topics. Keep PTP/GPS
time synchronization enabled. Move the rigid sensor assembly with rotations around at least two
axes, while maintaining useful overlap on planes and non-parallel structures.

For calibration recording, set `enable_internal_lidar_merge: false` and keep `multi_topic: 1` with
CustomMsg output enabled. Otherwise the current driver consumes both per-device clouds internally
and only publishes the already merged `livox/lidar` topic, which cannot be used to solve a new
secondary-to-main transform.

## Configure

Copy `config/example.yaml` and set the four exact topic names. Obtain them with:

```bash
ros2 bag info /path/to/bag
```

Set `initial_extrinsic` to a measured or existing approximate secondary-to-main transform.

## Build and test

```bash
colcon build --packages-select dual_lidar_calibration --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select dual_lidar_calibration
colcon test-result --verbose
```

## Run

```bash
ros2 run dual_lidar_calibration calibrate \
  --bag /path/to/bag \
  --static-bag /path/to/static_bag \
  --config /path/to/calibration.yaml \
  --output /tmp/dual_lidar_result
```

`--static-bag` is optional when the motion bag already contains rotation around at least two axes.
The static bag only needs the two configured IMU topics; its LiDAR messages are not decoded.

Exit code `0` means all quality gates passed. Exit code `2` means calibration completed but the
data or result failed a quality gate. Exit code `1` means an input/configuration/runtime error.
During GICP, one progress line is printed per frame with elapsed time, estimated remaining time,
RMSE, overlap ratio, acceptance state, and rejection reason.

On success, the output directory contains:

- `extrinsic.yaml`: includes `merge_extrinsic_back_to_front` for the existing driver merger.
- `extrinsic_matrix.txt`: 4x4 secondary-to-main transform.
- `frame_results.csv`: per-frame convergence and quality evidence.
- `summary.yaml`: observability, bias, dispersion, and pass/fail summary.
- `aligned_preview.pcd`: main points in red and transformed secondary points in cyan.

On failure, only `summary.yaml` and `frame_results.csv` are written; an unsafe extrinsic is not
emitted.
