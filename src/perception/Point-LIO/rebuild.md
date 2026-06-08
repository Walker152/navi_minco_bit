你现在需要在 Point-LIO / pointlio 代码中增加一套独立的测试打印，用于诊断高频 odom 发布和 pose update 统计是否真实，以及当前 EKF 更新是否退化为接近逐点更新。

背景：
当前运行日志中：

* odom_pub 约 260–280Hz
* pose_update 约 6800–7200Hz
  这可能说明 `pose_update` 统计的是 EKF 内部 LiDAR correction 调用频率，而不是 odom topic 发布频率。需要新增更细粒度的诊断打印，判断每次 pose update 对应多少点、每个 sync package 产生多少次 EKF update、odom 发布时间戳是否真实连续、是否存在 burst 发布。

要求：

1. 不要破坏现有算法逻辑。
2. 不要改变默认行为。
3. 所有新增诊断由一个新的 ROS2 参数单独控制，不要复用 `print_cloud_input_fps`。
4. 新参数建议命名：

   * `debug_pose_update_detail`
   * 默认值 false
5. 可选新增参数：

   * `debug_pose_update_detail_period`
   * 默认值 1.0 秒
6. 当 `debug_pose_update_detail=false` 时，不能有额外高频日志、不能明显增加运行开销。
7. 当 `debug_pose_update_detail=true` 时，每隔 `debug_pose_update_detail_period` 秒打印一次聚合统计，不要每次 update 都打印。

请实现以下诊断统计。

一、统计字段

在 `laserMapping.cpp` 中增加一个新的结构体，例如：

```cpp
struct PoseUpdateDebugStats
{
  std::mutex mutex;
  std::chrono::steady_clock::time_point window_start = std::chrono::steady_clock::now();

  uint64_t sync_count = 0;
  uint64_t pose_update_count = 0;
  uint64_t odom_pub_count = 0;

  uint64_t input_points_sum = 0;
  uint64_t downsample_points_sum = 0;
  uint64_t update_points_sum = 0;

  uint64_t min_points_per_update = std::numeric_limits<uint64_t>::max();
  uint64_t max_points_per_update = 0;

  uint64_t single_point_update_count = 0;
  uint64_t small_update_count = 0;      // points_per_update <= 5
  uint64_t medium_update_count = 0;     // 6 <= points_per_update <= 30
  uint64_t large_update_count = 0;      // points_per_update > 30

  double sensor_update_dt_sum_ms = 0.0;
  double sensor_update_dt_min_ms = std::numeric_limits<double>::infinity();
  double sensor_update_dt_max_ms = 0.0;
  uint64_t sensor_update_dt_count = 0;

  double odom_stamp_dt_sum_ms = 0.0;
  double odom_stamp_dt_min_ms = std::numeric_limits<double>::infinity();
  double odom_stamp_dt_max_ms = 0.0;
  uint64_t odom_stamp_dt_count = 0;

  double odom_wall_dt_sum_ms = 0.0;
  double odom_wall_dt_min_ms = std::numeric_limits<double>::infinity();
  double odom_wall_dt_max_ms = 0.0;
  uint64_t odom_wall_dt_count = 0;

  double last_sensor_update_time = -1.0;
  double last_odom_stamp_time = -1.0;
  std::chrono::steady_clock::time_point last_odom_wall_time;
  bool has_last_odom_wall_time = false;
};
```

可以根据实际代码风格简化，但必须保留这些核心指标：

* sync_count
* pose_update_count
* odom_pub_count
* input_points_sum
* downsample_points_sum
* update_points_sum
* min/max/avg points per update
* single/small/medium/large update count
* sensor time dt between pose updates
* odom header stamp dt
* odom wall time dt

二、参数读取

在 `parameters.cpp` / `parameters.h` 中增加全局参数：

```cpp
extern bool debug_pose_update_detail;
extern double debug_pose_update_detail_period;
```

默认值：

```cpp
bool debug_pose_update_detail = false;
double debug_pose_update_detail_period = 1.0;
```

在 `readParameters(rclcpp::Node & nh)` 中声明和读取：

```cpp
nh.declare_parameter<bool>("debug_pose_update_detail", false);
nh.get_parameter("debug_pose_update_detail", debug_pose_update_detail);

nh.declare_parameter<double>("debug_pose_update_detail_period", 1.0);
nh.get_parameter("debug_pose_update_detail_period", debug_pose_update_detail_period);

if (debug_pose_update_detail_period <= 0.05) {
  debug_pose_update_detail_period = 1.0;
}
```

如果当前工程参数命名集中在 `common.*` 或其他 namespace 下，请按现有风格放置，但要保证 yaml 可以这样控制：

```yaml
point_lio:
  ros__parameters:
    debug_pose_update_detail: true
    debug_pose_update_detail_period: 1.0
```

或者如果项目已有 `common` 分组，则使用：

```yaml
point_lio:
  ros__parameters:
    common.debug_pose_update_detail: true
    common.debug_pose_update_detail_period: 1.0
```

以当前代码实际参数命名风格为准。

三、在 sync package 入口记录

在每次成功获得一个 `MeasureGroup` 并进入主处理流程时，记录：

```cpp
sync_count++
input_points_sum += Measures.lidar ? Measures.lidar->size() : 0
```

如果当前代码在处理后有 `feats_undistort` 和 `feats_down_body`，则记录：

* `feats_undistort->size()` 作为 input/full undistorted points
* `feats_down_body->size()` 作为 downsample points

注意不要因为空指针导致崩溃。

四、在每次 EKF pose update 后记录 points_per_update

找到当前调用：

```cpp
kf_input.update_iterated_dyn_share_modified(...)
```

或：

```cpp
kf_output.update_iterated_dyn_share_modified(...)
```

成功后记录 `pose_update_count++`。

同时记录本次 update 使用的点数。

当前代码中通常有：

* `time_seq[k]`
* `idx`
* `feats_down_size`
* `time_current`

请根据实际循环逻辑计算本次更新点数：

* 如果 `time_seq[k]` 表示当前 batch 的结束索引，则本次点数应为 `time_seq[k] - previous_time_seq_end`
* 如果当前已有局部变量能表示本次参与 h_model 的点数，请直接使用那个变量
* 不要简单固定写 1
* 不要用整帧 `feats_down_size` 冒充每次 update 点数

记录：

```cpp
record_pose_update_debug(points_per_update, time_current);
```

其中 `time_current` 是滤波器当前 sensor time，单位秒。

该函数内部统计：

* update_points_sum += points_per_update
* min/max points_per_update
* single_point_update_count：points_per_update <= 1
* small_update_count：points_per_update <= 5
* medium_update_count：6 到 30
* large_update_count：大于 30
* sensor_update_dt：当前 time_current 与上一次 pose update 的 time_current 差值，单位 ms

五、在 odom 真正 publish 后记录 odom 时间

在 `publish_odometry()` 中，只有真正执行 publish 之后才记录 odom_pub_count。

记录：

* odom header stamp dt：当前 odom header stamp 与上一次 odom header stamp 的差，单位 ms
* odom wall dt：当前 steady_clock 时间与上一次真正 publish wall time 的差，单位 ms

不要在被限频 return 的路径上记录 odom_pub_count。

六、聚合打印格式

每隔 `debug_pose_update_detail_period` 秒打印一次，建议格式如下：

```text
[Point-LIO][PoseDebug] window=1.00s
  sync=10.0Hz, pose_update=7050.0Hz, odom_pub=275.0Hz
  updates_per_sync=705.0, odom_per_sync=27.5
  full_pts/sync_avg=19000.0, down_pts/sync_avg=8500.0
  pts/update avg=1.20 min=1 max=8
  update_bins single=6200 small=820 medium=30 large=0
  sensor_update_dt_ms avg=0.14 min=0.02 max=1.10
  odom_stamp_dt_ms avg=3.70 min=1.00 max=8.00
  odom_wall_dt_ms avg=3.65 min=1.05 max=7.80
```

必须打印以下推论提示：

* 如果 `pts/update avg <= 2` 或 `single_point_update_count / pose_update_count > 0.5`，打印 WARN：
  `Pose update is close to point-wise update; consider batching time_seq.`
* 如果 `pose_update_count / sync_count` 很大，例如大于 100，打印 WARN：
  `Many EKF updates per sync package; updates may be burst processed after each cloud frame.`
* 如果 `odom_pub_count` 明显小于 `pose_update_count`，打印 INFO：
  `Odom publish is rate-limited or decoupled from EKF update. Use odom_pub as external output rate.`

七、线程安全和性能要求

* 统计函数必须在 `debug_pose_update_detail` 为 false 时立刻 return。
* 聚合统计用 mutex 保护。
* 不允许在高频 update 中调用 RCLCPP_INFO。
* 高频路径只做简单整数累加和少量 double 运算。
* 不能影响原来的 `record_runtime_rate()` 统计。

八、验收标准

完成后，运行时如果参数关闭：

* 原有日志不变
* 没有 `[Point-LIO][PoseDebug]` 输出

参数开启后：

* 每秒输出一次 `[Point-LIO][PoseDebug]`
* 能看到 `pts/update avg`
* 能看到 `updates_per_sync`
* 能看到 `odom_stamp_dt_ms` 和 `odom_wall_dt_ms`
* 能判断当前 6800–7200Hz pose_update 是真正小 batch，还是接近逐点更新/帧后 burst 更新

九、不要做的事情

* 不要修改 EKF 算法
* 不要修改点云处理结果
* 不要修改 odom 发布逻辑
* 不要默认开启该功能
* 不要把 debug 输出塞进每次 update
* 不要把 `pose_update` 改名导致已有统计失效
