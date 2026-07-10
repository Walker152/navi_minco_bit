// #include <so3_math.h>
#include <cstdint>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "li_initialization.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

// surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
// Global map for visualization
PointCloudXYZI::Ptr global_map_ptr(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;

std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

int sleep_time = 0;

auto LOGGER = rclcpp::get_logger("laserMapping");

enum class RuntimeRateEvent
{
  CloudInput,
  SyncInput,
  OdomPublish,
  PoseUpdate,
};

struct RuntimeRateStats
{
  std::mutex mutex;
  std::chrono::steady_clock::time_point window_start = std::chrono::steady_clock::now();
  uint64_t cloud_input_count = 0;
  uint64_t sync_input_count = 0;
  uint64_t odom_publish_count = 0;
  uint64_t pose_update_count = 0;
  uint64_t process_count = 0;
  double process_sum_ms = 0.0;
  double process_max_ms = 0.0;
};

RuntimeRateStats runtime_rate_stats;

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
  uint64_t small_update_count = 0;
  uint64_t medium_update_count = 0;
  uint64_t large_update_count = 0;

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

  double full_undistort_time_sum_ms = 0.0;
  double full_undistort_time_min_ms = std::numeric_limits<double>::infinity();
  double full_undistort_time_max_ms = 0.0;
  uint64_t full_undistort_time_count = 0;

  double map_incremental_time_sum_ms = 0.0;
  double map_incremental_time_min_ms = std::numeric_limits<double>::infinity();
  double map_incremental_time_max_ms = 0.0;
  uint64_t map_incremental_time_count = 0;

  double last_sensor_update_time = -1.0;
  double last_odom_stamp_time = -1.0;
  std::chrono::steady_clock::time_point last_odom_wall_time;
  bool has_last_odom_wall_time = false;
};

PoseUpdateDebugStats pose_update_debug_stats;

struct FullCloudPoint
{
  PointType point;
  double stamp{0.0};
};

struct StateSnapshot
{
  double stamp{0.0};
  V3D pos{V3D::Zero()};
  M3D rot{M3D::Identity()};
  V3D vel{V3D::Zero()};
  V3D acc{V3D::Zero()};
  V3D omg{V3D::Zero()};
  M3D R_L_I{M3D::Identity()};
  V3D T_L_I{V3D::Zero()};
};

constexpr double kFullCloudMaxQueueAge = 0.20;
constexpr double kFullCloudStampEps = 1.0e-6;

bool fullCloudPointStampLess(const FullCloudPoint & a, const FullCloudPoint & b)
{
  return a.stamp < b.stamp;
}

std::deque<FullCloudPoint> full_cloud_queue;
bool has_full_cloud_anchor = false;
StateSnapshot full_cloud_anchor;
PointCloudXYZI::Ptr full_cloud_world(new PointCloudXYZI());
uint64_t full_cloud_dropped_points_this_scan = 0;
uint64_t last_full_cloud_enqueued_points = 0;
uint64_t last_full_cloud_out_of_order = 0;

double average_or_zero(double sum, uint64_t count)
{
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

double min_or_zero(double value, uint64_t count)
{
  return count > 0 ? value : 0.0;
}

void reset_pose_update_debug_window_locked(std::chrono::steady_clock::time_point now)
{
  pose_update_debug_stats.window_start = now;
  pose_update_debug_stats.sync_count = 0;
  pose_update_debug_stats.pose_update_count = 0;
  pose_update_debug_stats.odom_pub_count = 0;
  pose_update_debug_stats.input_points_sum = 0;
  pose_update_debug_stats.downsample_points_sum = 0;
  pose_update_debug_stats.update_points_sum = 0;
  pose_update_debug_stats.min_points_per_update = std::numeric_limits<uint64_t>::max();
  pose_update_debug_stats.max_points_per_update = 0;
  pose_update_debug_stats.single_point_update_count = 0;
  pose_update_debug_stats.small_update_count = 0;
  pose_update_debug_stats.medium_update_count = 0;
  pose_update_debug_stats.large_update_count = 0;
  pose_update_debug_stats.sensor_update_dt_sum_ms = 0.0;
  pose_update_debug_stats.sensor_update_dt_min_ms = std::numeric_limits<double>::infinity();
  pose_update_debug_stats.sensor_update_dt_max_ms = 0.0;
  pose_update_debug_stats.sensor_update_dt_count = 0;
  pose_update_debug_stats.odom_stamp_dt_sum_ms = 0.0;
  pose_update_debug_stats.odom_stamp_dt_min_ms = std::numeric_limits<double>::infinity();
  pose_update_debug_stats.odom_stamp_dt_max_ms = 0.0;
  pose_update_debug_stats.odom_stamp_dt_count = 0;
  pose_update_debug_stats.odom_wall_dt_sum_ms = 0.0;
  pose_update_debug_stats.odom_wall_dt_min_ms = std::numeric_limits<double>::infinity();
  pose_update_debug_stats.odom_wall_dt_max_ms = 0.0;
  pose_update_debug_stats.odom_wall_dt_count = 0;
  pose_update_debug_stats.full_undistort_time_sum_ms = 0.0;
  pose_update_debug_stats.full_undistort_time_min_ms = std::numeric_limits<double>::infinity();
  pose_update_debug_stats.full_undistort_time_max_ms = 0.0;
  pose_update_debug_stats.full_undistort_time_count = 0;
  pose_update_debug_stats.map_incremental_time_sum_ms = 0.0;
  pose_update_debug_stats.map_incremental_time_min_ms = std::numeric_limits<double>::infinity();
  pose_update_debug_stats.map_incremental_time_max_ms = 0.0;
  pose_update_debug_stats.map_incremental_time_count = 0;
}

void maybe_print_pose_update_debug_locked(std::chrono::steady_clock::time_point now)
{
  const double period = debug_pose_update_detail_period > 0.05 ? debug_pose_update_detail_period : 1.0;
  const double elapsed =
    std::chrono::duration_cast<std::chrono::duration<double>>(now - pose_update_debug_stats.window_start)
      .count();

  if (elapsed < period) {
    return;
  }

  const double sync_hz = static_cast<double>(pose_update_debug_stats.sync_count) / elapsed;
  const double pose_update_hz = static_cast<double>(pose_update_debug_stats.pose_update_count) / elapsed;
  const double odom_pub_hz = static_cast<double>(pose_update_debug_stats.odom_pub_count) / elapsed;
  const double updates_per_sync = pose_update_debug_stats.sync_count > 0
                                    ? static_cast<double>(pose_update_debug_stats.pose_update_count) /
                                        static_cast<double>(pose_update_debug_stats.sync_count)
                                    : 0.0;
  const double odom_per_sync = pose_update_debug_stats.sync_count > 0
                                 ? static_cast<double>(pose_update_debug_stats.odom_pub_count) /
                                     static_cast<double>(pose_update_debug_stats.sync_count)
                                 : 0.0;
  const double full_pts_per_sync = pose_update_debug_stats.sync_count > 0
                                     ? static_cast<double>(pose_update_debug_stats.input_points_sum) /
                                         static_cast<double>(pose_update_debug_stats.sync_count)
                                     : 0.0;
  const double down_pts_per_sync = pose_update_debug_stats.sync_count > 0
                                     ? static_cast<double>(pose_update_debug_stats.downsample_points_sum) /
                                         static_cast<double>(pose_update_debug_stats.sync_count)
                                     : 0.0;
  const double avg_points_per_update = pose_update_debug_stats.pose_update_count > 0
                                         ? static_cast<double>(pose_update_debug_stats.update_points_sum) /
                                             static_cast<double>(pose_update_debug_stats.pose_update_count)
                                         : 0.0;
  const uint64_t min_points_per_update =
    pose_update_debug_stats.pose_update_count > 0 ? pose_update_debug_stats.min_points_per_update : 0;

  const double avg_full_undistort_time_ms =
    average_or_zero(
      pose_update_debug_stats.full_undistort_time_sum_ms,
      pose_update_debug_stats.full_undistort_time_count);

  const double avg_map_incremental_time_ms =
    average_or_zero(
      pose_update_debug_stats.map_incremental_time_sum_ms,
      pose_update_debug_stats.map_incremental_time_count);

  RCLCPP_INFO(LOGGER,
    "[Point-LIO][PoseDebug] window=%.2fs\n"
    "  sync=%.1fHz, pose_update=%.1fHz, odom_pub=%.1fHz\n"
    "  updates_per_sync=%.1f, odom_per_sync=%.1f\n"
    "  full_pts/sync_avg=%.1f, down_pts/sync_avg=%.1f\n"
    "  pts/update avg=%.2f min=%llu max=%llu\n"
    "  update_bins single=%llu small=%llu medium=%llu large=%llu\n"
    "  full_undistort_time_ms avg=%.2f min=%.2f max=%.2f count=%llu\n"
    "  map_incremental_time_ms avg=%.2f min=%.2f max=%.2f count=%llu\n"
    "  sensor_update_dt_ms avg=%.2f min=%.2f max=%.2f\n"
    "  odom_stamp_dt_ms avg=%.2f min=%.2f max=%.2f\n"
    "  odom_wall_dt_ms avg=%.2f min=%.2f max=%.2f",
    elapsed,
    sync_hz,
    pose_update_hz,
    odom_pub_hz,
    updates_per_sync,
    odom_per_sync,
    full_pts_per_sync,
    down_pts_per_sync,
    avg_points_per_update,
    static_cast<unsigned long long>(min_points_per_update),
    static_cast<unsigned long long>(pose_update_debug_stats.max_points_per_update),
    static_cast<unsigned long long>(pose_update_debug_stats.single_point_update_count),
    static_cast<unsigned long long>(pose_update_debug_stats.small_update_count),
    static_cast<unsigned long long>(pose_update_debug_stats.medium_update_count),
    static_cast<unsigned long long>(pose_update_debug_stats.large_update_count),
    avg_full_undistort_time_ms,
    min_or_zero(
      pose_update_debug_stats.full_undistort_time_min_ms,
      pose_update_debug_stats.full_undistort_time_count),
    pose_update_debug_stats.full_undistort_time_max_ms,
    static_cast<unsigned long long>(pose_update_debug_stats.full_undistort_time_count),
    avg_map_incremental_time_ms,
    min_or_zero(
      pose_update_debug_stats.map_incremental_time_min_ms,
      pose_update_debug_stats.map_incremental_time_count),
    pose_update_debug_stats.map_incremental_time_max_ms,
    static_cast<unsigned long long>(pose_update_debug_stats.map_incremental_time_count),
    average_or_zero(
      pose_update_debug_stats.sensor_update_dt_sum_ms, pose_update_debug_stats.sensor_update_dt_count),
    min_or_zero(
      pose_update_debug_stats.sensor_update_dt_min_ms, pose_update_debug_stats.sensor_update_dt_count),
    pose_update_debug_stats.sensor_update_dt_max_ms,
    average_or_zero(
      pose_update_debug_stats.odom_stamp_dt_sum_ms, pose_update_debug_stats.odom_stamp_dt_count),
    min_or_zero(pose_update_debug_stats.odom_stamp_dt_min_ms, pose_update_debug_stats.odom_stamp_dt_count),
    pose_update_debug_stats.odom_stamp_dt_max_ms,
    average_or_zero(
      pose_update_debug_stats.odom_wall_dt_sum_ms, pose_update_debug_stats.odom_wall_dt_count),
    min_or_zero(pose_update_debug_stats.odom_wall_dt_min_ms, pose_update_debug_stats.odom_wall_dt_count),
    pose_update_debug_stats.odom_wall_dt_max_ms);

  if (pose_update_debug_stats.pose_update_count > 0) {
    const double single_ratio = static_cast<double>(pose_update_debug_stats.single_point_update_count) /
                                static_cast<double>(pose_update_debug_stats.pose_update_count);
    if (avg_points_per_update <= 2.0 || single_ratio > 0.5) {
      RCLCPP_WARN(LOGGER,
        "[Point-LIO][PoseDebug] Pose update is close to point-wise update; consider batching "
        "time_seq.");
    }
    if (updates_per_sync > 100.0) {
      RCLCPP_WARN(LOGGER,
        "[Point-LIO][PoseDebug] Many EKF updates per sync package; updates may be burst processed "
        "after each cloud frame.");
    }
    if (pose_update_debug_stats.odom_pub_count < pose_update_debug_stats.pose_update_count) {
      RCLCPP_INFO(LOGGER,
        "[Point-LIO][PoseDebug] Odom publish is rate-limited or decoupled from EKF update. Use "
        "odom_pub as external output rate.");
    }
  }

  reset_pose_update_debug_window_locked(now);
}

void record_pose_update_debug_sync(uint64_t input_points)
{
  if (!debug_pose_update_detail) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  ++pose_update_debug_stats.sync_count;
  pose_update_debug_stats.input_points_sum += input_points;
  maybe_print_pose_update_debug_locked(now);
}

void record_pose_update_debug_downsample(uint64_t downsample_points)
{
  if (!debug_pose_update_detail) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  pose_update_debug_stats.downsample_points_sum += downsample_points;
  maybe_print_pose_update_debug_locked(now);
}

void record_pose_update_debug_update(uint64_t points_per_update, double sensor_time)
{
  if (!debug_pose_update_detail) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  ++pose_update_debug_stats.pose_update_count;
  pose_update_debug_stats.update_points_sum += points_per_update;
  pose_update_debug_stats.min_points_per_update =
    std::min(pose_update_debug_stats.min_points_per_update, points_per_update);
  pose_update_debug_stats.max_points_per_update =
    std::max(pose_update_debug_stats.max_points_per_update, points_per_update);

  if (points_per_update <= 1) {
    ++pose_update_debug_stats.single_point_update_count;
  }
  if (points_per_update <= 5) {
    ++pose_update_debug_stats.small_update_count;
  } else if (points_per_update <= 30) {
    ++pose_update_debug_stats.medium_update_count;
  } else {
    ++pose_update_debug_stats.large_update_count;
  }

  if (pose_update_debug_stats.last_sensor_update_time >= 0.0) {
    const double dt_ms = (sensor_time - pose_update_debug_stats.last_sensor_update_time) * 1000.0;
    pose_update_debug_stats.sensor_update_dt_sum_ms += dt_ms;
    pose_update_debug_stats.sensor_update_dt_min_ms =
      std::min(pose_update_debug_stats.sensor_update_dt_min_ms, dt_ms);
    pose_update_debug_stats.sensor_update_dt_max_ms =
      std::max(pose_update_debug_stats.sensor_update_dt_max_ms, dt_ms);
    ++pose_update_debug_stats.sensor_update_dt_count;
  }
  pose_update_debug_stats.last_sensor_update_time = sensor_time;
  maybe_print_pose_update_debug_locked(now);
}

void record_pose_update_debug_odom(const builtin_interfaces::msg::Time & stamp)
{
  if (!debug_pose_update_detail) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double stamp_time = rclcpp::Time(stamp).seconds();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  ++pose_update_debug_stats.odom_pub_count;

  if (pose_update_debug_stats.last_odom_stamp_time >= 0.0) {
    const double dt_ms = (stamp_time - pose_update_debug_stats.last_odom_stamp_time) * 1000.0;
    pose_update_debug_stats.odom_stamp_dt_sum_ms += dt_ms;
    pose_update_debug_stats.odom_stamp_dt_min_ms =
      std::min(pose_update_debug_stats.odom_stamp_dt_min_ms, dt_ms);
    pose_update_debug_stats.odom_stamp_dt_max_ms =
      std::max(pose_update_debug_stats.odom_stamp_dt_max_ms, dt_ms);
    ++pose_update_debug_stats.odom_stamp_dt_count;
  }
  pose_update_debug_stats.last_odom_stamp_time = stamp_time;

  if (pose_update_debug_stats.has_last_odom_wall_time) {
    const double dt_ms =
      std::chrono::duration<double, std::milli>(now - pose_update_debug_stats.last_odom_wall_time).count();
    pose_update_debug_stats.odom_wall_dt_sum_ms += dt_ms;
    pose_update_debug_stats.odom_wall_dt_min_ms =
      std::min(pose_update_debug_stats.odom_wall_dt_min_ms, dt_ms);
    pose_update_debug_stats.odom_wall_dt_max_ms =
      std::max(pose_update_debug_stats.odom_wall_dt_max_ms, dt_ms);
    ++pose_update_debug_stats.odom_wall_dt_count;
  }
  pose_update_debug_stats.last_odom_wall_time = now;
  pose_update_debug_stats.has_last_odom_wall_time = true;
  maybe_print_pose_update_debug_locked(now);
}

void record_pose_update_debug_full_undistort(double time_ms)
{
  if (!debug_pose_update_detail || !std::isfinite(time_ms) || time_ms < 0.0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  pose_update_debug_stats.full_undistort_time_sum_ms += time_ms;
  pose_update_debug_stats.full_undistort_time_min_ms =
    std::min(pose_update_debug_stats.full_undistort_time_min_ms, time_ms);
  pose_update_debug_stats.full_undistort_time_max_ms =
    std::max(pose_update_debug_stats.full_undistort_time_max_ms, time_ms);
  ++pose_update_debug_stats.full_undistort_time_count;
  maybe_print_pose_update_debug_locked(now);
}

void record_pose_update_debug_map_incremental(double time_ms)
{
  if (!debug_pose_update_detail || !std::isfinite(time_ms) || time_ms < 0.0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(pose_update_debug_stats.mutex);
  pose_update_debug_stats.map_incremental_time_sum_ms += time_ms;
  pose_update_debug_stats.map_incremental_time_min_ms =
    std::min(pose_update_debug_stats.map_incremental_time_min_ms, time_ms);
  pose_update_debug_stats.map_incremental_time_max_ms =
    std::max(pose_update_debug_stats.map_incremental_time_max_ms, time_ms);
  ++pose_update_debug_stats.map_incremental_time_count;
  maybe_print_pose_update_debug_locked(now);
}

void record_runtime_rate(RuntimeRateEvent event)
{
  if (!print_cloud_input_fps) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(runtime_rate_stats.mutex);

  switch (event) {
  case RuntimeRateEvent::CloudInput:
    ++runtime_rate_stats.cloud_input_count;
    break;
  case RuntimeRateEvent::SyncInput:
    ++runtime_rate_stats.sync_input_count;
    break;
  case RuntimeRateEvent::OdomPublish:
    ++runtime_rate_stats.odom_publish_count;
    break;
  case RuntimeRateEvent::PoseUpdate:
    ++runtime_rate_stats.pose_update_count;
    break;
  }

  const double elapsed =
    std::chrono::duration_cast<std::chrono::duration<double>>(now - runtime_rate_stats.window_start)
      .count();
  const double print_period = cloud_input_fps_print_period > 0.0 ? cloud_input_fps_print_period : 1.0;
  if (elapsed < print_period) {
    return;
  }

  const double cloud_input_fps = static_cast<double>(runtime_rate_stats.cloud_input_count) / elapsed;
  const double sync_input_fps = static_cast<double>(runtime_rate_stats.sync_input_count) / elapsed;
  const double odom_publish_fps = static_cast<double>(runtime_rate_stats.odom_publish_count) / elapsed;
  const double pose_update_fps = static_cast<double>(runtime_rate_stats.pose_update_count) / elapsed;
  const double avg_process_ms =
    runtime_rate_stats.process_count > 0
      ? runtime_rate_stats.process_sum_ms / static_cast<double>(runtime_rate_stats.process_count)
      : 0.0;
  RCLCPP_INFO(LOGGER,
    "[Point-LIO] rates: cloud_cb=%.2fHz (%llu), sync_in=%.2fHz (%llu), odom_pub=%.2fHz (%llu), "
    "pose_update=%.2fHz (%llu), avg_process=%.2fms, max_process=%.2fms, window=%.2fs",
    cloud_input_fps,
    static_cast<unsigned long long>(runtime_rate_stats.cloud_input_count),
    sync_input_fps,
    static_cast<unsigned long long>(runtime_rate_stats.sync_input_count),
    odom_publish_fps,
    static_cast<unsigned long long>(runtime_rate_stats.odom_publish_count),
    pose_update_fps,
    static_cast<unsigned long long>(runtime_rate_stats.pose_update_count),
    avg_process_ms,
    runtime_rate_stats.process_max_ms,
    elapsed);

  runtime_rate_stats.cloud_input_count = 0;
  runtime_rate_stats.sync_input_count = 0;
  runtime_rate_stats.odom_publish_count = 0;
  runtime_rate_stats.pose_update_count = 0;
  runtime_rate_stats.process_count = 0;
  runtime_rate_stats.process_sum_ms = 0.0;
  runtime_rate_stats.process_max_ms = 0.0;
  runtime_rate_stats.window_start = now;
}

void record_runtime_process_time(double process_ms)
{
  if (!print_cloud_input_fps) {
    return;
  }

  std::lock_guard<std::mutex> lock(runtime_rate_stats.mutex);
  ++runtime_rate_stats.process_count;
  runtime_rate_stats.process_sum_ms += process_ms;
  runtime_rate_stats.process_max_ms = std::max(runtime_rate_stats.process_max_ms, process_ms);
}

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

// 读取 PCD 文件,返回点云指针
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string & file_path)
{
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(LOGGER, "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(LOGGER, "Loaded %zu points from %s", pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

inline void dump_lio_state_to_log(FILE * fp_)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp_, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp_, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp_, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // omega
    fprintf(fp_, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp_, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));     // Bias_g
    fprintf(fp_, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));     // Bias_a
    fprintf(fp_,
      "%lf %lf %lf ",
      kf_input.x_.gravity(0),
      kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(fp_, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // omega
    fprintf(fp_, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // Acc
    fprintf(fp_, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));     // Bias_g
    fprintf(fp_, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));     // Bias_a
    fprintf(fp_,
      "%lf %lf %lf ",
      kf_output.x_.gravity(0),
      kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp_, "\r\n");
  fflush(fp_);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

bool isFinitePoint(const PointType & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void enqueueFullCloud(const PointCloudXYZI::Ptr & cloud, double lidar_beg_time)
{
  last_full_cloud_enqueued_points = 0;
  last_full_cloud_out_of_order = 0;
  if (!cloud) {
    return;
  }

  std::vector<FullCloudPoint> batch;
  batch.reserve(cloud->points.size());

  double prev_stamp = -std::numeric_limits<double>::infinity();
  for (const auto & point : cloud->points) {
    if (!isFinitePoint(point)) {
      continue;
    }

    FullCloudPoint full_point;
    full_point.point = point;
    full_point.stamp = lidar_beg_time + static_cast<double>(point.curvature) / 1000.0;
    if (!std::isfinite(full_point.stamp)) {
      continue;
    }

    if (full_point.stamp < prev_stamp) {
      ++last_full_cloud_out_of_order;
    }
    prev_stamp = full_point.stamp;

    batch.emplace_back(std::move(full_point));
    ++last_full_cloud_enqueued_points;
  }

  if (batch.empty()) {
    return;
  }

  // Sort batch by stamp before appending to queue
  std::stable_sort(batch.begin(), batch.end(), fullCloudPointStampLess);

  // Append sorted batch to queue
  for (auto & fp : batch) {
    full_cloud_queue.push_back(std::move(fp));
  }

  // Ensure full queue is globally sorted (handles residual future points from
  // prior scans interleaved with current-scan points)
  std::stable_sort(full_cloud_queue.begin(), full_cloud_queue.end(), fullCloudPointStampLess);
}

StateSnapshot makeStateSnapshot(double stamp)
{
  StateSnapshot snapshot;
  snapshot.stamp = stamp;

  if (!use_imu_as_input) {
    snapshot.pos = kf_output.x_.pos;
    snapshot.rot = kf_output.x_.rot;
    snapshot.vel = kf_output.x_.vel;
    snapshot.acc = kf_output.x_.rot * kf_output.x_.acc + kf_output.x_.gravity;
    snapshot.omg = kf_output.x_.omg;
    if (extrinsic_est_en) {
      snapshot.R_L_I = kf_output.x_.offset_R_L_I;
      snapshot.T_L_I = kf_output.x_.offset_T_L_I;
    } else {
      snapshot.R_L_I = Lidar_R_wrt_IMU;
      snapshot.T_L_I = Lidar_T_wrt_IMU;
    }
  } else {
    snapshot.pos = kf_input.x_.pos;
    snapshot.rot = kf_input.x_.rot;
    snapshot.vel = kf_input.x_.vel;
    snapshot.acc = kf_input.x_.rot * (input_in.acc - kf_input.x_.ba) + kf_input.x_.gravity;
    snapshot.omg = input_in.gyro - kf_input.x_.bg;
    if (extrinsic_est_en) {
      snapshot.R_L_I = kf_input.x_.offset_R_L_I;
      snapshot.T_L_I = kf_input.x_.offset_T_L_I;
    } else {
      snapshot.R_L_I = Lidar_R_wrt_IMU;
      snapshot.T_L_I = Lidar_T_wrt_IMU;
    }
  }

  return snapshot;
}

PointType transformFullPointForward(const StateSnapshot & anchor, const FullCloudPoint & full_point)
{
  const double dt = full_point.stamp - anchor.stamp;
  const M3D R_pred = anchor.rot * Exp(anchor.omg, dt);
  const V3D p_pred = anchor.pos + anchor.vel * dt + 0.5 * anchor.acc * dt * dt;

  const V3D p_lidar(full_point.point.x, full_point.point.y, full_point.point.z);
  const V3D p_imu = anchor.R_L_I * p_lidar + anchor.T_L_I;
  const V3D p_world = R_pred * p_imu + p_pred;

  PointType output = full_point.point;
  output.x = p_world.x();
  output.y = p_world.y();
  output.z = p_world.z();
  return output;
}

void processFullCloudSegmentWithSnapshot(
  const StateSnapshot & current_snapshot)
{
  if (!std::isfinite(current_snapshot.stamp)) {
    return;
  }

  if (!has_full_cloud_anchor) {
    full_cloud_anchor = current_snapshot;
    has_full_cloud_anchor = true;
    return;
  }

  if (current_snapshot.stamp <= full_cloud_anchor.stamp + kFullCloudStampEps) {
    full_cloud_anchor = current_snapshot;
    return;
  }

  size_t dropped_old_points = 0;

  while (!full_cloud_queue.empty() &&
         current_snapshot.stamp - full_cloud_queue.front().stamp > kFullCloudMaxQueueAge) {
    full_cloud_queue.pop_front();
    ++dropped_old_points;
  }

  while (!full_cloud_queue.empty()) {
    const FullCloudPoint & full_point = full_cloud_queue.front();
    if (full_point.stamp + kFullCloudStampEps < full_cloud_anchor.stamp) {
      full_cloud_queue.pop_front();
      ++dropped_old_points;
      continue;
    }
    if (full_point.stamp > current_snapshot.stamp + kFullCloudStampEps) {
      break;
    }

    const PointType point_world = transformFullPointForward(full_cloud_anchor, full_point);
    if (isFinitePoint(point_world)) {
      full_cloud_world->push_back(point_world);
    }
    full_cloud_queue.pop_front();
  }

  full_cloud_dropped_points_this_scan += dropped_old_points;
  full_cloud_anchor = current_snapshot;
}

void publishAccumulatedFullCloudWorld(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
  double stamp)
{
  if (!pub || !full_cloud_world || full_cloud_world->empty()) {
    return;
  }

  full_cloud_world->width = static_cast<uint32_t>(full_cloud_world->size());
  full_cloud_world->height = 1;
  full_cloud_world->is_dense = false;

  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*full_cloud_world, *cloud_msg);
  cloud_msg->header.stamp = get_ros_time(stamp);
  cloud_msg->header.frame_id = "camera_init";
  pub->publish(std::move(cloud_msg));

  if (print_cloud_input_fps) {
    RCLCPP_INFO(LOGGER,
      "[Point-LIO][FullCloud] enqueued=%llu published=%zu queue_remaining=%zu "
      "dropped_old=%llu out_of_order=%llu",
      static_cast<unsigned long long>(last_full_cloud_enqueued_points),
      full_cloud_world->size(),
      full_cloud_queue.size(),
      static_cast<unsigned long long>(full_cloud_dropped_points_this_scan),
      static_cast<unsigned long long>(last_full_cloud_out_of_order));
  }

  full_cloud_world->clear();
  full_cloud_dropped_points_this_scan = 0;
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);

  if (pcd_save_en && !points_to_add.empty()) {
    global_map_ptr->points.insert(global_map_ptr->points.end(), points_to_add.begin(), points_to_add.end());
    global_map_ptr->width = static_cast<uint32_t>(global_map_ptr->points.size());
    global_map_ptr->height = 1;
    global_map_ptr->is_dense = true;
  }
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

void publish_accumulated_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
  if (!pcd_save_en || accumulated_map_publish_hz <= 0.0 || !global_map_ptr || global_map_ptr->empty()) {
    return;
  }

  static double last_publish_time = -std::numeric_limits<double>::infinity();
  const double publish_period = 1.0 / accumulated_map_publish_hz;
  if (lidar_end_time - last_publish_time < publish_period) {
    return;
  }
  last_publish_time = lidar_end_time;

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*global_map_ptr, map_msg);
  map_msg.header.stamp = get_ros_time(lidar_end_time);
  map_msg.header.frame_id = "camera_init";
  pubLaserCloudMap->publish(map_msg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(
          string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T> void set_posestamp(T & out)
{
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

  auto set_output_from_kf = [&](const auto & kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

// 发布里程计话题
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
  static auto last_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  if (current_time - last_time < std::chrono::milliseconds(5)) {
    return;
  }
  last_time = current_time;

  // // -------------------- Odom / Yaw 调试统计 --------------------
  // static bool yaw_initialized = false;
  // static double last_yaw = 0.0;
  // static auto same_yaw_start_time = current_time;
  // static auto last_yaw_change_time = current_time;
  // static auto stats_window_start_time = current_time;
  // static size_t odom_pub_count_in_window = 0;
  // static size_t yaw_update_count_in_window = 0;

  constexpr double kYawDiffEps = 1e-4;  // rad，小于该阈值认为 yaw 未变化

  auto normalize_angle = [](double a) {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  };
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  if (publish_odometry_without_downsample) {
    odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(odomAftMapped.pose.pose);

  tf2::Quaternion q_pose(odomAftMapped.pose.pose.orientation.x,
    odomAftMapped.pose.pose.orientation.y,
    odomAftMapped.pose.pose.orientation.z,
    odomAftMapped.pose.pose.orientation.w);
  double roll_pose = 0.0;
  double pitch_pose = 0.0;
  double yaw_pose = 0.0;
  tf2::Matrix3x3(q_pose).getRPY(roll_pose, pitch_pose, yaw_pose);

  // odom_pub_count_in_window++;
  // if (!yaw_initialized) {
  //   yaw_initialized = true;
  //   last_yaw = yaw_pose;
  //   same_yaw_start_time = current_time;
  //   last_yaw_change_time = current_time;
  // } else {
  //   const double yaw_diff = normalize_angle(yaw_pose - last_yaw);
  //   if (std::fabs(yaw_diff) <= kYawDiffEps) {
  //     // yaw 保持不变，持续时间统计由周期日志打印
  //   } else {
  //     // yaw 发生更新：打印上一段“yaw 不变”的持续时长
  //     const double same_yaw_duration =
  //       std::chrono::duration<double>(current_time - same_yaw_start_time).count();
  //     RCLCPP_INFO(
  //       LOGGER,
  //       "[ODOM DEBUG] Yaw unchanged duration: %.3f s, then updated by %.6f rad",
  //       same_yaw_duration,
  //       std::fabs(yaw_diff));

  //     yaw_update_count_in_window++;
  //     last_yaw_change_time = current_time;
  //     same_yaw_start_time = current_time;
  //     last_yaw = yaw_pose;
  //   }
  // }

  // const double window_dt =
  //   std::chrono::duration<double>(current_time - stats_window_start_time).count();
  // if (window_dt >= 1.0) {
  //   const double odom_pub_hz = static_cast<double>(odom_pub_count_in_window) / window_dt;
  //   const double yaw_update_hz = static_cast<double>(yaw_update_count_in_window) / window_dt;
  //   const double yaw_same_duration_now =
  //     std::chrono::duration<double>(current_time - same_yaw_start_time).count();
  //   const double since_last_yaw_update =
  //     std::chrono::duration<double>(current_time - last_yaw_change_time).count();

  //   RCLCPP_INFO(
  //     LOGGER,
  //     "[ODOM DEBUG] odom_pub: %.2f Hz | yaw_update: %.2f Hz | yaw_same_now: %.3f s | "
  //     "since_last_yaw_update: %.3f s | yaw: %.6f rad",
  //     odom_pub_hz,
  //     yaw_update_hz,
  //     yaw_same_duration_now,
  //     since_last_yaw_update,
  //     yaw_pose);

  //   stats_window_start_time = current_time;
  //   odom_pub_count_in_window = 0;
  //   yaw_update_count_in_window = 0;
  // }

  auto set_twist_linear_from_kf = [&](const auto & kf) {
    Eigen::Vector3d vel_world = kf.x_.vel;
    Eigen::Quaterniond q(kf.x_.rot);
    Eigen::Vector3d vel_body = q.inverse() * vel_world;

    odomAftMapped.twist.twist.linear.x = vel_body(0);
    odomAftMapped.twist.twist.linear.y = vel_body(1);
    odomAftMapped.twist.twist.linear.z = vel_body(2);
  };

  if (!use_imu_as_input) {
    set_twist_linear_from_kf(kf_output);
    odomAftMapped.twist.twist.angular.x = kf_output.x_.omg(0);
    odomAftMapped.twist.twist.angular.y = kf_output.x_.omg(1);
    odomAftMapped.twist.twist.angular.z = kf_output.x_.omg(2);
  } else {
    set_twist_linear_from_kf(kf_input);
    odomAftMapped.twist.twist.angular.x = angvel_avr(0);
    odomAftMapped.twist.twist.angular.y = angvel_avr(1);
    odomAftMapped.twist.twist.angular.z = angvel_avr(2);
  }

  pubOdomAftMapped->publish(odomAftMapped);
  record_runtime_rate(RuntimeRateEvent::OdomPublish);
  record_pose_update_debug_odom(odomAftMapped.header.stamp);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "body";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);

    geometry_msgs::msg::TransformStamped transform_inverse;
    transform_inverse.header.frame_id = "camera_init";
    transform_inverse.child_frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_pose);
    // tf2::Vector3 offset_vec(0.0, 0.15, 0.0);
    tf2::Vector3 offset_vec(0.0, 0.20, 0.0);  // 变形哨
    tf2::Vector3 base_pose =
      tf2::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z) +
      tf2::quatRotate(q, offset_vec);
    // tf2::Quaternion q(
    //   odomAftMapped.pose.pose.orientation.x,
    //   odomAftMapped.pose.pose.orientation.y,
    //   odomAftMapped.pose.pose.orientation.z,
    //   odomAftMapped.pose.pose.orientation.w);
    // tf2::Quaternion q_inv = q.inverse();

    // transform_inverse.transform.rotation.w = q_inv.w();
    // transform_inverse.transform.rotation.x = q_inv.x();
    // transform_inverse.transform.rotation.y = q_inv.y();
    // transform_inverse.transform.rotation.z = q_inv.z();
    transform_inverse.transform.translation.x = base_pose.x();
    transform_inverse.transform.translation.y = base_pose.y();
    transform_inverse.transform.translation.z = base_pose.z();
    transform_inverse.transform.rotation.w = 1;
    transform_inverse.transform.rotation.x = 0;
    transform_inverse.transform.rotation.y = 0;
    transform_inverse.transform.rotation.z = 0;
    transform_inverse.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform_inverse);

    geometry_msgs::msg::TransformStamped transform_slam_base;
    transform_slam_base.header.frame_id = "camera_init";
    transform_slam_base.child_frame_id = "slambase";
    transform_slam_base.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform_slam_base.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform_slam_base.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform_slam_base.transform.rotation.w = q.w();
    transform_slam_base.transform.rotation.x = q.x();
    transform_slam_base.transform.rotation.y = q.y();
    transform_slam_base.transform.rotation.z = q.z();
    transform_slam_base.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform_slam_base);
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

namespace point_lio {

class LaserMappingNode : public rclcpp::Node
{
public:
  explicit LaserMappingNode(const rclcpp::NodeOptions & options) : rclcpp::Node("laserMapping", options)
  {
    initialize();
    startWorker();
  }

  ~LaserMappingNode() override
  {
    stopWorker();
    if (!pcl_wait_save->empty() && pcd_save_en) {
      string file_name = string("scans.pcd");
      string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
      pcl::PCDWriter pcd_writer;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    if (fp_) {
      fclose(fp_);
      fp_ = nullptr;
    }
    fout_out.close();
    fout_imu_pbp.close();
  }

private:
  void initialize()
  {
    icp_client_ = create_client<std_srvs::srv::Trigger>("/gicp_recall");

    readParameters(*this);
    std::cout << "lidar_type: " << lidar_type << '\n';
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = "camera_init";

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    if (extrinsic_est_en) {
      if (!use_imu_as_input) {
        kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
        kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
      } else {
        kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
        kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
      }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    reset_cov(P_init_);
    kf_input.change_P(P_init_);
    reset_cov_output(P_init_output_);
    kf_output.change_P(P_init_output_);
    Q_input_ = process_noise_cov_input();
    Q_output_ = process_noise_cov_output();

    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp_ = fopen(pos_log_dir.c_str(), "w");
    open_file();

    if (print_cloud_input_fps) {
      RCLCPP_INFO(get_logger(),
        "[Point-LIO] runtime rate print enabled, period: %.2f s, lidar topic: %s",
        cloud_input_fps_print_period > 0.0 ? cloud_input_fps_print_period : 1.0,
        lid_topic.c_str());
    }

    if (p_pre->lidar_type == AVIA) {
      sub_pcl_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(), [](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
          record_runtime_rate(RuntimeRateEvent::CloudInput);
          livox_pcl_cbk(std::move(msg));
        });
    } else {
      sub_pcl_pc_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(), [](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
          record_runtime_rate(RuntimeRateEvent::CloudInput);
          standard_pcl_cbk(std::move(msg));
        });
    }
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
    pub_laser_cloud_full_res_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
    pub_laser_cloud_full_world_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_full", rclcpp::SensorDataQoS().keep_last(1));
    pub_laser_cloud_full_res_body_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
    pub_laser_cloud_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
    pub_odom_aft_mapped_ = create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 20);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 20);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  void startWorker()
  {
    running_.store(true);
    worker_thread_ = std::thread(&LaserMappingNode::processingLoop, this);
  }

  void stopWorker()
  {
    running_.store(false);
    sig_buffer.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  void processingLoop();

  std::atomic_bool running_{false};
  std::thread worker_thread_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr icp_client_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_res_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_full_res_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  FILE * fp_{nullptr};
  int frame_num_{0};
  double aver_time_consu_{0.0};
  double aver_time_icp_{0.0};
  double aver_time_match_{0.0};
  double aver_time_incre_{0.0};
  double aver_time_solve_{0.0};
  double aver_time_propag_{0.0};
  double last_proc_time_{-1.0};
  int startup_frame_cnt_{0};
  Eigen::Matrix<double, 24, 24> P_init_;
  Eigen::Matrix<double, 30, 30> P_init_output_;
  Eigen::Matrix<double, 24, 24> Q_input_;
  Eigen::Matrix<double, 30, 30> Q_output_;
};

void LaserMappingNode::processingLoop()
{
  rclcpp::Rate rate(500);
  while (rclcpp::ok() && running_.load()) {
    if (flg_exit)
      break;
    if (sync_packages(Measures)) {
      const auto process_wall_start = std::chrono::steady_clock::now();
      record_runtime_rate(RuntimeRateEvent::SyncInput);
      record_pose_update_debug_sync(Measures.lidar ? Measures.lidar->size() : 0);
      startup_frame_cnt_++;

      bool trigger_exit = false;
      std::string exit_reason = "";

      // 1. [故障监测] 时间戳乱序 (严重系统错误)
      if (last_proc_time_ > 0 && Measures.lidar_beg_time < last_proc_time_) {
        exit_reason = "Time Disorder Detected (Curr < Last)";
        trigger_exit = true;
      }

      // 2. [故障监测] 网络大延时 (严重丢包/网络拥塞)
      if (!trigger_exit && last_proc_time_ > 0 && (Measures.lidar_beg_time - last_proc_time_) > 1.0 &&
          startup_frame_cnt_ > 2000) {
        exit_reason = "Large Time Gap (>0.5s)";
        trigger_exit = true;
      }

      // [执行] 触发退出，由 ROS launch 的 respawn 机制接管
      if (trigger_exit) {
        RCLCPP_FATAL(LOGGER, "[CRASH MONITOR] %s. Respawning...", exit_reason.c_str());

        if (icp_client_->wait_for_service(std::chrono::seconds(1))) {
          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
          auto result = icp_client_->async_send_request(request);
          // 简单等待3s，期间维持通信，确保请求发出
          auto start = std::chrono::steady_clock::now();
          while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(3)) {
            if (result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
              break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
        exit(EXIT_FAILURE);
      }

      last_proc_time_ = Measures.lidar_beg_time;
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init_);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output_);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;
        full_cloud_queue.clear();
        has_full_cloud_anchor = false;
        full_cloud_world->clear();
        full_cloud_dropped_points_this_scan = 0;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
        global_map_ptr->clear();
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();
      {
        const double full_undistort_start = omp_get_wtime();
        p_imu->Process(Measures, feats_undistort);
        record_pose_update_debug_full_undistort(
          (omp_get_wtime() - full_undistort_start) * 1000.0);
      }
      enqueueFullCloud(feats_undistort, Measures.lidar_beg_time);
      if (!has_full_cloud_anchor) {
        full_cloud_anchor = makeStateSnapshot(Measures.lidar_beg_time);
        full_cloud_anchor.stamp = Measures.lidar_beg_time;
        has_full_cloud_anchor = true;
      }
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }
      record_pose_update_debug_downsample(feats_down_size);

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          if (enable_prior_pcd) {
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            if (map_cloud) {
              ivox_->AddPoints(map_cloud->points);
              if (pcd_save_en) {
                *global_map_ptr += *map_cloud;
              }
            }
          } else {
            ivox_->AddPoints(init_feats_world->points);
            if (pcd_save_en) {
              *global_map_ptr += *init_feats_world;
            }
          }
          if (pcd_save_en) {
            publish_init_map(pub_laser_cloud_map_);
          }
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        full_cloud_queue.clear();
        has_full_cloud_anchor = false;
        full_cloud_world->clear();
        full_cloud_dropped_points_this_scan = 0;
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu =
                get_time_sec(imu_next.header.stamp) == get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const && !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output_, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output_, input_in, false, true);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output_, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output_, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            record_runtime_rate(RuntimeRateEvent::PoseUpdate);
            record_pose_update_debug_update(static_cast<uint64_t>(time_seq[k]), time_current);
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                         << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose() << " "
                         << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (
                      get_time_sec(imu_next.header.stamp) < Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output_, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output_, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  // time_current;
              }
              kf_input.predict(dt, Q_input_, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty())
                break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input_, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            record_runtime_rate(RuntimeRateEvent::PoseUpdate);
            record_pose_update_debug_update(static_cast<uint64_t>(time_seq[k]), time_current);
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (
                      get_time_sec(imu_next.header.stamp) < Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input_, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  // time_current;
                }
                // kf_input.predict(dt, Q_input_, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped_, tf_broadcaster_);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      double map_incremental_ms = 0.0;
      bool map_incremental_called = false;
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (sleep_time > 200) {
            const double map_incremental_start = omp_get_wtime();
            MapIncremental();
            map_incremental_ms = (omp_get_wtime() - map_incremental_start) * 1000.0;
            map_incremental_called = true;
          }
        } else {
          const double map_incremental_start = omp_get_wtime();
          MapIncremental();
          map_incremental_ms = (omp_get_wtime() - map_incremental_start) * 1000.0;
          map_incremental_called = true;
        }
      }
      if (map_incremental_called) {
        record_pose_update_debug_map_incremental(map_incremental_ms);
      }
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en)
        publish_path(pub_path_);
      if (scan_pub_en || pcd_save_en)
        publish_frame_world(pub_laser_cloud_full_res_);
      if (has_full_cloud_anchor && lidar_end_time > full_cloud_anchor.stamp) {
        processFullCloudSegmentWithSnapshot(makeStateSnapshot(lidar_end_time));
      }
      publishAccumulatedFullCloudWorld(pub_laser_cloud_full_world_, lidar_end_time);
      if (scan_pub_en && scan_body_pub_en)
        publish_frame_body(pub_laser_cloud_full_res_body_);
      if (scan_pub_en)
        publish_accumulated_map(pub_laser_cloud_map_);

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num_++;
        aver_time_consu_ = aver_time_consu_ * (frame_num_ - 1) / frame_num_ + (t5 - t0) / frame_num_;
        {
          aver_time_icp_ = aver_time_icp_ * (frame_num_ - 1) / frame_num_ + update_time / frame_num_;
        }
        aver_time_match_ = aver_time_match_ * (frame_num_ - 1) / frame_num_ + (match_time) / frame_num_;
        aver_time_solve_ = aver_time_solve_ * (frame_num_ - 1) / frame_num_ + solve_time / frame_num_;
        aver_time_propag_ = aver_time_propag_ * (frame_num_ - 1) / frame_num_ + propag_time / frame_num_;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu_;
        time_log_counter++;
        printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
               "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0,
          aver_time_match_,
          aver_time_solve_,
          t3 - t1,
          t5 - t3,
          aver_time_consu_,
          aver_time_icp_,
          aver_time_propag_);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose() << " "
                     << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp_);
      }
      const auto process_wall_end = std::chrono::steady_clock::now();
      record_runtime_process_time(
        std::chrono::duration<double, std::milli>(process_wall_end - process_wall_start).count());
    }
    rate.sleep();
  }
}

}  // namespace point_lio

#ifndef POINT_LIO_BUILD_MAIN
RCLCPP_COMPONENTS_REGISTER_NODE(point_lio::LaserMappingNode)
#endif

#ifdef POINT_LIO_BUILD_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, SigHandle);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  auto node = std::make_shared<point_lio::LaserMappingNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
