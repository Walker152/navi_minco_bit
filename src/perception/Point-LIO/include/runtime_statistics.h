#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>

namespace point_lio {

std::filesystem::path resolveLogDirectory(
  const std::filesystem::path & configured_path, const std::filesystem::path & root_directory);

enum class RuntimeRateEvent
{
  CloudInput,
  SyncInput,
};

struct ImuLogRecord
{
  uint64_t sequence{0};
  double original_stamp{0.0};
  double corrected_stamp{0.0};
  double sensor_dt{0.0};
  double arrival_dt{0.0};
  int estimated_missing{0};
  int status{0};
  std::array<double, 3> gyro{};
  std::array<double, 3> acc{};
  std::size_t pending_queue_size{0};
};

struct StateLogRecord
{
  double relative_time{0.0};
  bool use_imu_as_input{false};
  std::array<double, 3> euler{};
  std::array<double, 3> pos{};
  std::array<double, 3> vel{};
  std::array<double, 3> omega{};
  std::array<double, 3> acc{};
  std::array<double, 3> gravity{};
  std::array<double, 3> bg{};
  std::array<double, 3> ba{};
  std::size_t point_count{0};
};

struct IvoxStatisticsRecord
{
  bool sampled{false};
  std::size_t valid_grids{0};
  double points_per_grid_avg{0.0};
  std::size_t points_per_grid_max{0};
  double points_per_grid_stddev{0.0};
  double collection_ms{0.0};
};

struct FramePerformanceRecord
{
  double lidar_beg_stamp{0.0};
  double lidar_end_stamp{0.0};
  double latest_lidar_stamp{std::numeric_limits<double>::quiet_NaN()};
  double preprocess_ms{std::numeric_limits<double>::quiet_NaN()};
  double process_ms{0.0};
  double full_undistort_ms{0.0};
  double map_incremental_ms{0.0};
  double path_publish_ms{0.0};
  double cloud_publish_ms{0.0};
  double full_cloud_publish_ms{0.0};
  double map_publish_ms{0.0};
  std::size_t input_points{0};
  std::size_t downsample_points{0};
  std::size_t full_cloud_queue_points{0};
  std::size_t global_map_points{0};
  std::size_t pcd_wait_points{0};
  std::size_t path_pose_count{0};
  std::size_t effective_feature_points{0};
  IvoxStatisticsRecord ivox{};
};

class RuntimeStatistics
{
public:
  RuntimeStatistics() = default;
  ~RuntimeStatistics();

  RuntimeStatistics(const RuntimeStatistics &) = delete;
  RuntimeStatistics & operator=(const RuntimeStatistics &) = delete;

  static RuntimeStatistics & instance();

  bool initialize(bool enabled,
    const std::filesystem::path & log_directory,
    bool print_rate_summary,
    bool print_pose_detail,
    double print_period_sec);
  void shutdown();
  bool enabled() const noexcept;

  void recordRate(RuntimeRateEvent event);
  void recordLidarCallback(double lidar_stamp,
    double preprocess_ms,
    std::size_t pending_lidar_frames,
    std::size_t lidar_buffer_frames);
  void recordSensorQueues(std::size_t pending_lidar_frames,
    std::size_t lidar_buffer_frames,
    std::size_t pending_imu_samples,
    std::size_t imu_buffer_samples);
  void beginFrame(double lidar_beg_stamp, std::size_t input_points);
  void recordPoseUpdate(
    uint64_t points_per_update, double sensor_time, double update_time_ms, bool successful);
  void recordOdomPublish(double odom_stamp, double publish_time);
  void recordFullUndistort(double time_ms);
  void recordMapIncremental(double time_ms);
  void recordImu(const ImuLogRecord & record);
  void recordMat(const StateLogRecord & record);
  void recordPos(const StateLogRecord & record);
  void recordFrame(const FramePerformanceRecord & record);

private:
  void closeFilesLocked();
  void flushFilesLocked();
  void maybeFlushLocked(std::chrono::steady_clock::time_point now);
  void maybePrintSummaryLocked(std::chrono::steady_clock::time_point now);

  std::atomic<bool> enabled_{false};
  mutable std::mutex mutex_;
  std::ofstream imu_stream_;
  std::ofstream mat_stream_;
  std::ofstream pos_stream_;
  std::ofstream performance_stream_;

  bool print_rate_summary_{false};
  bool print_pose_detail_{false};
  double print_period_sec_{1.0};
  std::chrono::steady_clock::time_point start_time_{};
  std::chrono::steady_clock::time_point window_start_{};
  std::chrono::steady_clock::time_point last_flush_time_{};

  double latest_lidar_stamp_{std::numeric_limits<double>::quiet_NaN()};
  double latest_preprocess_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_odom_publish_age_ms_{std::numeric_limits<double>::quiet_NaN()};
  double frame_full_undistort_ms_{0.0};
  double frame_map_incremental_ms_{0.0};
  double frame_ekf_update_ms_{0.0};
  double frame_ekf_update_max_ms_{0.0};
  double frame_update_points_sum_{0.0};
  double process_time_sum_ms_{0.0};
  double process_time_max_ms_{0.0};
  double update_points_sum_{0.0};
  double last_pose_sensor_time_{std::numeric_limits<double>::quiet_NaN()};

  std::size_t pending_lidar_frames_{0};
  std::size_t lidar_buffer_frames_{0};
  std::size_t pending_imu_samples_{0};
  std::size_t imu_buffer_samples_{0};
  std::size_t frame_input_points_{0};

  uint64_t cloud_input_count_{0};
  uint64_t sync_input_count_{0};
  uint64_t pose_update_attempt_count_{0};
  uint64_t pose_update_count_{0};
  uint64_t odom_publish_count_{0};
  uint64_t frame_pose_update_attempt_count_{0};
  uint64_t frame_pose_update_count_{0};
  uint64_t frame_pose_update_failure_count_{0};
  uint64_t frame_odom_publish_count_{0};
  uint64_t frame_min_points_per_update_{std::numeric_limits<uint64_t>::max()};
  uint64_t frame_max_points_per_update_{0};
  uint64_t frame_sequence_{0};
  uint64_t process_count_{0};
  uint64_t min_points_per_update_{std::numeric_limits<uint64_t>::max()};
  uint64_t max_points_per_update_{0};
};

}  // namespace point_lio
