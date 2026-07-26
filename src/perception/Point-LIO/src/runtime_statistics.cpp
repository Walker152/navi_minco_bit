#include "runtime_statistics.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace point_lio {

namespace {

template <std::size_t N> void writeArray(std::ostream & stream, const std::array<double, N> & values)
{
  for (const double value : values) {
    stream << value << ' ';
  }
}

double finiteOrZero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

std::filesystem::path resolveLogDirectory(
  const std::filesystem::path & configured_path, const std::filesystem::path & root_directory)
{
  if (configured_path.empty()) {
    return root_directory / "Log";
  }
  if (configured_path.is_absolute()) {
    return configured_path;
  }
  return root_directory / configured_path;
}

RuntimeStatistics::~RuntimeStatistics()
{
  shutdown();
}

RuntimeStatistics & RuntimeStatistics::instance()
{
  static RuntimeStatistics statistics;
  return statistics;
}

bool RuntimeStatistics::initialize(bool enabled,
  const std::filesystem::path & log_directory,
  bool print_rate_summary,
  bool print_pose_detail,
  double print_period_sec)
{
  shutdown();
  if (!enabled) {
    return true;
  }

  std::error_code error;
  std::filesystem::create_directories(log_directory, error);
  if (error) {
    std::cerr << "[Point-LIO][RuntimeStatistics] Failed to create " << log_directory << ": "
              << error.message() << '\n';
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  imu_stream_.clear();
  mat_stream_.clear();
  pos_stream_.clear();
  performance_stream_.clear();
  imu_stream_.open(log_directory / "imu_pbp.txt", std::ios::out);
  mat_stream_.open(log_directory / "mat_out.txt", std::ios::out);
  pos_stream_.open(log_directory / "pos_log.txt", std::ios::out);
  performance_stream_.open(log_directory / "performance.csv", std::ios::out);
  if (!imu_stream_ || !mat_stream_ || !pos_stream_ || !performance_stream_) {
    std::cerr << "[Point-LIO][RuntimeStatistics] Failed to open log files under " << log_directory << '\n';
    closeFilesLocked();
    return false;
  }

  imu_stream_ << "# seq original_stamp corrected_stamp sensor_dt arrival_dt estimated_missing status "
                 "gyro_x gyro_y gyro_z acc_x acc_y acc_z pending_queue_size\n"
              << "# status: 0=normal 1=gap 2=duplicate 3=rollback_dropped\n";

  performance_stream_ << "frame_seq,elapsed_ms,lidar_beg_stamp,lidar_end_stamp,latest_lidar_stamp,"
                         "lidar_backlog_ms,pending_lidar_frames,lidar_buffer_frames,pending_imu_samples,"
                         "imu_buffer_samples,input_points,downsample_points,pose_update_attempt_count,"
                         "pose_update_count,pose_update_failure_count,pose_update_points_avg,"
                         "pose_update_points_min,pose_update_points_max,ekf_update_ms,ekf_update_max_ms,"
                         "effective_feature_points,odom_publish_count,preprocess_ms,process_ms,"
                         "full_undistort_ms,map_incremental_ms,path_publish_ms,cloud_publish_ms,"
                         "full_cloud_publish_ms,map_publish_ms,odom_publish_age_ms,full_cloud_queue_points,"
                         "global_map_points,pcd_wait_points,path_pose_count,ivox_stats_sampled,"
                         "ivox_valid_grids,ivox_points_per_grid_avg,ivox_points_per_grid_max,"
                         "ivox_points_per_grid_stddev,ivox_stats_ms\n";

  print_rate_summary_ = print_rate_summary;
  print_pose_detail_ = print_pose_detail;
  print_period_sec_ = print_period_sec > 0.05 ? print_period_sec : 1.0;
  latest_lidar_stamp_ = std::numeric_limits<double>::quiet_NaN();
  latest_preprocess_ms_ = std::numeric_limits<double>::quiet_NaN();
  last_odom_publish_age_ms_ = std::numeric_limits<double>::quiet_NaN();
  frame_full_undistort_ms_ = 0.0;
  frame_map_incremental_ms_ = 0.0;
  frame_ekf_update_ms_ = 0.0;
  frame_ekf_update_max_ms_ = 0.0;
  frame_update_points_sum_ = 0.0;
  process_time_sum_ms_ = 0.0;
  process_time_max_ms_ = 0.0;
  update_points_sum_ = 0.0;
  last_pose_sensor_time_ = std::numeric_limits<double>::quiet_NaN();
  pending_lidar_frames_ = 0;
  lidar_buffer_frames_ = 0;
  pending_imu_samples_ = 0;
  imu_buffer_samples_ = 0;
  frame_input_points_ = 0;
  cloud_input_count_ = 0;
  sync_input_count_ = 0;
  pose_update_attempt_count_ = 0;
  pose_update_count_ = 0;
  odom_publish_count_ = 0;
  frame_pose_update_attempt_count_ = 0;
  frame_pose_update_count_ = 0;
  frame_pose_update_failure_count_ = 0;
  frame_odom_publish_count_ = 0;
  frame_min_points_per_update_ = std::numeric_limits<uint64_t>::max();
  frame_max_points_per_update_ = 0;
  frame_sequence_ = 0;
  process_count_ = 0;
  min_points_per_update_ = std::numeric_limits<uint64_t>::max();
  max_points_per_update_ = 0;
  start_time_ = std::chrono::steady_clock::now();
  window_start_ = start_time_;
  last_flush_time_ = start_time_;
  enabled_.store(true, std::memory_order_release);
  return true;
}

void RuntimeStatistics::shutdown()
{
  enabled_.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> lock(mutex_);
  flushFilesLocked();
  closeFilesLocked();
}

bool RuntimeStatistics::enabled() const noexcept
{
  return enabled_.load(std::memory_order_acquire);
}

void RuntimeStatistics::recordRate(RuntimeRateEvent event)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (event == RuntimeRateEvent::CloudInput) {
    ++cloud_input_count_;
  } else {
    ++sync_input_count_;
  }
}

void RuntimeStatistics::recordLidarCallback(double lidar_stamp,
  double preprocess_ms,
  std::size_t pending_lidar_frames,
  std::size_t lidar_buffer_frames)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_lidar_stamp_ = lidar_stamp;
  latest_preprocess_ms_ = preprocess_ms;
  pending_lidar_frames_ = pending_lidar_frames;
  lidar_buffer_frames_ = lidar_buffer_frames;
}

void RuntimeStatistics::recordSensorQueues(std::size_t pending_lidar_frames,
  std::size_t lidar_buffer_frames,
  std::size_t pending_imu_samples,
  std::size_t imu_buffer_samples)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_lidar_frames_ = pending_lidar_frames;
  lidar_buffer_frames_ = lidar_buffer_frames;
  pending_imu_samples_ = pending_imu_samples;
  imu_buffer_samples_ = imu_buffer_samples;
}

void RuntimeStatistics::beginFrame(double, std::size_t input_points)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  frame_input_points_ = input_points;
  frame_pose_update_count_ = 0;
  frame_pose_update_attempt_count_ = 0;
  frame_pose_update_failure_count_ = 0;
  frame_odom_publish_count_ = 0;
  frame_full_undistort_ms_ = 0.0;
  frame_map_incremental_ms_ = 0.0;
  frame_ekf_update_ms_ = 0.0;
  frame_ekf_update_max_ms_ = 0.0;
  frame_update_points_sum_ = 0.0;
  frame_min_points_per_update_ = std::numeric_limits<uint64_t>::max();
  frame_max_points_per_update_ = 0;
}

void RuntimeStatistics::recordPoseUpdate(
  uint64_t points_per_update, double sensor_time, double update_time_ms, bool successful)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ++pose_update_attempt_count_;
  ++frame_pose_update_attempt_count_;
  frame_update_points_sum_ += static_cast<double>(points_per_update);
  frame_min_points_per_update_ = std::min(frame_min_points_per_update_, points_per_update);
  frame_max_points_per_update_ = std::max(frame_max_points_per_update_, points_per_update);
  if (std::isfinite(update_time_ms) && update_time_ms >= 0.0) {
    frame_ekf_update_ms_ += update_time_ms;
    frame_ekf_update_max_ms_ = std::max(frame_ekf_update_max_ms_, update_time_ms);
  }

  update_points_sum_ += static_cast<double>(points_per_update);
  min_points_per_update_ = std::min(min_points_per_update_, points_per_update);
  max_points_per_update_ = std::max(max_points_per_update_, points_per_update);
  if (successful) {
    ++pose_update_count_;
    ++frame_pose_update_count_;
    last_pose_sensor_time_ = sensor_time;
  } else {
    ++frame_pose_update_failure_count_;
  }
}

void RuntimeStatistics::recordOdomPublish(double odom_stamp, double publish_time)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ++odom_publish_count_;
  ++frame_odom_publish_count_;
  last_odom_publish_age_ms_ = (publish_time - odom_stamp) * 1000.0;
}

void RuntimeStatistics::recordFullUndistort(double time_ms)
{
  if (!enabled() || !std::isfinite(time_ms) || time_ms < 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  frame_full_undistort_ms_ += time_ms;
}

void RuntimeStatistics::recordMapIncremental(double time_ms)
{
  if (!enabled() || !std::isfinite(time_ms) || time_ms < 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  frame_map_incremental_ms_ += time_ms;
}

void RuntimeStatistics::recordImu(const ImuLogRecord & record)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  imu_stream_ << record.sequence << ' ' << std::fixed << std::setprecision(9) << record.original_stamp
              << ' ' << record.corrected_stamp << ' ' << record.sensor_dt << ' ' << record.arrival_dt << ' '
              << record.estimated_missing << ' ' << record.status << ' ';
  writeArray(imu_stream_, record.gyro);
  writeArray(imu_stream_, record.acc);
  imu_stream_ << record.pending_queue_size << '\n';
  maybeFlushLocked(std::chrono::steady_clock::now());
}

void RuntimeStatistics::recordMat(const StateLogRecord & record)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  mat_stream_ << std::setprecision(9) << record.relative_time << ' ';
  writeArray(mat_stream_, record.euler);
  writeArray(mat_stream_, record.pos);
  writeArray(mat_stream_, record.vel);
  if (!record.use_imu_as_input) {
    writeArray(mat_stream_, record.omega);
    writeArray(mat_stream_, record.acc);
    writeArray(mat_stream_, record.gravity);
    writeArray(mat_stream_, record.bg);
    writeArray(mat_stream_, record.ba);
  } else {
    writeArray(mat_stream_, record.bg);
    writeArray(mat_stream_, record.ba);
    writeArray(mat_stream_, record.gravity);
  }
  mat_stream_ << record.point_count << '\n';
}

void RuntimeStatistics::recordPos(const StateLogRecord & record)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const std::array<double, 3> zeros{};
  pos_stream_ << std::setprecision(9) << record.relative_time << ' ';
  writeArray(pos_stream_, record.euler);
  writeArray(pos_stream_, record.pos);
  writeArray(pos_stream_, zeros);
  writeArray(pos_stream_, record.vel);
  writeArray(pos_stream_, zeros);
  writeArray(pos_stream_, record.bg);
  writeArray(pos_stream_, record.ba);
  writeArray(pos_stream_, record.gravity);
  pos_stream_ << '\n';
}

void RuntimeStatistics::recordFrame(const FramePerformanceRecord & record)
{
  if (!enabled()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++frame_sequence_;
  ++process_count_;
  process_time_sum_ms_ += record.process_ms;
  process_time_max_ms_ = std::max(process_time_max_ms_, record.process_ms);

  const double latest_lidar_stamp =
    std::isfinite(record.latest_lidar_stamp) ? record.latest_lidar_stamp : latest_lidar_stamp_;
  const double preprocess_ms =
    std::isfinite(record.preprocess_ms) ? record.preprocess_ms : latest_preprocess_ms_;
  const double lidar_backlog_ms = std::isfinite(latest_lidar_stamp)
                                    ? (latest_lidar_stamp - record.lidar_beg_stamp) * 1000.0
                                    : std::numeric_limits<double>::quiet_NaN();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(now - start_time_).count();
  const double frame_update_points_avg =
    frame_pose_update_attempt_count_ > 0
      ? frame_update_points_sum_ / static_cast<double>(frame_pose_update_attempt_count_)
      : 0.0;
  const uint64_t frame_min_points = frame_pose_update_attempt_count_ > 0 ? frame_min_points_per_update_ : 0;

  performance_stream_
    << std::setprecision(9) << frame_sequence_ << ',' << elapsed_ms << ',' << record.lidar_beg_stamp << ','
    << record.lidar_end_stamp << ',' << latest_lidar_stamp << ',' << lidar_backlog_ms << ','
    << pending_lidar_frames_ << ',' << lidar_buffer_frames_ << ',' << pending_imu_samples_ << ','
    << imu_buffer_samples_ << ',' << (record.input_points > 0 ? record.input_points : frame_input_points_)
    << ',' << record.downsample_points << ',' << frame_pose_update_attempt_count_ << ','
    << frame_pose_update_count_ << ',' << frame_pose_update_failure_count_ << ',' << frame_update_points_avg
    << ',' << frame_min_points << ',' << frame_max_points_per_update_ << ',' << frame_ekf_update_ms_ << ','
    << frame_ekf_update_max_ms_ << ',' << record.effective_feature_points << ','
    << frame_odom_publish_count_ << ',' << preprocess_ms << ',' << record.process_ms << ','
    << (record.full_undistort_ms > 0.0 ? record.full_undistort_ms : frame_full_undistort_ms_) << ','
    << (record.map_incremental_ms > 0.0 ? record.map_incremental_ms : frame_map_incremental_ms_) << ','
    << record.path_publish_ms << ',' << record.cloud_publish_ms << ',' << record.full_cloud_publish_ms
    << ',' << record.map_publish_ms << ',' << last_odom_publish_age_ms_ << ','
    << record.full_cloud_queue_points << ',' << record.global_map_points << ',' << record.pcd_wait_points
    << ',' << record.path_pose_count << ',' << static_cast<int>(record.ivox.sampled) << ','
    << record.ivox.valid_grids << ',' << record.ivox.points_per_grid_avg << ','
    << record.ivox.points_per_grid_max << ',' << record.ivox.points_per_grid_stddev << ','
    << record.ivox.collection_ms << '\n';

  maybePrintSummaryLocked(now);
  maybeFlushLocked(now);
}

void RuntimeStatistics::closeFilesLocked()
{
  if (imu_stream_.is_open()) {
    imu_stream_.close();
  }
  if (mat_stream_.is_open()) {
    mat_stream_.close();
  }
  if (pos_stream_.is_open()) {
    pos_stream_.close();
  }
  if (performance_stream_.is_open()) {
    performance_stream_.close();
  }
}

void RuntimeStatistics::flushFilesLocked()
{
  if (imu_stream_.is_open()) {
    imu_stream_.flush();
  }
  if (mat_stream_.is_open()) {
    mat_stream_.flush();
  }
  if (pos_stream_.is_open()) {
    pos_stream_.flush();
  }
  if (performance_stream_.is_open()) {
    performance_stream_.flush();
  }
}

void RuntimeStatistics::maybeFlushLocked(std::chrono::steady_clock::time_point now)
{
  if (now - last_flush_time_ >= std::chrono::seconds(1)) {
    flushFilesLocked();
    last_flush_time_ = now;
  }
}

void RuntimeStatistics::maybePrintSummaryLocked(std::chrono::steady_clock::time_point now)
{
  const double elapsed = std::chrono::duration<double>(now - window_start_).count();
  if (elapsed < print_period_sec_) {
    return;
  }

  if (print_rate_summary_ || print_pose_detail_) {
    const double cloud_hz = static_cast<double>(cloud_input_count_) / elapsed;
    const double sync_hz = static_cast<double>(sync_input_count_) / elapsed;
    const double pose_hz = static_cast<double>(pose_update_count_) / elapsed;
    const double odom_hz = static_cast<double>(odom_publish_count_) / elapsed;
    const double avg_process_ms =
      process_count_ > 0 ? process_time_sum_ms_ / static_cast<double>(process_count_) : 0.0;
    const double avg_points = pose_update_attempt_count_ > 0
                                ? update_points_sum_ / static_cast<double>(pose_update_attempt_count_)
                                : 0.0;
    const uint64_t min_points = pose_update_attempt_count_ > 0 ? min_points_per_update_ : 0;

    std::cout << "[Point-LIO][RuntimeStatistics] cloud=" << cloud_hz << "Hz sync=" << sync_hz
              << "Hz pose_update=" << pose_hz << "Hz odom=" << odom_hz
              << "Hz process_ms(avg/max)=" << avg_process_ms << '/' << process_time_max_ms_
              << " points/update(avg/min/max)=" << avg_points << '/' << min_points << '/'
              << max_points_per_update_ << " lidar_backlog_ms="
              << (std::isfinite(latest_lidar_stamp_) && std::isfinite(last_pose_sensor_time_)
                     ? (latest_lidar_stamp_ - last_pose_sensor_time_) * 1000.0
                     : 0.0)
              << " odom_age_ms=" << finiteOrZero(last_odom_publish_age_ms_)
              << " queues(lidar/imu)=" << lidar_buffer_frames_ << '/' << imu_buffer_samples_ << '\n';
  }

  cloud_input_count_ = 0;
  sync_input_count_ = 0;
  pose_update_attempt_count_ = 0;
  pose_update_count_ = 0;
  odom_publish_count_ = 0;
  process_count_ = 0;
  process_time_sum_ms_ = 0.0;
  process_time_max_ms_ = 0.0;
  update_points_sum_ = 0.0;
  min_points_per_update_ = std::numeric_limits<uint64_t>::max();
  max_points_per_update_ = 0;
  window_start_ = now;
}

}  // namespace point_lio
