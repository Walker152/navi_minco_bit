#include <rog_map/performance_monitor.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double safeValue(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

double hzFrom(double count, double first_stamp, double last_stamp)
{
  const double dt = last_stamp - first_stamp;
  if (count <= 1.0 || dt <= 1.0e-6) {
    return 0.0;
  }
  return (count - 1.0) / dt;
}

std::string num(double value)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << safeValue(value);
  return ss.str();
}

std::string boolean(double value)
{
  return value != 0.0 ? "1" : "0";
}

void writeCsvLine(std::ofstream & stream, const std::vector<std::string> & fields)
{
  for (size_t i = 0; i < fields.size(); ++i) {
    stream << fields[i];
    if (i + 1 < fields.size()) {
      stream << ',';
    }
  }
  stream << '\n';
}

}  // namespace

PerformanceMonitor::ScopedTimer::ScopedTimer(PerformanceMonitor * monitor, double RuntimeStats::*field)
: monitor_(monitor), field_(field)
{
  if (monitor_ && monitor_->enabled() && field_) {
    start_ = std::chrono::steady_clock::now();
  } else {
    monitor_ = nullptr;
    field_ = nullptr;
  }
}

PerformanceMonitor::ScopedTimer::~ScopedTimer()
{
  if (!monitor_ || !field_) {
    return;
  }
  monitor_->addElapsed(field_, elapsedMs(start_));
}

void PerformanceMonitor::configure(const PerformanceConfig & config)
{
  close();
  config_ = config;
  if (config_.csv_flush_every_n <= 0) {
    config_.csv_flush_every_n = 30;
  }
  resetStats();
  resetWindow(0.0);
  performance_csv_rows_ = 0;
  detailed_csv_rows_ = 0;
  summary_csv_rows_ = 0;

  if (!config_.enable) {
    return;
  }
  if (csvEnabled()) {
    performance_csv_.open(config_.csv_path, std::ios::out | std::ios::trunc);
    map_info_csv_.open(config_.map_info_csv_path, std::ios::out | std::ios::trunc);
    if (!performance_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open csv_path: " << config_.csv_path << std::endl;
    }
    if (!map_info_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open map_info_csv_path: " << config_.map_info_csv_path
                << std::endl;
    }
  }
  if (detailedCsvEnabled()) {
    detailed_csv_.open(config_.detailed_csv_path, std::ios::out | std::ios::trunc);
    if (!detailed_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open detailed_csv_path: " << config_.detailed_csv_path
                << std::endl;
    } else {
      writeDetailedHeader();
    }
  }
  if (summaryCsvEnabled()) {
    summary_csv_.open(config_.summary_csv_path, std::ios::out | std::ios::trunc);
    if (!summary_csv_.is_open()) {
      std::cerr << "[ROGMapPerf] failed to open summary_csv_path: " << config_.summary_csv_path
                << std::endl;
    } else {
      writeSummaryHeader();
    }
  }
}

void PerformanceMonitor::writePerformanceCsvHeader(const std::vector<std::string> & fields)
{
  if (!csvEnabled() || !performance_csv_.is_open()) {
    return;
  }
  writeCsvLine(performance_csv_, fields);
}

void PerformanceMonitor::writePerformanceCsvRow(const std::vector<double> & values)
{
  if (!csvEnabled() || !performance_csv_.is_open()) {
    return;
  }
  std::vector<std::string> fields;
  fields.reserve(values.size());
  for (const double value : values) {
    fields.push_back(num(value));
  }
  writeCsvLine(performance_csv_, fields);
  flushIfNeeded(performance_csv_, performance_csv_rows_);
}

void PerformanceMonitor::recordCloudCallback(
  double stamp, double points, double queue_delay_ms, double convert_time_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (first_cloud_stamp_ <= 0.0) {
    first_cloud_stamp_ = stamp;
  }
  last_cloud_stamp_ = stamp;
  cloud_callback_count_ += 1.0;
  cloud_points_sum_ += points;
  cloud_points_max_ = std::max(cloud_points_max_, points);
  last_cloud_points_ = points;
  last_cloud_queue_delay_ms_ = queue_delay_ms;
  last_cloud_convert_time_ms_ = convert_time_ms;
  if (window_.start_stamp <= 0.0) {
    resetWindow(stamp);
  }
  window_.cloud_callbacks += 1.0;
  window_.last_stamp = stamp;
}

void PerformanceMonitor::recordCloudConvertTime(double convert_time_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_cloud_convert_time_ms_ = convert_time_ms;
}

void PerformanceMonitor::recordCloudDropEmpty()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_empty_count_ += 1.0;
}

void PerformanceMonitor::recordCloudDropNoOdom()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_no_odom_count_ += 1.0;
}

void PerformanceMonitor::recordCloudDropOdomTimeout()
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_cloud_odom_timeout_count_ += 1.0;
}

void PerformanceMonitor::recordValidCloud(double odom_age_ms)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  valid_cloud_count_ += 1.0;
  last_odom_age_ms_ = odom_age_ms;
  const double stamp = last_cloud_stamp_ > 0.0 ? last_cloud_stamp_ : last_odom_stamp_;
  if (first_valid_update_stamp_ <= 0.0) {
    first_valid_update_stamp_ = stamp;
  }
  last_valid_update_stamp_ = stamp;
}

void PerformanceMonitor::recordOdom(double stamp)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (first_odom_stamp_ <= 0.0) {
    first_odom_stamp_ = stamp;
  }
  last_odom_stamp_ = stamp;
  odom_received_count_ += 1.0;
}

void PerformanceMonitor::fillInputStats(RuntimeStats & stats)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  stats.cloud_callback_count = cloud_callback_count_;
  stats.cloud_callback_hz = hzFrom(cloud_callback_count_, first_cloud_stamp_, last_cloud_stamp_);
  stats.cloud_msg_points = last_cloud_points_;
  stats.cloud_msg_points_avg =
    cloud_callback_count_ > 0.0 ? cloud_points_sum_ / cloud_callback_count_ : 0.0;
  stats.cloud_msg_points_max = cloud_points_max_;
  stats.cloud_convert_time_ms = last_cloud_convert_time_ms_;
  stats.cloud_queue_delay_ms = last_cloud_queue_delay_ms_;
  stats.valid_cloud_count = valid_cloud_count_;
  stats.valid_update_hz = hzFrom(valid_cloud_count_, first_valid_update_stamp_, last_valid_update_stamp_);
  stats.dropped_cloud_empty_count = dropped_cloud_empty_count_;
  stats.dropped_cloud_no_odom_count = dropped_cloud_no_odom_count_;
  stats.dropped_cloud_odom_timeout_count = dropped_cloud_odom_timeout_count_;
  stats.odom_received_count = odom_received_count_;
  stats.odom_hz = hzFrom(odom_received_count_, first_odom_stamp_, last_odom_stamp_);
  stats.odom_age_ms = last_odom_age_ms_;
}

void PerformanceMonitor::observeUpdate(const RuntimeStats & stats)
{
  if (!enabled()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  stats_ = stats;
  if (first_valid_update_stamp_ <= 0.0) {
    first_valid_update_stamp_ = stats.stamp;
  }
  last_valid_update_stamp_ = stats.stamp;
  if (window_.start_stamp <= 0.0) {
    resetWindow(stats.stamp);
  }
  window_.last_stamp = stats.stamp;
  window_.valid_updates += 1.0;
  window_.field_updates += stats.field_actual_update != 0.0 ? 1.0 : 0.0;
  window_.update_count += 1.0;
  window_.update_total_sum += stats.total_update_time;
  window_.update_total_max = std::max(window_.update_total_max, stats.total_update_time);
  window_.raycast_sum += stats.raycast_time;
  window_.raycast_max = std::max(window_.raycast_max, stats.raycast_time);
  window_.projection_sum += stats.projection_total_time;
  window_.projection_max = std::max(window_.projection_max, stats.projection_total_time);
  window_.field_sum += stats.field_time;
  window_.field_max = std::max(window_.field_max, stats.field_time);
  window_.query_sum += stats.query_refresh_time;
  window_.query_max = std::max(window_.query_max, stats.query_refresh_time);
  window_.full_refresh_delta += stats.projection_refresh_reason.rfind("full_", 0) == 0 ? 1.0 : 0.0;
  window_.dirty_update_delta += stats.projection_refresh_reason == "dirty_update" ? 1.0 : 0.0;
  window_.field_skip_delta += stats.field_actual_update == 0.0 ? 1.0 : 0.0;
  window_.field_skip_not_dirty_delta += stats.field_skip_reason == "not_dirty" ? 1.0 : 0.0;
  window_.field_skip_period_not_ready_delta += stats.field_skip_reason == "period_not_ready" ? 1.0 : 0.0;
  window_.mask_changed_delta += stats.layer_mask_changed != 0.0 ? 1.0 : 0.0;
  window_.mask_diff_ratio_sum += stats.layer_mask_diff_ratio;
  window_.dirty_ratio_sum += stats.projection_dirty_ratio;
  window_.input_points_sum += stats.input_point_count;
  window_.input_points_max = std::max(window_.input_points_max, stats.input_point_count);

  if (detailedCsvEnabled() && detailed_csv_.is_open()) {
    writeDetailedRow(stats);
  }
  maybeWriteSummary(stats.stamp);
}

void PerformanceMonitor::close()
{
  if (performance_csv_.is_open()) {
    performance_csv_.flush();
    performance_csv_.close();
  }
  if (map_info_csv_.is_open()) {
    map_info_csv_.flush();
    map_info_csv_.close();
  }
  if (detailed_csv_.is_open()) {
    detailed_csv_.flush();
    detailed_csv_.close();
  }
  if (summary_csv_.is_open()) {
    summary_csv_.flush();
    summary_csv_.close();
  }
}

void PerformanceMonitor::addElapsed(double RuntimeStats::*field, double elapsed_ms)
{
  if (!field) {
    return;
  }
  stats_.*field += elapsed_ms;
}

void PerformanceMonitor::writeDetailedHeader()
{
  writeCsvLine(detailed_csv_,
    {"stamp",
      "update_seq",
      "cloud_callback_hz_window",
      "valid_update_hz_window",
      "cloud_msg_points",
      "cloud_msg_points_avg",
      "cloud_msg_points_max",
      "cloud_queue_delay_ms",
      "cloud_convert_time_ms",
      "valid_cloud_count",
      "dropped_cloud_empty_count",
      "dropped_cloud_no_odom_count",
      "dropped_cloud_odom_timeout_count",
      "odom_hz",
      "odom_age_ms",
      "input_point_count",
      "raycast_input_point_count",
      "raycast_used_point_count",
      "raycast_skipped_near_count",
      "raycast_skipped_far_count",
      "raycast_skipped_outside_count",
      "update_total_time_ms",
      "update_robot_state_time_ms",
      "prob_update_time_ms",
      "raycast_time_ms",
      "raycast_parallel_time_ms",
      "raycast_merge_time_ms",
      "inflation_time_ms",
      "decay_time_ms",
      "refresh_layers_time_ms",
      "performance_csv_write_time_ms",
      "projection_total_time_ms",
      "projection_config_time_ms",
      "projection_update_full_time_ms",
      "projection_update_dirty_time_ms",
      "projection_hole_fill_time_ms",
      "projection_value_mask_time_ms",
      "projection_refresh_reason",
      "projection_cell_count",
      "projection_z_layers",
      "projection_scanned_voxel_estimate",
      "projection_dirty_column_count",
      "projection_dirty_expanded_column_count",
      "projection_dirty_ratio",
      "projection_full_refresh_count",
      "projection_dirty_update_count",
      "projection_no_update_count",
      "projection_thin_surface_count",
      "projection_vertical_wall_count",
      "projection_hollow_tunnel_count",
      "projection_ambiguous_occupied_count",
      "projection_empty_column_count",
      "projection_insufficient_observation_count",
      "layer_mask_changed",
      "layer_mask_diff_count",
      "layer_mask_diff_ratio",
      "layer_mask_free_count",
      "layer_mask_occupied_count",
      "layer_type_free_count",
      "layer_type_passable_count",
      "layer_type_occupied_count",
      "layer_type_unknown_count",
      "field_enabled",
      "field_dirty_before",
      "field_period_ready",
      "field_should_update",
      "field_actual_update",
      "field_skip_reason",
      "field_time_ms",
      "field_edt_positive_time_ms",
      "field_edt_negative_time_ms",
      "field_distance_fill_time_ms",
      "field_copy_time_ms",
      "field_sequence",
      "field_sequence_delta",
      "field_update_count",
      "field_skipped_count",
      "field_skip_not_dirty_count",
      "field_skip_period_not_ready_count",
      "field_skip_layer_empty_count",
      "field_skip_disabled_count",
      "field_stale",
      "field_age_ms",
      "field_update_interval_ms",
      "field_update_hz_window",
      "query_refresh_time_ms",
      "query_snapshot_alloc_time_ms",
      "query_copy_values_time_ms",
      "query_copy_types_height_delta_confidence_time_ms",
      "query_copy_field_distances_time_ms",
      "query_update_pointer_time_ms",
      "query_sequence",
      "query_field_sequence",
      "query_field_stale",
      "query_field_age_ms",
      "query_distance_size",
      "cpu_thread_hint"});
}

void PerformanceMonitor::writeSummaryHeader()
{
  writeCsvLine(summary_csv_,
    {"window_start_stamp",
      "window_duration_s",
      "cloud_callback_hz",
      "valid_update_hz",
      "field_update_hz",
      "avg_update_total_time_ms",
      "max_update_total_time_ms",
      "avg_raycast_time_ms",
      "max_raycast_time_ms",
      "avg_projection_time_ms",
      "max_projection_time_ms",
      "avg_field_time_ms",
      "max_field_time_ms",
      "avg_query_refresh_time_ms",
      "max_query_refresh_time_ms",
      "full_refresh_count_delta",
      "dirty_update_count_delta",
      "field_update_count_delta",
      "field_skip_count_delta",
      "field_skip_not_dirty_delta",
      "field_skip_period_not_ready_delta",
      "mask_changed_count_delta",
      "avg_mask_diff_ratio",
      "avg_dirty_ratio",
      "avg_input_points",
      "max_input_points"});
}

void PerformanceMonitor::writeDetailedRow(const RuntimeStats & s)
{
  const double hw = static_cast<double>(std::max(1U, std::thread::hardware_concurrency()));
  writeCsvLine(detailed_csv_,
    {num(s.stamp),
      num(s.update_seq),
      num(s.cloud_callback_hz),
      num(s.valid_update_hz),
      num(s.cloud_msg_points),
      num(s.cloud_msg_points_avg),
      num(s.cloud_msg_points_max),
      num(s.cloud_queue_delay_ms),
      num(s.cloud_convert_time_ms),
      num(s.valid_cloud_count),
      num(s.dropped_cloud_empty_count),
      num(s.dropped_cloud_no_odom_count),
      num(s.dropped_cloud_odom_timeout_count),
      num(s.odom_hz),
      num(s.odom_age_ms),
      num(s.input_point_count),
      num(s.raycast_input_point_count),
      num(s.raycast_used_point_count),
      num(s.raycast_skipped_near_count),
      num(s.raycast_skipped_far_count),
      num(s.raycast_skipped_outside_count),
      num(s.total_update_time),
      num(s.update_robot_state_time),
      num(s.prob_update_time),
      num(s.raycast_time),
      num(s.raycast_parallel_time),
      num(s.raycast_merge_time),
      num(s.inflation_time),
      num(s.decay_time),
      num(s.refresh_layers_time),
      num(s.performance_csv_write_time),
      num(s.projection_total_time),
      num(s.projection_config_time),
      num(s.projection_update_full_time),
      num(s.projection_update_dirty_time),
      num(s.projection_hole_fill_time),
      num(s.projection_value_mask_time),
      sanitize(s.projection_refresh_reason),
      num(s.projection_cell_count),
      num(s.projection_z_layers),
      num(s.projection_scanned_voxel_estimate),
      num(s.dirty_column_count),
      num(s.dirty_expanded_column_count),
      num(s.projection_dirty_ratio),
      num(s.full_layer_refresh_count),
      num(s.dirty_layer_update_count),
      num(s.projection_no_update_count),
      num(s.projection_thin_surface_count),
      num(s.projection_vertical_wall_count),
      num(s.projection_hollow_tunnel_count),
      num(s.projection_ambiguous_occupied_count),
      num(s.projection_empty_column_count),
      num(s.projection_insufficient_observation_count),
      boolean(s.layer_mask_changed),
      num(s.layer_mask_diff_count),
      num(s.layer_mask_diff_ratio),
      num(s.layer_mask_free_count),
      num(s.layer_mask_occupied_count),
      num(s.free_count),
      num(s.passable_count),
      num(s.occupied_count),
      num(s.unknown_count),
      boolean(s.field_enabled),
      boolean(s.field_dirty_before),
      boolean(s.field_period_ready),
      boolean(s.field_should_update),
      boolean(s.field_actual_update),
      sanitize(s.field_skip_reason),
      num(s.field_time),
      num(s.field_edt_positive_time),
      num(s.field_edt_negative_time),
      num(s.field_distance_fill_time),
      num(s.field_copy_time),
      num(s.field_sequence),
      num(s.field_sequence_delta),
      num(s.field_update_count),
      num(s.field_skipped_count),
      num(s.field_skip_not_dirty_count),
      num(s.field_skip_period_not_ready_count),
      num(s.field_skip_layer_empty_count),
      num(s.field_skip_disabled_count),
      boolean(s.field_stale),
      num(s.field_age_ms),
      num(s.field_update_interval_ms),
      num(s.field_update_hz_window),
      num(s.query_refresh_time),
      num(s.query_snapshot_alloc_time),
      num(s.query_copy_values_time),
      num(s.query_copy_types_height_delta_confidence_time),
      num(s.query_copy_field_distances_time),
      num(s.query_update_pointer_time),
      num(s.query_sequence),
      num(s.query_field_sequence),
      boolean(s.query_field_stale),
      num(s.query_field_age_ms),
      num(s.query_distance_size),
      num(s.cpu_thread_hint > 0.0 ? s.cpu_thread_hint : hw)});
  flushIfNeeded(detailed_csv_, detailed_csv_rows_);
}

void PerformanceMonitor::maybeWriteSummary(double stamp)
{
  if (window_.start_stamp <= 0.0) {
    resetWindow(stamp);
    return;
  }
  const double period = config_.summary_rate > 0.0 ? 1.0 / config_.summary_rate : 1.0;
  const double duration = std::max(0.0, stamp - window_.start_stamp);
  if (duration + 1.0e-9 < period) {
    return;
  }
  const double updates = std::max(1.0, window_.update_count);
  const double cloud_hz = duration > 1.0e-6 ? window_.cloud_callbacks / duration : 0.0;
  const double valid_hz = duration > 1.0e-6 ? window_.valid_updates / duration : 0.0;
  const double field_hz = duration > 1.0e-6 ? window_.field_updates / duration : 0.0;

  if (summaryCsvEnabled() && summary_csv_.is_open()) {
    writeCsvLine(summary_csv_,
      {num(window_.start_stamp),
        num(duration),
        num(cloud_hz),
        num(valid_hz),
        num(field_hz),
        num(window_.update_total_sum / updates),
        num(window_.update_total_max),
        num(window_.raycast_sum / updates),
        num(window_.raycast_max),
        num(window_.projection_sum / updates),
        num(window_.projection_max),
        num(window_.field_sum / updates),
        num(window_.field_max),
        num(window_.query_sum / updates),
        num(window_.query_max),
        num(window_.full_refresh_delta),
        num(window_.dirty_update_delta),
        num(window_.field_updates),
        num(window_.field_skip_delta),
        num(window_.field_skip_not_dirty_delta),
        num(window_.field_skip_period_not_ready_delta),
        num(window_.mask_changed_delta),
        num(window_.mask_diff_ratio_sum / updates),
        num(window_.dirty_ratio_sum / updates),
        num(window_.input_points_sum / updates),
        num(window_.input_points_max)});
    flushIfNeeded(summary_csv_, summary_csv_rows_);
  }

  if (printEnabled()) {
    std::cerr << "[ROGMapPerf] win=" << std::fixed << std::setprecision(2) << duration
              << "s cloud_cb=" << cloud_hz << "Hz valid_update=" << valid_hz
              << "Hz field_update=" << field_hz << "Hz\n"
              << "  time_ms avg/max: total=" << window_.update_total_sum / updates << '/'
              << window_.update_total_max << " raycast=" << window_.raycast_sum / updates << '/'
              << window_.raycast_max << " proj=" << window_.projection_sum / updates << '/'
              << window_.projection_max << " field=" << window_.field_sum / updates << '/'
              << window_.field_max << " query=" << window_.query_sum / updates << '/' << window_.query_max
              << '\n'
              << " projection: full=" << window_.full_refresh_delta
              << " dirty=" << window_.dirty_update_delta
              << " dirty_ratio_avg=" << window_.dirty_ratio_sum / updates
              << " mask_change=" << window_.mask_changed_delta
              << " avg_mask_diff=" << window_.mask_diff_ratio_sum / updates << '\n'
              << " classification: thin=" << stats_.projection_thin_surface_count
              << " wall=" << stats_.projection_vertical_wall_count
              << " tunnel=" << stats_.projection_hollow_tunnel_count
              << " ambiguous=" << stats_.projection_ambiguous_occupied_count
              << " empty=" << stats_.projection_empty_column_count
              << " insufficient=" << stats_.projection_insufficient_observation_count << '\n'
              << " field: updates=" << window_.field_updates << " skips=" << window_.field_skip_delta
              << " skip_not_dirty=" << window_.field_skip_not_dirty_delta
              << " skip_period=" << window_.field_skip_period_not_ready_delta
              << " seq=" << stats_.field_sequence << " age_ms=" << stats_.field_age_ms << '\n'
              << " points: avg=" << window_.input_points_sum / updates
              << " max=" << window_.input_points_max << " dropped_no_odom=" << dropped_cloud_no_odom_count_
              << " dropped_timeout=" << dropped_cloud_odom_timeout_count_
              << " raycast_latest: input = " << stats_.raycast_input_point_count
              << " used=" << stats_.raycast_used_point_count << " hit=" << stats_.hit_count
              << " miss=" << stats_.miss_count << " skip_near=" << stats_.raycast_skipped_near_count
              << " skip_far=" << stats_.raycast_skipped_far_count
              << " skip_outside=" << stats_.raycast_skipped_outside_count
              << " decayed=" << stats_.decayed_count << std::endl;
  }
  resetWindow(stamp);
}

void PerformanceMonitor::resetWindow(double stamp)
{
  window_ = WindowAccumulator{};
  window_.start_stamp = stamp;
  window_.last_stamp = stamp;
}

void PerformanceMonitor::flushIfNeeded(std::ofstream & stream, int & row_count)
{
  ++row_count;
  if (config_.csv_flush_every_n > 0 && row_count % config_.csv_flush_every_n == 0) {
    stream.flush();
  }
}

std::string PerformanceMonitor::sanitize(const std::string & value) const
{
  std::string out = value;
  std::replace(out.begin(), out.end(), ',', '_');
  return out;
}

}  // namespace rog_map
