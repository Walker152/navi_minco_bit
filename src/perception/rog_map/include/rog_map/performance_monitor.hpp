#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace rog_map {

struct RuntimeStats
{
  double stamp{0.0};
  double update_seq{0.0};
  double cloud_callback_count{0.0};
  double cloud_callback_hz{0.0};
  double cloud_msg_points{0.0};
  double cloud_msg_points_avg{0.0};
  double cloud_msg_points_max{0.0};
  double cloud_convert_time_ms{0.0};
  double cloud_queue_delay_ms{0.0};
  double valid_cloud_count{0.0};
  double valid_update_hz{0.0};
  double dropped_cloud_empty_count{0.0};
  double dropped_cloud_no_odom_count{0.0};
  double dropped_cloud_odom_timeout_count{0.0};
  double odom_received_count{0.0};
  double odom_hz{0.0};
  double odom_lookup_time_ms{0.0};
  double odom_age_ms{0.0};
  double total_update_time{0.0};
  double update_robot_state_time{0.0};
  double raycast_time{0.0};
  double prob_update_time{0.0};
  double inflation_time{0.0};
  double raycast_parallel_time{0.0};
  double raycast_merge_time{0.0};
  double decay_time{0.0};
  double projection_time{0.0};
  double field_time{0.0};
  double query_refresh_time{0.0};
  double input_point_count{0.0};
  double raycast_input_point_count{0.0};
  double raycast_used_point_count{0.0};
  double raycast_skipped_near_count{0.0};
  double raycast_skipped_far_count{0.0};
  double raycast_skipped_outside_count{0.0};
  double cache_count{0.0};
  double inflation_count{0.0};
  double hit_count{0.0};
  double miss_count{0.0};
  double active_cell_count{0.0};
  double dirty_column_count_from_probmap{0.0};
  double refresh_layers_time{0.0};
  double performance_csv_write_time{0.0};
  double projection_total_time{0.0};
  double projection_config_time{0.0};
  double projection_scanner_time{0.0};
  double projection_update_full_time{0.0};
  double projection_update_dirty_time{0.0};
  double projection_terrain_time{0.0};
  double projection_hole_fill_time{0.0};
  double projection_value_mask_time{0.0};
  double projection_count_cells_time{0.0};
  double projection_cell_count{0.0};
  double projection_z_min_id{0.0};
  double projection_z_max_id{0.0};
  double projection_z_layers{0.0};
  double projection_scanned_voxel_estimate{0.0};
  double projection_force_full_refresh{0.0};
  double projection_geometry_changed{0.0};
  double projection_full_layer_required{0.0};
  double projection_dirty_column_enabled{0.0};
  double projection_dirty_over_ratio{0.0};
  double projection_dirty_ratio{0.0};
  double projection_no_update_count{0.0};
  std::string projection_refresh_reason{"none"};
  double layer_mask_changed{0.0};
  double layer_mask_diff_count{0.0};
  double layer_mask_diff_ratio{0.0};
  double layer_mask_free_count{0.0};
  double layer_mask_occupied_count{0.0};
  double layer_value_free_count{0.0};
  double layer_value_passable_count{0.0};
  double layer_value_occupied_count{0.0};
  double layer_value_unknown_count{0.0};
  double occupied_count{0.0};
  double unknown_count{0.0};
  double passable_count{0.0};
  double free_count{0.0};
  double decayed_count{0.0};
  double dirty_column_count{0.0};
  double dirty_expanded_column_count{0.0};
  double full_layer_refresh_count{0.0};
  double dirty_layer_update_count{0.0};
  double field_enabled{0.0};
  double field_dirty_before{0.0};
  double field_period_ready{0.0};
  double field_should_update{0.0};
  double field_actual_update{0.0};
  std::string field_skip_reason{"none"};
  double field_skipped_count{0.0};
  double field_update_from_mask_time{0.0};
  double field_edt_positive_time{0.0};
  double field_inverse_mask_time{0.0};
  double field_edt_negative_time{0.0};
  double field_distance_fill_time{0.0};
  double field_copy_time{0.0};
  double field_sequence{0.0};
  double field_sequence_delta{0.0};
  double field_update_count{0.0};
  double field_skip_not_dirty_count{0.0};
  double field_skip_period_not_ready_count{0.0};
  double field_skip_layer_empty_count{0.0};
  double field_skip_disabled_count{0.0};
  double field_stale{0.0};
  double field_age_ms{0.0};
  double field_update_interval_ms{0.0};
  double field_update_hz_window{0.0};
  double query_snapshot_alloc_time{0.0};
  double query_copy_values_time{0.0};
  double query_copy_types_heights_confidence_time{0.0};
  double query_copy_field_distances_time{0.0};
  double query_update_pointer_time{0.0};
  double query_sequence{0.0};
  double query_field_sequence{0.0};
  double query_field_stale{0.0};
  double query_field_age_ms{0.0};
  double query_distance_size{0.0};
  double cpu_thread_hint{0.0};
};

struct PerformanceConfig
{
  bool enable{true};
  bool csv_enable{false};
  std::string csv_path{"/tmp/rog_map_performance.csv"};
  std::string map_info_csv_path{"/tmp/rog_map_info.csv"};
  bool detailed_enable{true};
  bool detailed_csv_enable{false};
  std::string detailed_csv_path{"/tmp/rog_map_perf_detailed.csv"};
  bool summary_csv_enable{false};
  std::string summary_csv_path{"/tmp/rog_map_perf_summary.csv"};
  int csv_flush_every_n{30};
  bool publish_enable{true};
  bool print_enable{false};
  double summary_rate{1.0};
};

class PerformanceMonitor
{
public:
  class ScopedTimer
  {
  public:
    ScopedTimer(PerformanceMonitor * monitor, double RuntimeStats::*field);
    ~ScopedTimer();
    ScopedTimer(const ScopedTimer &) = delete;
    ScopedTimer & operator=(const ScopedTimer &) = delete;

  private:
    PerformanceMonitor * monitor_{nullptr};
    double RuntimeStats::*field_{nullptr};
    std::chrono::steady_clock::time_point start_{};
  };

  void configure(const PerformanceConfig & config);
  bool enabled() const { return config_.enable; }
  bool csvEnabled() const { return config_.enable && config_.csv_enable; }
  bool detailedCsvEnabled() const { return config_.enable && config_.detailed_enable && config_.detailed_csv_enable; }
  bool summaryCsvEnabled() const { return config_.enable && config_.summary_csv_enable; }
  bool publishEnabled() const { return config_.enable && config_.publish_enable; }
  bool printEnabled() const { return config_.enable && config_.print_enable; }

  RuntimeStats & stats() { return stats_; }
  const RuntimeStats & stats() const { return stats_; }
  void resetStats() { stats_ = RuntimeStats{}; }

  ScopedTimer scoped(double RuntimeStats::*field) { return ScopedTimer(this, field); }

  std::ofstream & performanceCsv() { return performance_csv_; }
  std::ofstream & mapInfoCsv() { return map_info_csv_; }
  void writePerformanceCsvHeader(const std::vector<std::string> & fields);
  void writePerformanceCsvRow(const std::vector<double> & values);
  void recordCloudCallback(double stamp, double points, double queue_delay_ms, double convert_time_ms);
  void recordCloudConvertTime(double convert_time_ms);
  void recordCloudDropEmpty();
  void recordCloudDropNoOdom();
  void recordCloudDropOdomTimeout();
  void recordValidCloud(double odom_age_ms);
  void recordOdom(double stamp);
  void fillInputStats(RuntimeStats & stats);
  void observeUpdate(const RuntimeStats & stats);
  void close();

private:
  struct WindowAccumulator
  {
    double start_stamp{0.0};
    double last_stamp{0.0};
    double cloud_callbacks{0.0};
    double valid_updates{0.0};
    double field_updates{0.0};
    double update_count{0.0};
    double update_total_sum{0.0};
    double update_total_max{0.0};
    double raycast_sum{0.0};
    double raycast_max{0.0};
    double projection_sum{0.0};
    double projection_max{0.0};
    double field_sum{0.0};
    double field_max{0.0};
    double query_sum{0.0};
    double query_max{0.0};
    double full_refresh_delta{0.0};
    double dirty_update_delta{0.0};
    double field_skip_delta{0.0};
    double field_skip_not_dirty_delta{0.0};
    double field_skip_period_not_ready_delta{0.0};
    double mask_changed_delta{0.0};
    double mask_diff_ratio_sum{0.0};
    double dirty_ratio_sum{0.0};
    double input_points_sum{0.0};
    double input_points_max{0.0};
  };

  void addElapsed(double RuntimeStats::*field, double elapsed_ms);
  void writeDetailedHeader();
  void writeSummaryHeader();
  void writeDetailedRow(const RuntimeStats & stats);
  void maybeWriteSummary(double stamp);
  void resetWindow(double stamp);
  void flushIfNeeded(std::ofstream & stream, int & row_count);
  std::string sanitize(const std::string & value) const;

  PerformanceConfig config_{};
  RuntimeStats stats_{};
  std::ofstream performance_csv_;
  std::ofstream map_info_csv_;
  std::ofstream detailed_csv_;
  std::ofstream summary_csv_;
  int performance_csv_rows_{0};
  int detailed_csv_rows_{0};
  int summary_csv_rows_{0};
  double first_cloud_stamp_{0.0};
  double last_cloud_stamp_{0.0};
  double cloud_callback_count_{0.0};
  double cloud_points_sum_{0.0};
  double cloud_points_max_{0.0};
  double last_cloud_points_{0.0};
  double last_cloud_queue_delay_ms_{0.0};
  double last_cloud_convert_time_ms_{0.0};
  double valid_cloud_count_{0.0};
  double dropped_cloud_empty_count_{0.0};
  double dropped_cloud_no_odom_count_{0.0};
  double dropped_cloud_odom_timeout_count_{0.0};
  double first_odom_stamp_{0.0};
  double last_odom_stamp_{0.0};
  double odom_received_count_{0.0};
  double last_odom_age_ms_{0.0};
  double first_valid_update_stamp_{0.0};
  double last_valid_update_stamp_{0.0};
  WindowAccumulator window_{};
  std::mutex mutex_;
};

}  // namespace rog_map
