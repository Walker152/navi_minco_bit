#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

namespace rog_map {

struct RuntimeStats
{
  double total_update_time{0.0};
  double raycast_time{0.0};
  double prob_update_time{0.0};
  double inflation_time{0.0};
  double raycast_parallel_time{0.0};
  double raycast_merge_time{0.0};
  double decay_time{0.0};
  double projection_time{0.0};
  double field_time{0.0};
  double query_refresh_time{0.0};
  double visualization_time{0.0};
  double input_point_count{0.0};
  double cache_count{0.0};
  double inflation_count{0.0};
  double hit_count{0.0};
  double miss_count{0.0};
  double occupied_count{0.0};
  double unknown_count{0.0};
  double passable_count{0.0};
  double free_count{0.0};
  double decayed_count{0.0};
  double dirty_column_count{0.0};
  double dirty_expanded_column_count{0.0};
  double full_layer_refresh_count{0.0};
  double dirty_layer_update_count{0.0};
  double field_skipped_count{0.0};
};

struct PerformanceConfig
{
  bool enable{true};
  bool csv_enable{false};
  std::string csv_path{"/tmp/rog_map_performance.csv"};
  std::string map_info_csv_path{"/tmp/rog_map_info.csv"};
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
  void close();

private:
  void addElapsed(double RuntimeStats::*field, double elapsed_ms);

  PerformanceConfig config_{};
  RuntimeStats stats_{};
  std::ofstream performance_csv_;
  std::ofstream map_info_csv_;
};

}  // namespace rog_map
