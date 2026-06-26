#ifndef MINCO_CORE__PERFORMANCE__PLANNER_PERFORMANCE_MONITOR_HPP_
#define MINCO_CORE__PERFORMANCE__PLANNER_PERFORMANCE_MONITOR_HPP_

#include <chrono>
#include <fstream>
#include <limits>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minco_planner {

struct PlannerPerformanceConfig
{
  bool enable{true};
  bool print_enable{true};
  bool detailed_csv_enable{false};
  bool odom_sub_debug_enable{true};

  std::string detailed_csv_path{"/tmp/minco_perf_detailed.csv"};

  std::string run_id;
  std::string scenario;
  std::string variant;

  double print_period_sec{1.0};
  int csv_flush_every_n{30};
};

struct PlannerPerfSample
{
  std::string planner_mode{"UNKNOWN"};
  std::string failure_reason{"NONE"};
  double stamp_ros{0.0};
  long long stamp_steady_ns{0};
  bool success{false};
  double global_search_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double local_search_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double optimizer_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double total_replan_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double planner_hz{std::numeric_limits<double>::quiet_NaN()};
};

using MincoPerfSample = PlannerPerfSample;

struct OdomSubWindowStats
{
  std::chrono::steady_clock::time_point window_start{};

  uint64_t callback_count{0};
  double delay_sum_ms{0.0};
  double delay_min_ms{std::numeric_limits<double>::infinity()};
  double delay_max_ms{0.0};
  uint64_t delay_count{0};
};

class PlannerPerformanceMonitor
{
public:
  PlannerPerformanceMonitor() = default;
  ~PlannerPerformanceMonitor();

  void configure(const PlannerPerformanceConfig & config, rclcpp::Logger logger);
  void close();

  bool enabled() const;
  bool detailedCsvEnabled() const;
  bool printEnabled() const;

  void recordPlannerSample(const PlannerPerfSample & sample);

  void recordOdomCallback(
    const rclcpp::Time & now,
    const builtin_interfaces::msg::Time & stamp);

  static long long steadyNowNs();

private:
  void writeDetailedHeader();
  void writeDetailedRow(const PlannerPerfSample & sample);

  void maybePrintOdomStats();
  void resetOdomWindow(std::chrono::steady_clock::time_point now);

  void flushIfNeeded();

  static std::string num(double value);
  static std::string boolean(bool value);
  static std::string sanitize(std::string value);

  PlannerPerformanceConfig config_{};
  rclcpp::Logger logger_{rclcpp::get_logger("PlannerPerformanceMonitor")};
  std::ofstream detailed_csv_;
  int detailed_csv_rows_{0};
  OdomSubWindowStats odom_stats_{};
};

}  // namespace minco_planner

#endif  // MINCO_CORE__PERFORMANCE__PLANNER_PERFORMANCE_MONITOR_HPP_
