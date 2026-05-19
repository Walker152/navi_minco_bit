#ifndef MINCO_PLANNER__PLANNER_PROFILER_HPP_
#define MINCO_PLANNER__PLANNER_PROFILER_HPP_

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace minco_planner {

class PlannerProfiler
{
public:
  PlannerProfiler() = default;
  ~PlannerProfiler();

  void configure(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & param_prefix,
    const rclcpp::Logger & logger);

  bool enabled() const { return enabled_; }
  bool hasActiveCycle() const;

  void beginCycle(const std::string & event_name);
  void mark(const std::string & stage_name, double duration_ms);
  void setMetric(const std::string & metric_name, double value);
  void finishCycle(bool success);
  void recordSafetyCheck(double duration_ms, bool safe);
  void close();

private:
  struct CycleData
  {
    std::string event{"none"};
    std::chrono::steady_clock::time_point start_time{};
    std::unordered_map<std::string, double> stages;
    std::unordered_map<std::string, double> metrics;
    bool active{false};
  };

  void declareAndReadParameters(const nav2_util::LifecycleNode::WeakPtr & node, const std::string & prefix);
  void writeHeaderIfNeeded();
  void writeCsvRow(const CycleData & data, bool success);
  void maybeLogSummary(const CycleData & data, bool success);
  void warnCsvFailure(const std::string & reason);
  static std::string wallTimeString();
  static double getValue(const std::unordered_map<std::string, double> & values, const std::string & key);

  mutable std::mutex mutex_;
  nav2_util::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("PlannerProfiler")};

  bool enabled_{false};
  bool log_enabled_{true};
  bool csv_enabled_{false};
  double log_period_sec_{1.0};
  std::string csv_path_{"/tmp/minco_planner_profile.csv"};

  std::ofstream csv_file_;
  bool csv_header_written_{false};
  bool csv_warned_{false};

  CycleData current_;
  std::chrono::steady_clock::time_point last_log_time_{};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNER_PROFILER_HPP_
