#include "minco_controller/performance/mpc_performance_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace minco_controller {

MpcPerformanceMonitor::~MpcPerformanceMonitor()
{
  close();
}

void MpcPerformanceMonitor::configure(const MpcPerformanceConfig & config, rclcpp::Logger logger)
{
  close();
  config_ = config;
  logger_ = logger;
  detailed_csv_rows_ = 0;
  resetOdomWindow(std::chrono::steady_clock::now());

  if (!enabled() || !detailedCsvEnabled()) {
    return;
  }

  detailed_csv_.open(config_.detailed_csv_path, std::ios::out | std::ios::trunc);
  if (!detailed_csv_.is_open()) {
    RCLCPP_ERROR(logger_, "[MincoMpc] Failed to open MPC perf CSV: %s", config_.detailed_csv_path.c_str());
    config_.detailed_csv_enable = false;
    return;
  }

  writeDetailedHeader();
}

void MpcPerformanceMonitor::close()
{
  if (detailed_csv_.is_open()) {
    detailed_csv_.flush();
    detailed_csv_.close();
  }
}

bool MpcPerformanceMonitor::enabled() const
{
  return config_.enable;
}

bool MpcPerformanceMonitor::detailedCsvEnabled() const
{
  return config_.enable && config_.detailed_csv_enable;
}

bool MpcPerformanceMonitor::printEnabled() const
{
  return config_.enable && config_.print_enable;
}

void MpcPerformanceMonitor::recordMpcSample(const MpcPerfSample & sample)
{
  if (!detailedCsvEnabled() || !detailed_csv_.is_open()) {
    return;
  }

  writeDetailedRow(sample);
  flushIfNeeded();
}

void MpcPerformanceMonitor::recordOdomCallback(
  const rclcpp::Time & now,
  const builtin_interfaces::msg::Time & stamp)
{
  if (!enabled() || !config_.odom_sub_debug_enable) {
    return;
  }

  if (odom_stats_.window_start == std::chrono::steady_clock::time_point{}) {
    resetOdomWindow(std::chrono::steady_clock::now());
  }

  ++odom_stats_.callback_count;

  const rclcpp::Time stamp_time(stamp);
  const double delay_ms = (now - stamp_time).seconds() * 1000.0;
  if (std::isfinite(delay_ms)) {
    odom_stats_.delay_sum_ms += delay_ms;
    odom_stats_.delay_min_ms = std::min(odom_stats_.delay_min_ms, delay_ms);
    odom_stats_.delay_max_ms = std::max(odom_stats_.delay_max_ms, delay_ms);
    ++odom_stats_.delay_count;
  }

  maybePrintOdomStats();
}

long long MpcPerformanceMonitor::steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void MpcPerformanceMonitor::writeDetailedHeader()
{
  if (!detailed_csv_.is_open()) {
    return;
  }

  detailed_csv_
    << "run_id,scenario,variant,stamp_ros,stamp_steady_ns,success,solve_time_ms,cycle_time_ms,"
       "controller_hz,cmd_vx,cmd_vy,cmd_wz,ref_vx,ref_vy,ref_wz\n";
}

void MpcPerformanceMonitor::writeDetailedRow(const MpcPerfSample & sample)
{
  detailed_csv_ << sanitize(config_.run_id) << ',' << sanitize(config_.scenario) << ','
                << sanitize(config_.variant) << ',' << num(sample.stamp_ros) << ','
                << sample.stamp_steady_ns << ',' << boolean(sample.success) << ','
                << num(sample.solve_time_ms) << ',' << num(sample.cycle_time_ms) << ','
                << num(sample.controller_hz) << ',' << num(sample.cmd_vx) << ','
                << num(sample.cmd_vy) << ',' << num(sample.cmd_wz) << ',' << num(sample.ref_vx) << ','
                << num(sample.ref_vy) << ',' << num(sample.ref_wz) << '\n';
}

void MpcPerformanceMonitor::maybePrintOdomStats()
{
  if (!printEnabled()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double window_sec = std::chrono::duration<double>(now - odom_stats_.window_start).count();
  const double print_period_sec = std::max(config_.print_period_sec, 1.0e-6);
  if (window_sec < print_period_sec) {
    return;
  }

  const double callback_hz = window_sec > 1.0e-9
                               ? static_cast<double>(odom_stats_.callback_count) / window_sec
                               : 0.0;
  const double avg_delay_ms = odom_stats_.delay_count > 0
                                ? odom_stats_.delay_sum_ms / static_cast<double>(odom_stats_.delay_count)
                                : std::numeric_limits<double>::quiet_NaN();
  const double min_delay_ms = odom_stats_.delay_count > 0
                                ? odom_stats_.delay_min_ms
                                : std::numeric_limits<double>::quiet_NaN();
  const double max_delay_ms = odom_stats_.delay_count > 0
                                ? odom_stats_.delay_max_ms
                                : std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(
    logger_,
    "[MincoMpc][OdomSub] callback_hz=%.3f delay_ms avg=%.3f min=%.3f max=%.3f count=%lu",
    callback_hz,
    avg_delay_ms,
    min_delay_ms,
    max_delay_ms,
    static_cast<unsigned long>(odom_stats_.callback_count));

  resetOdomWindow(now);
}

void MpcPerformanceMonitor::resetOdomWindow(std::chrono::steady_clock::time_point now)
{
  odom_stats_.window_start = now;
  odom_stats_.callback_count = 0;
  odom_stats_.delay_sum_ms = 0.0;
  odom_stats_.delay_min_ms = std::numeric_limits<double>::infinity();
  odom_stats_.delay_max_ms = 0.0;
  odom_stats_.delay_count = 0;
}

void MpcPerformanceMonitor::flushIfNeeded()
{
  const int flush_every_n = std::max(config_.csv_flush_every_n, 1);
  if (++detailed_csv_rows_ % flush_every_n == 0) {
    detailed_csv_.flush();
  }
}

std::string MpcPerformanceMonitor::num(double value)
{
  if (!std::isfinite(value)) {
    return "NaN";
  }

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << value;
  return ss.str();
}

std::string MpcPerformanceMonitor::boolean(bool value)
{
  return value ? "1" : "0";
}

std::string MpcPerformanceMonitor::sanitize(std::string value)
{
  std::replace(value.begin(), value.end(), ',', '_');
  return value;
}

}  // namespace minco_controller
