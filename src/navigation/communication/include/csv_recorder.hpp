#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>

#include "utils/custom_protocol.hpp"

namespace ns_com {

struct DeltaYawDiagnostics
{
  double odom_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double odom_receive_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double odom_age_ms{std::numeric_limits<double>::quiet_NaN()};
  double chassis_sample_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double chassis_imu_yaw{std::numeric_limits<double>::quiet_NaN()};
  uint64_t history_size{0};
  double history_span_ms{0.0};
  double odom_minus_oldest_ms{std::numeric_limits<double>::quiet_NaN()};
  double odom_minus_newest_ms{std::numeric_limits<double>::quiet_NaN()};
  double best_signed_dt_ms{std::numeric_limits<double>::quiet_NaN()};
  bool match_found{false};
  double delta_candidate{std::numeric_limits<double>::quiet_NaN()};
  bool delta_yaw_initialized{false};
  double delta_last_update_age_ms{std::numeric_limits<double>::quiet_NaN()};
  uint64_t consecutive_match_failures{0};
  uint64_t self_packet_count{0};
  double self_packet_age_ms{std::numeric_limits<double>::quiet_NaN()};
  double offline_publish_cost_us{std::numeric_limits<double>::quiet_NaN()};
};

class CsvRecorder
{
public:
  static void initialize(bool enabled) noexcept;
  static void record(
    const ChassisTarget & data, int send_result, const DeltaYawDiagnostics & diagnostics) noexcept;

private:
  CsvRecorder() noexcept;

  static CsvRecorder & instance() noexcept;
  void recordChassis(
    const ChassisTarget & data, int send_result, const DeltaYawDiagnostics & diagnostics) noexcept;
  void writePrefix(int send_result);

  static std::atomic<bool> enabled_;
  std::ofstream stream_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace ns_com
