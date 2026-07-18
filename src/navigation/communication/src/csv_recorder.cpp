#include "csv_recorder.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>

namespace ns_com {
namespace {

constexpr const char * kLogDirectory = "/tmp/communication_logs";

std::string makeTimestamp(const std::chrono::system_clock::time_point & time, const char * format)
{
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(time);
  std::tm local_time{};
  localtime_r(&raw_time, &local_time);
  std::ostringstream out;
  out << std::put_time(&local_time, format);
  return out.str();
}

}  // namespace

std::atomic<bool> CsvRecorder::enabled_{false};

CsvRecorder::CsvRecorder() noexcept : start_time_(std::chrono::steady_clock::now())
{
  try {
    std::error_code error;
    std::filesystem::create_directories(kLogDirectory, error);
    if (error) {
      std::cerr << "[COM] Failed to create CSV log directory " << kLogDirectory << ": "
                << error.message() << std::endl;
      return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::filesystem::path path = std::filesystem::path(kLogDirectory) /
      ("sent_messages_" + makeTimestamp(now, "%Y%m%d_%H%M%S") + ".csv");
    stream_.open(path, std::ios::out | std::ios::app);
    if (!stream_.is_open()) {
      std::cerr << "[COM] Failed to open CSV log file: " << path << std::endl;
      return;
    }

    stream_ << "system_time,elapsed_ms,send_result,send_success,"
               "vx_mps,vy_mps,vw_rpm,current_yaw,current_vx,current_vy,current_vw,"
               "fx_global,fy_global,fw_global,use_speed_control,delta_yaw,"
               "odom_stamp_sec,odom_receive_stamp_sec,odom_age_ms,"
               "chassis_sample_stamp_sec,chassis_imu_yaw,history_size,history_span_ms,"
               "odom_minus_oldest_ms,odom_minus_newest_ms,best_signed_dt_ms,match_found,"
               "delta_candidate,delta_yaw_initialized,delta_last_update_age_ms,"
               "consecutive_match_failures,self_packet_count,self_packet_age_ms,"
               "offline_publish_cost_us\n";
    stream_.flush();
  } catch (const std::exception & error) {
    std::cerr << "[COM] Failed to initialize CSV recorder: " << error.what() << std::endl;
  }
}

CsvRecorder & CsvRecorder::instance() noexcept
{
  static CsvRecorder recorder;
  return recorder;
}

void CsvRecorder::initialize(bool enabled) noexcept
{
  enabled_.store(enabled, std::memory_order_relaxed);
  if (enabled) {
    (void)instance();
  }
}

void CsvRecorder::record(
  const ChassisTarget & data, int send_result, const DeltaYawDiagnostics & diagnostics) noexcept
{
  if (!enabled_.load(std::memory_order_relaxed)) {
    return;
  }
  instance().recordChassis(data, send_result, diagnostics);
}

void CsvRecorder::writePrefix(int send_result)
{
  const auto system_now = std::chrono::system_clock::now();
  const auto elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_);
  stream_ << makeTimestamp(system_now, "%Y-%m-%d %H:%M:%S") << ',' << elapsed.count() << ',' << send_result
          << ',' << (send_result == 0 ? 1 : 0) << ',';
}

void CsvRecorder::recordChassis(
  const ChassisTarget & data, int send_result, const DeltaYawDiagnostics & diagnostics) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open()) {
      return;
    }
    writePrefix(send_result);
    stream_ << std::setprecision(9) << data.vx_mps << ',' << data.vy_mps << ',' << data.vw_rpm << ','
            << data.current_yaw << ',' << data.current_vx << ',' << data.current_vy << ','
            << data.current_vw << ',' << data.fx_global << ',' << data.fy_global << ',' << data.fw_global
            << ',' << static_cast<int>(data.use_speed_control) << ',' << data.delta_yaw << ','
            << diagnostics.odom_stamp_sec << ',' << diagnostics.odom_receive_stamp_sec << ','
            << diagnostics.odom_age_ms << ',' << diagnostics.chassis_sample_stamp_sec << ','
            << diagnostics.chassis_imu_yaw << ',' << diagnostics.history_size << ','
            << diagnostics.history_span_ms << ',' << diagnostics.odom_minus_oldest_ms << ','
            << diagnostics.odom_minus_newest_ms << ',' << diagnostics.best_signed_dt_ms << ','
            << static_cast<int>(diagnostics.match_found) << ',' << diagnostics.delta_candidate << ','
            << static_cast<int>(diagnostics.delta_yaw_initialized) << ','
            << diagnostics.delta_last_update_age_ms << ',' << diagnostics.consecutive_match_failures << ','
            << diagnostics.self_packet_count << ',' << diagnostics.self_packet_age_ms << ','
            << diagnostics.offline_publish_cost_us << '\n';
    stream_.flush();
  } catch (const std::exception & error) {
    std::cerr << "[COM] Failed to record chassis CSV row: " << error.what() << std::endl;
  }
}

}  // namespace ns_com
