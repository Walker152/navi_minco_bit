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

const char * packetTypeName(PacketTypeEnum packet_type)
{
  switch (packet_type) {
    case ENUM_PACKET_NAV_DATA:
      return "NAV_DATA";
    case ENUM_PACKET_BEHAVIOR_DATA:
      return "BEHAVIOR_DATA";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

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

    stream_ << "system_time,elapsed_ms,packet_type,send_result,send_success,"
               "vx_mps,vy_mps,vw_rpm,current_yaw,current_vx,current_vy,current_vw,"
               "fx_global,fy_global,fw_global,use_speed_control,delta_yaw,"
               "pitch_mode,desire_stance,desire_lifter_pos,scan_yaw_min_deg,scan_yaw_max_deg,"
               "ammo_purchase_request,revive_request,remote_revive_request,remote_ammo_request,"
               "remote_health_request,use_limited_scan,not_aim_enemy,use_capacitor\n";
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

void CsvRecorder::initialize() noexcept
{
  (void)instance();
}

void CsvRecorder::record(
  const ChassisTarget & data, PacketTypeEnum packet_type, int send_result) noexcept
{
  instance().recordChassis(data, packet_type, send_result);
}

void CsvRecorder::record(
  const BehaviorData & data, PacketTypeEnum packet_type, int send_result) noexcept
{
  instance().recordBehavior(data, packet_type, send_result);
}

void CsvRecorder::writePrefix(PacketTypeEnum packet_type, int send_result)
{
  const auto system_now = std::chrono::system_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_);
  stream_ << makeTimestamp(system_now, "%Y-%m-%d %H:%M:%S") << ',' << elapsed.count() << ','
          << packetTypeName(packet_type) << ',' << send_result << ',' << (send_result == 0 ? 1 : 0)
          << ',';
}

void CsvRecorder::recordChassis(
  const ChassisTarget & data, PacketTypeEnum packet_type, int send_result) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open()) {
      return;
    }
    writePrefix(packet_type, send_result);
    stream_ << std::setprecision(9) << data.vx_mps << ',' << data.vy_mps << ',' << data.vw_rpm << ','
            << data.current_yaw << ',' << data.current_vx << ',' << data.current_vy << ','
            << data.current_vw << ',' << data.fx_global << ',' << data.fy_global << ','
            << data.fw_global << ',' << static_cast<int>(data.use_speed_control) << ',' << data.delta_yaw
            << ",,,,,,,,,,,,,\n";
    stream_.flush();
  } catch (const std::exception & error) {
    std::cerr << "[COM] Failed to record chassis CSV row: " << error.what() << std::endl;
  }
}

void CsvRecorder::recordBehavior(
  const BehaviorData & data, PacketTypeEnum packet_type, int send_result) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open()) {
      return;
    }
    writePrefix(packet_type, send_result);
    stream_ << ",,,,,,,,,,,," << static_cast<int>(data.pitch_mode) << ','
            << static_cast<int>(data.desire_stance) << ',' << static_cast<int>(data.desire_lifter_pos) << ','
            << std::setprecision(9) << data.scan_yaw_min_deg << ',' << data.scan_yaw_max_deg << ','
            << data.ammo_purchase_request << ',' << static_cast<int>(data.revive_request) << ','
            << static_cast<int>(data.remote_revive_request) << ','
            << static_cast<int>(data.remote_ammo_request) << ','
            << static_cast<int>(data.remote_health_request) << ','
            << static_cast<int>(data.use_limited_scan) << ',' << static_cast<int>(data.not_aim_enemy) << ','
            << static_cast<int>(data.use_capacitor) << '\n';
    stream_.flush();
  } catch (const std::exception & error) {
    std::cerr << "[COM] Failed to record behavior CSV row: " << error.what() << std::endl;
  }
}

}  // namespace ns_com
