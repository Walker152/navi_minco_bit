#pragma once

#include <chrono>
#include <fstream>
#include <mutex>

#include "utils/custom_protocol.hpp"

namespace ns_com {

class CsvRecorder
{
public:
  static void initialize() noexcept;
  static void record(
    const ChassisTarget & data, PacketTypeEnum packet_type, int send_result) noexcept;
  static void record(
    const BehaviorData & data, PacketTypeEnum packet_type, int send_result) noexcept;

  template <typename T>
  static void record(const T &, PacketTypeEnum, int) noexcept
  {
    // Field recording is intentionally limited to packet types that are actively sent.
  }

private:
  CsvRecorder() noexcept;

  static CsvRecorder & instance() noexcept;
  void recordChassis(
    const ChassisTarget & data, PacketTypeEnum packet_type, int send_result) noexcept;
  void recordBehavior(
    const BehaviorData & data, PacketTypeEnum packet_type, int send_result) noexcept;
  void writePrefix(PacketTypeEnum packet_type, int send_result);

  std::ofstream stream_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace ns_com
