#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>

#include "MyUtils/DataType/ByteArray.hpp"
#include "MyUtils/Net/FdManager.hpp"
#include "MyUtils/Net/SerialPort.hpp"
#include "MyUtils/Thread/ThreadManager.hpp"
#include "MyUtils/Timer/Timer.hpp"

#include "utils/color_text.hpp"
#include "utils/custom_protocol.hpp"
#include "utils/log.hpp"
#include "utils/protocol.hpp"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
using namespace color_text;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <fmt/core.h>

// 前置声明
namespace ns_com {
class ComInterfaceRos;
}

#define STM32_NAME "stm32"

#ifdef COM_DEBUG
#define LOG_DEBUG(prefix, ...) ::com_log::log_info((prefix), __VA_ARGS__)
#define LOG_DEBUG_BLOCK(prefix, ...) ::com_log::log_block((prefix), __VA_ARGS__)
#else
#define LOG_DEBUG(prefix, ...)                                                                             \
do {                                                                                                       \
} while (0)
#define LOG_DEBUG_BLOCK(prefix, ...)                                                                       \
do {                                                                                                       \
} while (0)
#endif
namespace ns_com {

using ByteArray = MyUtils::DataType::ByteArray;

template <typename T> struct PacketTraits;
class Communication
{
public:
  // 初始化通信模块
  static void init();

  // 发送底盘目标数据包到STM32
  template <typename T> static int send2stm32(const T & data_packet)
  {
    PacketHeader header(ENUM_PACKET_NAV_DATA, sizeof(PacketHeader) + sizeof(T));
    header.packet_type = static_cast<uint8_t>(PacketTraits<T>::packet_type);
    header.start1 = 0xa5;
    header.start2 = 0x5a;
    header.from = static_cast<uint8_t>(ArmEnum::ENUM_ARM_SENTRY);
    header.setTo(ArmEnum::ENUM_ARM_SLAVE_COMPUTER);
    header.setDataLen(sizeof(T));

    return __send2stm32(header, &data_packet);
  }
  // 设置 ROS 接口（由上层注入），使本模块不直接依赖 rclcpp 细节
  static void setRosInterface(const std::shared_ptr<ComInterfaceRos> & iface);

private:
  // fd管理器
  static MyUtils::Net::FdManager fd_manager;
  static std::atomic<uint16_t> arm_seq_num;
  static uint16_t puncture_seq_num;
  static MyUtils::MyTimer::TimerManager timer_manager;
  static std::string STM32_PORT;

  static void __open(const std::string & name,
    const std::string & port,
    MyUtils::Net::fd_read_cb read_cb,
    int baud_rate = 9600,
    int n_bits = 8,
    int stop_length = 1,
    char check_type = 'N');

  // 实际发送
  static int __send2stm32(PacketHeader & header, const void * data);

  // STM32串口数据读取回调
  static void stm32_read_cb(ByteArray arr);

  // ROS 接口指针
  static std::shared_ptr<ComInterfaceRos> ros_if_;
};

// 数据包类型映射：默认不支持，为每个可发送类型提供特化。
template <typename T> struct PacketTraits
{
  static_assert(sizeof(T) == 0, "PacketTraits<T> 未特化，无法确定 packet_type");
  static constexpr uint8_t packet_type = 0;
};

template <> struct PacketTraits<ChassisTarget>
{
  static constexpr uint8_t packet_type = static_cast<uint8_t>(ENUM_PACKET_NAV_DATA);
};

template <> struct PacketTraits<GlobalPath>
{
  static constexpr uint8_t packet_type = static_cast<uint8_t>(ENUM_PACKET_GLOBAL_PATH);
};

}  // namespace ns_com