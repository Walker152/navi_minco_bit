#pragma once

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <string>
#include <string_view>
#include <thread>

#include "MyUtils/DataType/ByteArray.hpp"
#include "MyUtils/Net/FdManager.hpp"
#include "MyUtils/Net/SerialPort.hpp"
#include "MyUtils/Thread/ThreadManager.hpp"
#include "MyUtils/Timer/Timer.hpp"

#include "utils/custom_protocol.hpp"
#include "utils/protocol.hpp"
#include "utils/color_text.hpp"

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
namespace ns_com { class ComInterfaceRos; }

#define STM32_NAME "stm32"

namespace ns_com
{

  using ByteArray = MyUtils::DataType::ByteArray;

  template<typename T>
  struct PacketTraits;

  namespace log {
    inline std::string now_string() {
      using namespace std::chrono;
      const auto now = system_clock::now();
      const auto now_t = system_clock::to_time_t(now);
      std::tm tm_buf{};
    #if defined(_WIN32)
      localtime_s(&tm_buf, &now_t);
    #else
      localtime_r(&now_t, &tm_buf);
    #endif
      const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
      std::ostringstream ts;
      ts << std::setfill('0')
         << std::setw(2) << tm_buf.tm_hour << ':'
         << std::setw(2) << tm_buf.tm_min  << ':'
         << std::setw(2) << tm_buf.tm_sec  << '.'
         << std::setw(3) << ms.count();
      return ts.str();
    }
    template <typename T>
    struct named_value {
      std::string_view name;
      const T* ptr;
    };

    template <typename T>
    constexpr named_value<T> nv(std::string_view name, const T& v) {
      return {name, &v};
    }

    void log_info_line(std::string_view text);

    template <typename... NVs>
    inline void log_info(std::string prefix, const NVs&... nvs) {
      std::ostringstream oss;
      oss << std::boolalpha;
      ((oss << prefix << nvs.name << " = " << *(nvs.ptr) << ::color_text::RESET << '\n'), ...);
      log_info_line(oss.str());
    }

    // 带分割线与列对齐的块状调试输出
    template <typename... NVs>
    inline void log_block(std::string prefix, const NVs&... nvs) {
      // 计算最大键名宽度
      std::size_t maxw = 0;
      (void)std::initializer_list<int>{ (maxw = std::max<std::size_t>(maxw, nvs.name.size()), 0)... };

      // 构造分割线（至少 50 列）
      const std::size_t bar_len = std::max<std::size_t>(50, maxw + 10);
      std::string bar(bar_len, '-');

      std::ostringstream oss;
      oss << std::boolalpha;
      // 时间戳 + 顶部分割线
      oss << prefix << '[' << now_string() << "] " << bar << ::color_text::RESET << '\n';
      // 对齐的 name : value
      ((oss << prefix << std::left << std::setw(static_cast<int>(maxw)) << nvs.name
            << " : " << *(nvs.ptr) << ::color_text::RESET << '\n'), ...);
      // 底部分割线
      oss << prefix << bar << ::color_text::RESET << '\n';

      log_info_line(oss.str());
    }
  } // namespace log

  #define NV(var) ::ns_com::log::nv(#var, (var))

  #ifdef COM_DEBUG
    #define LOG_DEBUG(prefix, ...) ::ns_com::log::log_info((prefix), __VA_ARGS__)
    #define LOG_DEBUG_BLOCK(prefix, ...) ::ns_com::log::log_block((prefix), __VA_ARGS__)
  #else
    #define LOG_DEBUG(prefix, ...) do { } while(0)
    #define LOG_DEBUG_BLOCK(prefix, ...) do { } while(0)
  #endif

  class Communication
  {
  public:
    // 初始化通信模块
    static void init();

    // 发送底盘目标数据包到STM32
    template<typename T>
    static int send2stm32(const T& data_packet) {
      PacketHeader header(ENUM_PACKET_ARMOR_DATA, sizeof(PacketHeader) + sizeof(T));
      header.packet_type = static_cast<uint8_t>(PacketTraits<T>::packet_type);
      header.start1 = 0xa5;
      header.start2 = 0x5a;
      header.from = static_cast<uint8_t>(ArmEnum::ENUM_ARM_SENTRY);
      header.setTo(ArmEnum::ENUM_ARM_SLAVE_COMPUTER);
      header.setDataLen(sizeof(T));
      
      return __send2stm32(header, &data_packet);
    }
    // 设置 ROS 接口（由上层注入），使本模块不直接依赖 rclcpp 细节
    static void setRosInterface(const std::shared_ptr<ComInterfaceRos> &iface);

  private:
    // fd管理器
    static MyUtils::Net::FdManager fd_manager;
    static std::atomic<uint16_t> arm_seq_num;
    static uint16_t puncture_seq_num;
    static MyUtils::MyTimer::TimerManager timer_manager;
    static std::string STM32_PORT;

    static void __open(const std::string& name,
                       const std::string& port,
                       MyUtils::Net::fd_read_cb read_cb,
                       int baud_rate = 9600,
                       int n_bits = 8,
                       int stop_length = 1,
                       char check_type = 'N');

    // 实际发送
    static int __send2stm32(PacketHeader& header, const void* data);

    // STM32串口数据读取回调
    static void stm32_read_cb(ByteArray arr);

    // ROS 接口指针
    static std::shared_ptr<ComInterfaceRos> ros_if_;

  };

  // 数据包类型映射：默认不支持，为每个可发送类型提供特化。
  template<typename T>
  struct PacketTraits {
    static_assert(sizeof(T) == 0, "PacketTraits<T> 未特化，无法确定 packet_type");
    static constexpr uint8_t packet_type = 0;
  };

  template<>
  struct PacketTraits<ChassisTarget> {
    static constexpr uint8_t packet_type = static_cast<uint8_t>(ENUM_PACKET_NAV_DATA);
  };

}  // namespace ns_com