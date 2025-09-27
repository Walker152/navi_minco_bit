#pragma once

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "MyUtils/DataType/ByteArray.hpp"
#include "MyUtils/Net/FdManager.hpp"
#include "MyUtils/Net/SerialPort.hpp"
#include "MyUtils/Thread/ThreadManager.hpp"
#include "MyUtils/Timer/Timer.hpp"

#include "utils/custom_protocol.hpp"
#include "utils/protocol.hpp"

#include "robot_msgs/msg/nav.hpp"
#include "robot_msgs/msg/referee.hpp"
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>

#define STM32_NAME "stm32"

namespace ns_com
{

  using ByteArray = MyUtils::DataType::ByteArray;

  class Communication
  {
  public:
    // 初始化通信模块
    static void init();

    // 发送底盘目标数据包到STM32
    static int send2stm32(const _ChassisTarget& data_packet);

  private:
    // fd管理器
    static MyUtils::Net::FdManager fd_manager;
    // 线程安全的序号
    static std::atomic<uint16_t> arm_seq_num;
    static uint16_t puncture_seq_num;

    // 定时器管理
    static MyUtils::MyTimer::TimerManager timer_manager;

    // 串口端口号
    static std::string STM32_PORT;

    // 打开串口
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

    // 导航数据发布
    static void nav_publish(const NavRes* msg);

    // 裁判系统数据发布
    static void referee_publish(const RefereeInfo* msg);
  };

}  // namespace ns_com