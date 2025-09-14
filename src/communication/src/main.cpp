#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fmt/format.h>
#include <plog/Appenders/RollingFileAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <thread>
#include "../include/utils.h"
#include "../include/behavier_tree_com.h"
#include "../RM_encoder/communication.hpp"
#define STM32 "/dev/ttyACM0" // STM32串口设备路径宏定义

// ------------------------------------
// 日志模块ID 枚举，用于区分不同日志通道
// ------------------------------------
enum LoggerID
{
  kSTM32 = 10,       // 用于STM32相关日志
  kSTM32ErrMsg = 101 // 用于STM32错误信息日志
};

// -------------------------
// 主函数入口，ROS节点初始化及通信调度
// -------------------------
int main(int argc, char **argv)
{
  // ---- ROS节点初始化 ----
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto node = rclcpp::Node::make_shared("serial_test");

  // ---- 获取单例通信处理实例 ----
  // 应用层通信实例化、初始化
  auto &bcom = BehavierTreeCom::getInstance();
  bcom.Init();
  // 数据链路层实例化、初始化
  Communication com;
  com.init();

  // ---- 日志记录，debug用 ----

  // 获取当前ROS时间，转成系统时间
  std::time_t now_time_t = rclcpp::Clock().now().seconds(); // ROS2时间访问
  std::tm *now_tm = std::localtime(&now_time_t);

  // 使用fmt库格式化当前时间，生成目录和日志文件名
  std::string formatted_time =
      fmt::format("{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}",
                  now_tm->tm_year + 1900, now_tm->tm_mon + 1,
                  now_tm->tm_mday, now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec);

  // 创建日志目录，权限设为0777（所有用户可读写执行）
  std::string directory_path = fmt::format("./log/{}", formatted_time);
  mkdir(directory_path.c_str(), 0777);

  // 创建两个循环文件日志追加器（分别记录不同日志）
  // 1) stm32Appender 用于保存来自STM32的正常日志，最大文件大小1MB
  static plog::RollingFileAppender<plog::CsvFormatter> stm32Appender(
      fmt::format("./log/{}/stm32.csv", formatted_time).c_str(),
      1024 * 1024, 0xFFFF);

  // 2) stm32ErrMsgAppender 用于保存STM32的错误信息日志，文件大小限制为8KB
  static plog::RollingFileAppender<plog::CsvFormatter> stm32ErrMsgAppender(
      fmt::format("./log/{}/stm32ErrMsg.csv", formatted_time).c_str(),
      8000, 0xFFFF);

  // plog日志初始化，设置日志级别为 verbose，用于kSTM32ErrMsg分类
  plog::init<kSTM32ErrMsg>(plog::verbose, &stm32ErrMsgAppender);

  // TODO标记，未来考虑将主循环拆分为多线程，提高性能
  int count = 0; // 计数变量，暂未使用

  // 定义一个循环频率 (500 Hz)
  rclcpp::Rate loop_rate(50);

  while (rclcpp::ok())
  {
    // count += 1;
    // ChassisTarget target(0.2, 0.2, 0, 0, 0, 0);
    // Communication::send2stm32(target); // 发送速度控制命令到STM32
    // if (count%20==0)
    // {
    //   std::cout << "success_send!" << count << std::endl;
    //   /* code */
    // }

    loop_rate.sleep();
    // 通过单例获取的节点本体，传给spin，使该节点进入消息循环
    rclcpp::spin(std::shared_ptr<BehavierTreeCom>(&bcom, [](BehavierTreeCom *) {}));
  }

  std::cout << "程序已结束" << std::endl;

  rclcpp::shutdown();

  return 0;
}