#include "../include/utils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/behavier_tree_com.h"
#include <fmt/format.h>
#include <plog/Appenders/RollingFileAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <thread>
#include <../RM_encoder/communication.hpp>

#define STM32 "/dev/ttyACM0"

enum LoggerID
{
  kSTM32 = 10,
  kSTM32ErrMsg = 101
};

// ------ main ------
int main(int argc, char **argv)
{
  // ---- ros 节点初始化 ----
  ros::init(argc, argv, "serial_test");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  // ---- 保存通信器的实例 ----
  auto &bcom = BehavierTreeCom::getInstance();
  bcom.Init();
  //   auto& ccom = Communication::getInstance();
  //   ccom.init();
  Communication com;
  com.init();

  // ---- 日志初始化 ----
  // 获取当前时间并格式化，便于区分日志
  std::time_t now_time_t = ros::Time::now().toSec();
  std::tm *now_tm = std::localtime(&now_time_t);
  std::string formatted_time =
      fmt::format("{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}", now_tm->tm_year + 1900, now_tm->tm_mon + 1,
                  now_tm->tm_mday, now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec);
  // 创建日志目录
  std::string directory_path = fmt::format("./log/{}", formatted_time);
  mkdir(directory_path.c_str(), 0777); // 设置权限为 0755
  // 创建两个不同的文件记录器，单个文件最多 8KB
  static plog::RollingFileAppender<plog::CsvFormatter> stm32Appender(
      fmt::format("./log/{}/stm32.csv", formatted_time).c_str(), 1024 * 1024, 0xFFFF);
  // 初始化两个记录器

  // 用于保存 STM32 发来的数据
  static plog::RollingFileAppender<plog::CsvFormatter> stm32ErrMsgAppender(
      fmt::format("./log/{}/stm32ErrMsg.csv", formatted_time).c_str(), 8000, 0xFFFF);
  plog::init<kSTM32ErrMsg>(plog::verbose, &stm32ErrMsgAppender);

  // TODO: 拆分线程
  int count = 0;
  while (ros::ok())
  {
    // count += 1;
    // ChassisTarget target(0.2, 0.2, 0, 0, 0, 0);
    // Communication::send2stm32(target);
    // ros::Rate speed_send = 50;
    // speed_send.sleep();
    // std::cout << "success_send!" << count << endl;
    ros::spinOnce();
  }

  std::cout << "程序已结束" << std::endl;
  return 0;
}
