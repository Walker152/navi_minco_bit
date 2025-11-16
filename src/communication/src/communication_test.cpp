#include "com_interface_ros.hpp"
#include "com.hpp"

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

struct TestState {
  std::atomic<bool> nav_received{false};
  std::atomic<bool> evt_received{false};
  ros_interfaces::msg::Nav last_nav;
  ros_interfaces::msg::EventStatus last_evt;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // 被测节点
  auto com_node = std::make_shared<ns_com::ComInterfaceRos>("communication_test_node");
  com_node->bindCommunication();

  // 测试节点
  auto test_node = std::make_shared<rclcpp::Node>("communication_test_client");
  TestState state;

  // 订阅被测节点的发布
  auto nav_sub = test_node->create_subscription<ros_interfaces::msg::Nav>(
    "/NavRequest", 10,
    [&state](ros_interfaces::msg::Nav::ConstSharedPtr msg){ state.nav_received = true; state.last_nav = *msg; }
  );
  auto evt_sub = test_node->create_subscription<ros_interfaces::msg::EventStatus>(
    "/sentry/event_status", 10,
    [&state](ros_interfaces::msg::EventStatus::ConstSharedPtr msg){ state.evt_received = true; state.last_evt = *msg; }
  );

  // 发布控制指令到被测节点的订阅
  auto cmd_pub = test_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto odom_pub = test_node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  auto yaw_pub = test_node->create_publisher<std_msgs::msg::Float32>("/gimbal_yaw", 10);

  exec->add_node(com_node);
  exec->add_node(test_node);

  // 以线程方式运行 executor
  std::thread spin_th([&](){ exec->spin(); });

  // 1) 测试 ROS->发送：发布一些输入，触发 send2stm32（无法直接观测串口发送，仅做冒烟）
  geometry_msgs::msg::Twist cmd; cmd.linear.x = 0.6; cmd.linear.y = 0.2; cmd.angular.z = 0.0; // 触发非零速度
  nav_msgs::msg::Odometry odom; odom.pose.pose.orientation.w = 1.0; // 零姿态
  std_msgs::msg::Float32 yaw; yaw.data = 0.0f;

  odom_pub->publish(odom);
  yaw_pub->publish(yaw);
  cmd_pub->publish(cmd);

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(test_node->get_logger(), "Smoke: published cmd/odom/yaw to drive send2stm32 path");

  // 退出
  exec->remove_node(test_node);
  exec->remove_node(com_node);
  ns_com::Communication::setRosInterface(std::shared_ptr<ns_com::ComInterfaceRos>{});
  exec->cancel();
  if (spin_th.joinable()) spin_th.join();

  rclcpp::shutdown();
  return 0;
}
