#include "bt_manager/bt_manager.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/log.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto blackboard = std::make_shared<Sentry_BT::Blackboard>();
  auto ros_interface_node = std::make_shared<Sentry_BT::ros_interface>(blackboard);
  auto transform_utils_node = std::make_shared<Sentry_BT::TransformUtils>();

  const bool bt_debug_logs = ros_interface_node->declare_parameter<bool>("bt_debug_logs", false);
  const bool bt_debug_log_to_file =
    ros_interface_node->declare_parameter<bool>("bt_debug_log_to_file", false);
  const std::string bt_debug_log_file =
    ros_interface_node->declare_parameter<std::string>("bt_debug_log_file", "logs/bt_transition.log");

  Sentry_BT::detail::setTransitionLogEnabled(bt_debug_logs);
  if (bt_debug_log_to_file) {
    std::filesystem::path log_path(bt_debug_log_file);
    if (log_path.has_parent_path()) {
      std::filesystem::create_directories(log_path.parent_path());
    }
    Sentry_BT::detail::setTransitionLogFilePath(bt_debug_log_file);
  }
  Sentry_BT::detail::setTransitionLogFileEnabled(bt_debug_log_to_file);

  // 获取BT黑板并存储ros_interface实例指针
  auto bt_blackboard = blackboard->getBTBlackboard();
  bt_blackboard->set<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface", ros_interface_node);
  bt_blackboard->set<rclcpp::Node::SharedPtr>("node", ros_interface_node);
  bt_blackboard->set<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils", transform_utils_node);
  bt_blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(100));
  bt_blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(500));
  bt_blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(10000));

  // 使用多线程执行器并单独处理回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_interface_node);
  executor.add_node(transform_utils_node);
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  Sentry_BT::SentryBTManager bt_manager;
  auto pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_manager");
  if (!bt_manager.initialize(bt_blackboard, pkg_share_dir)) {
    RCLCPP_ERROR(ros_interface_node->get_logger(),
      "Failed to initialize SentryBTManager with tree path: %s",
      pkg_share_dir.c_str());
    return -1;
  }

  bt_manager.run(10.0);

  // 清理
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}