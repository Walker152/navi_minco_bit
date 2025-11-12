#include "bt_manager/bt_manager.hpp"
#include "bt_manager/ros_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  auto blackboard = std::make_shared<Sentry_BT::Blackboard>();
  auto ros_interface_node = std::make_shared<Sentry_BT::ros_interface>(blackboard);
  auto transform_utils_node = std::make_shared<Sentry_BT::TransformUtils>();

  // 获取BT黑板并存储ros_interface实例指针
  auto bt_blackboard = blackboard->getBTBlackboard();
  bt_blackboard->set("ros_interface", ros_interface_node);

  Sentry_BT::BTManager bt_manager;

  auto pkg_share_dir = ament_index_cpp::get_package_share_directory("bt_manager");
  std::cout << "Package share directory: " << pkg_share_dir << std::endl;
  std::string xml_file_path = pkg_share_dir + "/tree/nav_tree.xml";

  if(!bt_manager.initialize(xml_file_path, bt_blackboard))
  {
    RCLCPP_ERROR(ros_interface_node->get_logger(), "Failed to initialize BTManager with XML file: %s", xml_file_path.c_str());
    return -1;
  }

  // 使用多线程执行器并单独处理回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_interface_node);
  executor.add_node(transform_utils_node);

  // 创建单独线程处理执行器
  std::thread executor_thread([&executor]() { executor.spin(); });

  rclcpp::Rate loop_rate(10);  // 10 Hz
  while(rclcpp::ok())
  {
    bt_manager.execute();
    loop_rate.sleep();
  }

  // 清理
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}