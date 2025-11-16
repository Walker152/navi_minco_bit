#include "com_interface_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor exec;
  auto node = std::make_shared<ns_com::ComInterfaceRos>("communication");
  node->bindCommunication();
  exec.add_node(node);
  exec.spin();

  // 先清理 Communication 中持有的 ROS 接口引用，确保节点可以真正释放
  ns_com::Communication::setRosInterface(std::shared_ptr<ns_com::ComInterfaceRos>());
  exec.remove_node(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
