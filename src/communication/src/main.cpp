#include "com_handle.hpp"

using BehavierTreeCom = ns_com::BehavierTreeCom;

int main(int argc, char** argv)
{
  // ---- ROS节点初始化 ----
  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor exec;
  auto node = std::make_shared<BehavierTreeCom>("communication");

  exec.add_node(node);
  exec.spin();
  std::cout << "Game Over!\n";
  rclcpp::shutdown();


  return 0;
}
