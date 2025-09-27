#include "com_handle.hpp"

using BehavierTreeCom = ns_com::BehavierTreeCom;

int main(int argc, char** argv)
{
  // ---- ROS节点初始化 ----
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BehavierTreeCom>("com_node");
  rclcpp::spin(node);

  std::cout << "程序已结束\n";

  rclcpp::shutdown();

  return 0;
}
