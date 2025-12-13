#include "small_gicp.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<small_gicp_relocalization::SmallGicpRelocalizationNode>(options);

  RCLCPP_INFO(node->get_logger(), "Small GICP Relocalization Node Started!");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
