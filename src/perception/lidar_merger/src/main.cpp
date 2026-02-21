#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lidar_merger/lidar_merger_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMergerNode>());
  rclcpp::shutdown();
  return 0;
}
