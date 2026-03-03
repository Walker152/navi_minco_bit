#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pcd2pgm/pcd2pgm.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<pcd2pgm::PCLFiltersNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
