#include "checker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_rm::Checker>("checker");
  rclcpp::Rate loop(10);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop.sleep();
  }
}
