#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/load_map.hpp>

namespace Sentry_BT {

class ParamManager
{
public:
  explicit ParamManager(const rclcpp::Node::SharedPtr & node);

  bool changeMap(const std::string & yaml_path);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
};

}  // namespace Sentry_BT
