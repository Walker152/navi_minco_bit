#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <nav2_msgs/srv/load_map.hpp>

namespace Sentry_BT
{

class ParamManager
{
public:
  explicit ParamManager(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planner_server_name = "/planner_server");

  bool changeMapAndPcd(
    const std::string & yaml_path,
    const std::string & pcd_path,
    const std::string & planner_plugin_name = "MincoPlanner");

private:
  rclcpp::Node::SharedPtr node_;
  std::string planner_server_name_;

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_param_client_;
};

}  // namespace Sentry_BT
