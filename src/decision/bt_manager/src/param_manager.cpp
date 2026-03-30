#include "bt_manager/param_manager.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace Sentry_BT
{

using namespace std::chrono_literals;

ParamManager::ParamManager(
  const rclcpp::Node::SharedPtr & node,
  const std::string & planner_server_name)
: node_(node),
  planner_server_name_(planner_server_name)
{
  if (!node_) {
    throw std::invalid_argument("ParamManager: node is null");
  }

  load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
  planner_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, planner_server_name_);
}

bool ParamManager::changeMapAndPcd(
  const std::string & yaml_path,
  const std::string & pcd_path,
  const std::string & planner_plugin_name)
{
  if (!load_map_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ParamManager] /map_server/load_map service is unavailable.");
    return false;
  }

  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = yaml_path;

  load_map_client_->async_send_request(
    req,
    [logger = node_->get_logger(), yaml_path](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
      try {
        const auto resp = future.get();
        if (!resp) {
          RCLCPP_WARN(logger, "[ParamManager] LoadMap response is null: %s", yaml_path.c_str());
          return;
        }

        if (resp->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
          RCLCPP_ERROR(
            logger,
            "[ParamManager] Failed to load map yaml (%u): %s",
            static_cast<unsigned int>(resp->result),
            yaml_path.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "[ParamManager] LoadMap callback exception: %s", e.what());
      }
    });

  const std::string param_name = planner_plugin_name + ".static_esdf.esdf_pcd_path";
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(param_name, pcd_path);

  planner_param_client_->set_parameters(
    params,
    [logger = node_->get_logger(), param_name, pcd_path](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      try {
        const auto results = future.get();
        if (results.empty()) {
          RCLCPP_WARN(
            logger,
            "[ParamManager] planner_server set_parameters returned empty result: %s",
            param_name.c_str());
          return;
        }

        for (const auto & result : results) {
          if (!result.successful) {
            RCLCPP_ERROR(
              logger,
              "[ParamManager] Failed to set parameter %s -> %s, reason: %s",
              param_name.c_str(),
              pcd_path.c_str(),
              result.reason.c_str());
            return;
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "[ParamManager] set_parameters callback exception: %s", e.what());
      }
    });

  return true;
}

}  // namespace Sentry_BT
