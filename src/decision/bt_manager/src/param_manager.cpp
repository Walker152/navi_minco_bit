#include "bt_manager/param_manager.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

namespace Sentry_BT {

using namespace std::chrono_literals;

ParamManager::ParamManager(const rclcpp::Node::SharedPtr & node) : node_(node)
{
  if (!node_) {
    throw std::invalid_argument("ParamManager: node is null");
  }

  load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
}

bool ParamManager::changeMap(const std::string & yaml_path)
{
  if (!load_map_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "[ParamManager] /map_server/load_map service is unavailable.");
    return false;
  }

  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = yaml_path;

  load_map_client_->async_send_request(req,
    [logger = node_->get_logger(), yaml_path](
      rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
      try {
        const auto resp = future.get();
        if (!resp) {
          RCLCPP_WARN(logger, "[ParamManager] LoadMap response is null: %s", yaml_path.c_str());
          return;
        }

        if (resp->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
          RCLCPP_ERROR(logger,
            "[ParamManager] Failed to load map yaml (%u): %s",
            static_cast<unsigned int>(resp->result),
            yaml_path.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger, "[ParamManager] LoadMap callback exception: %s", e.what());
      }
    });

  return true;
}

}  // namespace Sentry_BT
