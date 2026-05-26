#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace Sentry_BT {

class AreaVisualizer
{
public:
  explicit AreaVisualizer(rclcpp::Node & node);

  void publishAreaMarkers(const rclcpp::Time & now);
  void clearAreaMarkers(const rclcpp::Time & now);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_marker_pub_;
};

}  // namespace Sentry_BT
