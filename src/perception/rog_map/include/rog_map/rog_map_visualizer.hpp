#pragma once

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rog_map/rog_map_core/config.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rog_map {

class ROGMapVisualizer
{
public:
  using Callback = std::function<void()>;

  struct Publishers
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub, raw_occ_pub, unknown_pub,
      esdf_neg_pub, esdf_occ_pub, occ_inf_pub, unknown_inf_pub, frontier_pub, esdf_pub,
      layer_height_delta_pub, field_pub, decay_cells_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr layer_value_pub,
      layer_value_dynamic_pub, layer_value_static_pub, layer_type_pub, layer_confidence_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_arr_pub;
  };

  ROGMapVisualizer() = default;

  void configure(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & topics,
    const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr & timers,
    const Config & cfg,
    Callback callback);

  void reset();
  const Publishers & publishers() const { return pubs_; }

private:
  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createPublisher(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics,
    const std::string & topic,
    const rclcpp::QoS & qos)
  {
    return rclcpp::create_publisher<MsgT>(parameters, topics, topic, qos);
  }

  Publishers pubs_{};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rog_map
