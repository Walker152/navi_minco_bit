#include <rog_map/rog_map_visualizer.hpp>

#include <algorithm>

namespace rog_map {

void ROGMapVisualizer::configure(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & topics,
  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr & timers,
  const Config & cfg,
  Callback callback)
{
  reset();
  if (!cfg.visualization_en || !base || !parameters || !topics || !timers) {
    return;
  }

  const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
  pubs_.occ_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/occupied", qos);
  pubs_.raw_occ_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/raw_occupied", qos);
  pubs_.unknown_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/unknown", qos);
  pubs_.occ_inf_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/inflated_occupied", qos);
  pubs_.unknown_inf_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/inflated_unknown", qos);
  pubs_.frontier_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/frontier", qos);
  if (cfg.esdf_en) {
    pubs_.esdf_pub =
      createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "rog_map/esdf", qos);
  }
  pubs_.layer_value_pub =
    createPublisher<nav_msgs::msg::OccupancyGrid>(parameters, topics, "/rog_map/layer_value", qos);
  pubs_.layer_value_dynamic_pub = createPublisher<nav_msgs::msg::OccupancyGrid>(
    parameters, topics, "/rog_map/layer_value_dynamic", qos);
  pubs_.layer_value_static_pub = createPublisher<nav_msgs::msg::OccupancyGrid>(
    parameters, topics, "/rog_map/layer_value_static", qos);
  pubs_.layer_type_pub =
    createPublisher<nav_msgs::msg::OccupancyGrid>(parameters, topics, "/rog_map/layer_type", qos);
  pubs_.layer_confidence_pub =
    createPublisher<nav_msgs::msg::OccupancyGrid>(parameters, topics, "/rog_map/layer_confidence", qos);
  pubs_.layer_height_delta_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/layer_height_delta", qos);
  pubs_.field_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/field", qos);
  pubs_.decay_cells_pub =
    createPublisher<sensor_msgs::msg::PointCloud2>(parameters, topics, "/rog_map/decay_cells", qos);
  pubs_.mkr_arr_pub =
    createPublisher<visualization_msgs::msg::MarkerArray>(parameters, topics, "/rog_map/map_bound", qos);

  if (cfg.visualization_rate > 0.0 && callback) {
    callback_group_ = base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto period =
      std::chrono::milliseconds(std::max(1, static_cast<int>(1000.0 / cfg.visualization_rate)));
    timer_ =
      rclcpp::create_wall_timer(period, std::move(callback), callback_group_, base.get(), timers.get());
  }
}

void ROGMapVisualizer::reset()
{
  timer_.reset();
  callback_group_.reset();
  pubs_ = Publishers{};
}

}  // namespace rog_map
