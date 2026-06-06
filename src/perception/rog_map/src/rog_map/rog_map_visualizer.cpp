#include <rog_map/rog_map_visualizer.hpp>

#include <algorithm>

namespace rog_map {

void ROGMapVisualizer::configure(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &base,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &parameters,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr &topics,
    const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr &timers,
    const Config &cfg,
    Callback callback) {
    reset();
    if (!cfg.visualization_en || !base || !parameters || !topics || !timers) {
        return;
    }

    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());
    if (cfg.viz_occupied_enable) {
        pubs_.occ_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_occupied_topic, qos);
    }
    if (cfg.viz_unknown_enable) {
        pubs_.unknown_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_unknown_topic, qos);
    }
    if (cfg.viz_inflated_occupied_enable) {
        pubs_.occ_inf_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_inflated_occupied_topic, qos);
    }
    if (cfg.viz_inflated_unknown_enable) {
        pubs_.unknown_inf_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_inflated_unknown_topic, qos);
    }
    if (cfg.viz_frontier_enable && cfg.frontier_extraction_en) {
        pubs_.frontier_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_frontier_topic, qos);
    }
    if (cfg.esdf_en) {
        pubs_.esdf_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, "rog_map/esdf", qos);
    }
    if (cfg.layer_en && cfg.viz_layer_value_enable) {
        pubs_.layer_value_pub = createPublisher<nav_msgs::msg::OccupancyGrid>(
            parameters, topics, cfg.viz_layer_value_topic, qos);
    }
    if (cfg.layer_en && cfg.viz_layer_type_enable) {
        pubs_.layer_type_pub = createPublisher<nav_msgs::msg::OccupancyGrid>(
            parameters, topics, cfg.viz_layer_type_topic, qos);
    }
    if (cfg.layer_en && cfg.viz_layer_confidence_enable) {
        pubs_.layer_confidence_pub = createPublisher<nav_msgs::msg::OccupancyGrid>(
            parameters, topics, cfg.viz_layer_confidence_topic, qos);
    }
    if (cfg.layer_en && cfg.viz_layer_height_enable) {
        pubs_.layer_height_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_layer_height_topic, qos);
    }
    if (cfg.viz_field_enable) {
        pubs_.field_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_field_topic, qos);
    }
    if (cfg.viz_decay_cells_enable) {
        pubs_.decay_cells_pub = createPublisher<sensor_msgs::msg::PointCloud2>(
            parameters, topics, cfg.viz_decay_cells_topic, qos);
    }
    if (cfg.performance_enable && cfg.performance_publish_enable) {
        pubs_.performance_pub = createPublisher<std_msgs::msg::String>(
            parameters, topics, cfg.performance_topic, qos);
    }
    if (cfg.viz_map_bound_enable) {
        pubs_.mkr_arr_pub = createPublisher<visualization_msgs::msg::MarkerArray>(
            parameters, topics, cfg.viz_map_bound_topic, qos);
    }

    if (cfg.viz_time_rate > 0.0 && callback) {
        callback_group_ = base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const auto period = std::chrono::milliseconds(
            std::max(1, static_cast<int>(1000.0 / cfg.viz_time_rate)));
        timer_ = rclcpp::create_wall_timer(
            period,
            std::move(callback),
            callback_group_,
            base.get(),
            timers.get());
    }
}

void ROGMapVisualizer::reset() {
    timer_.reset();
    callback_group_.reset();
    pubs_ = Publishers{};
}

}  // namespace rog_map
