#include "ros_interface.hpp"

#include <cmath>
#include <functional>

#include <pcl_conversions/pcl_conversions.h>

namespace EuclideanCluster
{

RosInterface::RosInterface()
: Node("euclidean_cluster")
{
  const std::string input_topic = this->declare_parameter<std::string>(
    "topics.input", "/filtered_points_no_ground");
  const std::string output_topic = this->declare_parameter<std::string>(
    "topics.output", "/clustered_obstacles");

  cluster_config_.z_min = this->declare_parameter<double>("cluster.z_min", 0.15);
  cluster_config_.z_max = this->declare_parameter<double>("cluster.z_max", 0.6);
  cluster_config_.max_detection_range =
    this->declare_parameter<double>("cluster.max_detection_range", 10.0);
  cluster_config_.leaf_size = this->declare_parameter<double>("cluster.leaf_size", 0.05);
  cluster_config_.cluster_tolerance =
    this->declare_parameter<double>("cluster.cluster_tolerance", 0.2);
  cluster_config_.min_cluster_size = this->declare_parameter<int>("cluster.min_cluster_size", 15);
  cluster_config_.max_cluster_size = this->declare_parameter<int>("cluster.max_cluster_size", 1000);

  tracker_config_.match_distance_threshold =
    this->declare_parameter<double>("tracker.match_distance_threshold", 0.5);
  tracker_config_.max_missed_frames =
    this->declare_parameter<int>("tracker.max_missed_frames", 3);
  tracker_config_.dynamic_speed_threshold =
    this->declare_parameter<double>("tracker.dynamic_speed_threshold", 0.2);
  tracker_config_.class_confirm_frames =
    this->declare_parameter<int>("tracker.class_confirm_frames", 3);
  tracker_config_.dt_default =
    this->declare_parameter<double>("tracker.dt_default", 0.1);
  tracker_config_.q_pos_x =
    this->declare_parameter<double>("tracker.q_pos_x", 0.01);
  tracker_config_.q_pos_y =
    this->declare_parameter<double>("tracker.q_pos_y", 0.01);
  tracker_config_.q_vel_x =
    this->declare_parameter<double>("tracker.q_vel_x", 0.25);
  tracker_config_.q_vel_y =
    this->declare_parameter<double>("tracker.q_vel_y", 0.25);
  tracker_config_.r_pos_x =
    this->declare_parameter<double>("tracker.r_pos_x", 0.04);
  tracker_config_.r_pos_y =
    this->declare_parameter<double>("tracker.r_pos_y", 0.04);

  cluster_alg_.configure(cluster_config_);
  tracker_alg_.configure(tracker_config_);

  sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&RosInterface::pointCloudCallback, this, std::placeholders::_1));

  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/clustered_obstacles_vis",
    rclcpp::QoS(rclcpp::KeepLast(5)));
  pub_obstacles_ = this->create_publisher<ros_interfaces::msg::DynamicObstacleArray>(
    output_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)));
};

void RosInterface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  float dt = std::max(1e-3f, tracker_config_.dt_default);
  rclcpp::Time current_stamp(msg->header.stamp);
  if (current_stamp.nanoseconds() == 0) {
    current_stamp = this->now();
  }
  if (has_last_stamp_) {
    const double dt_sec = (current_stamp - last_stamp_).seconds();
    if (std::isfinite(dt_sec) && dt_sec > 1e-3 && dt_sec < 1.0) {
      dt = static_cast<float>(dt_sec);
    }
  }
  last_stamp_ = current_stamp;
  has_last_stamp_ = true;

  std::vector<Detected_Obj> objects;
  cluster_alg_.processCloud(cloud, objects);
  tracker_alg_.update(objects, dt);

  ros_interfaces::msg::DynamicObstacleArray obstacle_array_msg;
  obstacle_array_msg.header = msg->header;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = msg->header;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  for (size_t i = 0; i < objects.size(); ++i) {
    ros_interfaces::msg::DynamicObstacle obstacle_msg;
    obstacle_msg.id = (objects[i].track_id >= 0) ? objects[i].track_id : static_cast<int>(i);
    obstacle_msg.centroid.x = objects[i].centroid.x();
    obstacle_msg.centroid.y = objects[i].centroid.y();
    obstacle_msg.centroid.z = objects[i].centroid.z();

    obstacle_msg.size.x = objects[i].size.x();
    obstacle_msg.size.y = objects[i].size.y();
    obstacle_msg.size.z = objects[i].size.z();

    obstacle_msg.orientation.x = objects[i].orientation.x();
    obstacle_msg.orientation.y = objects[i].orientation.y();
    obstacle_msg.orientation.z = objects[i].orientation.z();
    obstacle_msg.orientation.w = objects[i].orientation.w();

    obstacle_msg.velocity.x = objects[i].vx;
    obstacle_msg.velocity.y = objects[i].vy;
    obstacle_msg.velocity.z = 0.0;
    obstacle_msg.speed = objects[i].speed;

    obstacle_array_msg.obstacles.push_back(obstacle_msg);

    visualization_msgs::msg::Marker mk;
    mk.header = msg->header;
    mk.ns = "euclidean_cluster";
    mk.id = (objects[i].track_id >= 0) ? objects[i].track_id : static_cast<int>(i);
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose.position.x = objects[i].centroid.x();
    mk.pose.position.y = objects[i].centroid.y();
    mk.pose.position.z = objects[i].centroid.z();

    mk.pose.orientation.x = objects[i].orientation.x();
    mk.pose.orientation.y = objects[i].orientation.y();
    mk.pose.orientation.z = objects[i].orientation.z();
    mk.pose.orientation.w = objects[i].orientation.w();

    mk.scale.x = objects[i].size.x();
    mk.scale.y = objects[i].size.y();
    mk.scale.z = objects[i].size.z();

    if (objects[i].dynamic_confirmed) {
      mk.color.r = 1.0f;
      mk.color.g = 0.2f;
      mk.color.b = 0.2f;
      mk.color.a = 0.8f;

      RCLCPP_INFO(
        this->get_logger(),
        "Dynamic obstacle id=%d pos=(%.2f, %.2f) vel=(%.2f, %.2f) speed=%.2f m/s",
        objects[i].track_id,
        static_cast<double>(objects[i].centroid.x()),
        static_cast<double>(objects[i].centroid.y()),
        static_cast<double>(objects[i].vx),
        static_cast<double>(objects[i].vy),
        static_cast<double>(objects[i].speed));
    } else {
      mk.color.r = 0.1f;
      mk.color.g = 1.0f;
      mk.color.b = 0.1f;
      mk.color.a = 0.5f;
    }

    marker_array.markers.push_back(mk);
  }

  pub_markers_->publish(marker_array);
  pub_obstacles_->publish(obstacle_array_msg);
}

}  // namespace EuclideanCluster
