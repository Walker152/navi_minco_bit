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
    "topics.output", "/detected_bounding_boxs");

  cluster_config_.z_min = this->declare_parameter<double>("cluster.z_min", 0.15);
  cluster_config_.z_max = this->declare_parameter<double>("cluster.z_max", 0.6);
  cluster_config_.leaf_size = this->declare_parameter<double>("cluster.leaf_size", 0.05);
  cluster_config_.min_cluster_size = this->declare_parameter<int>("cluster.min_cluster_size", 15);
  cluster_config_.max_cluster_size = this->declare_parameter<int>("cluster.max_cluster_size", 1000);

  const auto seg_d = this->declare_parameter<std::vector<double>>(
    "cluster.seg_distances", {5.0, 10.0, 15.0, 20.0});
  cluster_config_.seg_distances.assign(seg_d.begin(), seg_d.end());

  const auto clu_d = this->declare_parameter<std::vector<double>>(
    "cluster.cluster_distances", {0.15, 0.20, 0.25, 0.30});
  cluster_config_.cluster_distances.assign(clu_d.begin(), clu_d.end());

  tracker_config_.match_distance_threshold =
    this->declare_parameter<double>("tracker.match_distance_threshold", 0.5);
  tracker_config_.max_missed_frames =
    this->declare_parameter<int>("tracker.max_missed_frames", 3);
  tracker_config_.dynamic_speed_threshold =
    this->declare_parameter<double>("tracker.dynamic_speed_threshold", 0.2);

  cluster_alg_.configure(cluster_config_);
  tracker_alg_.configure(tracker_config_);

  sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&RosInterface::pointCloudCallback, this, std::placeholders::_1));

  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    output_topic,
    rclcpp::QoS(rclcpp::KeepLast(5)));
};

void RosInterface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  float dt = 0.1f;
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

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = msg->header;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  for (size_t i = 0; i < objects.size(); ++i) {
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

    if (objects[i].speed > tracker_config_.dynamic_speed_threshold) {
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
}

}  // namespace EuclideanCluster
