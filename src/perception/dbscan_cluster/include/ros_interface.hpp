#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_interfaces/msg/dynamic_obstacle.hpp>
#include <ros_interfaces/msg/dynamic_obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dbscan_cluster.hpp"
#include "kalman_tracker.hpp"

namespace DBSCANCluster
{

class RosInterface : public rclcpp::Node
{
public:
  RosInterface();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  DBSCANClusterAlg cluster_alg_;
  KalmanTracker tracker_alg_;
  ClusterConfig cluster_config_;
  TrackerConfig tracker_config_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<ros_interfaces::msg::DynamicObstacleArray>::SharedPtr pub_obstacles_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_stamp_ = false;
};

}  // namespace DBSCANCluster
