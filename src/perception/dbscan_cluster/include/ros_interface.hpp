#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ros_interfaces/msg/dynamic_obstacle.hpp>
#include <ros_interfaces/msg/dynamic_obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dbscan_cluster.hpp"
#include "kalman_tracker.hpp"

namespace DBSCANCluster
{

struct VisionTarget
{
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
};

class RosInterface : public rclcpp::Node
{
public:
  RosInterface();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void visionCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

  DBSCANClusterAlg cluster_alg_;
  KalmanTracker tracker_alg_;
  ClusterConfig cluster_config_;
  TrackerConfig tracker_config_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_vision_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<ros_interfaces::msg::DynamicObstacleArray>::SharedPtr pub_obstacles_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_stamp_ = false;
  
  std::vector<VisionTarget> latest_vision_targets_;
  std::mutex vision_mutex_;
};

}  // namespace DBSCANCluster
