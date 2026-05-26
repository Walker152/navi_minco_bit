#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "gicp_filter.hpp"

namespace icp_relocalization::gicp_utils {

void publishCurrentTransform(const std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_broadcaster,
  const std::string & map_frame,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Time & stamp);

void publishStaticTf(const std::shared_ptr<tf2_ros::StaticTransformBroadcaster> & static_tf_broadcaster,
  const std::string & map_frame,
  const std::string & cloud_frame_id,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Time & stamp);

void publishVisualization(const PointCloud::Ptr & cloud,
  const rclcpp::Time & stamp,
  bool visualization_en,
  bool gicp_initialized,
  const std::string & map_frame,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & aligned_cloud_pub);

void printEvaluation(
  const Eigen::Matrix4f & initial_guess, const Eigen::Matrix4f & final_transformation, double time_ms);

void publishTargetCroppedDebug(bool visualization_en,
  const std::string & map_frame,
  const rclcpp::Time & stamp,
  const GicpFilter * gicp_filter,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_target_cropped);

void publishSourceCroppedDebug(bool visualization_en,
  const GicpFilter::Options & gicp_options,
  const std::string & map_frame,
  const PointCloud::Ptr & source,
  const Eigen::Matrix4f & initial_guess,
  const rclcpp::Time & stamp,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_source_cropped);

}  // namespace icp_relocalization::gicp_utils
