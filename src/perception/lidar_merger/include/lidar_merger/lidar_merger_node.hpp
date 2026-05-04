#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

class LidarMergerNode : public rclcpp::Node
{
public:
  LidarMergerNode();
  explicit LidarMergerNode(const rclcpp::NodeOptions & options);

private:
  void loadParams();
  void loadExtrinsics();

  static inline livox_ros_driver2::msg::CustomPoint transformPoint(
    const livox_ros_driver2::msg::CustomPoint & pt_in,
    const Eigen::Matrix3f & R,
    const Eigen::Vector3f & t);

  static sensor_msgs::msg::PointCloud2 customMsgToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg & msg,
    const std_msgs::msg::Header & header,
    const Eigen::Matrix4f * transform = nullptr);

  void syncCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_front,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_back);

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<livox_ros_driver2::msg::CustomMsg,
    livox_ros_driver2::msg::CustomMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::string front_topic_;
  std::string back_topic_;
  std::string merged_topic_;
  std::string merged_frame_id_;

  int qos_depth_ = 10;
  bool best_effort_ = true;
  int sync_queue_size_ = 20;
  bool publish_pointcloud_ = false;

  std::vector<double> extrinsic_back_to_front_;

  std::string front_cloud_topic_;
  std::string back_cloud_topic_;
  std::string merged_cloud_topic_;

  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub_front_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub_back_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_merged_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_front_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_back_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_cloud_;

  Eigen::Matrix4f T_front_back_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f R_front_back_ = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_front_back_ = Eigen::Vector3f::Zero();
};
