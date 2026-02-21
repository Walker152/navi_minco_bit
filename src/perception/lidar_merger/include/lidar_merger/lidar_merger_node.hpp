#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

class LidarMergerNode : public rclcpp::Node {
public:
  LidarMergerNode();

private:
  void loadParams();
  void loadExtrinsics();

  static livox_ros_driver2::msg::CustomPoint transformPoint(
    const livox_ros_driver2::msg::CustomPoint & pt_in,
    const Eigen::Matrix4f & T);

  void syncCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_front,
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_back);

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    livox_ros_driver2::msg::CustomMsg,
    livox_ros_driver2::msg::CustomMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::string front_topic_;
  std::string back_topic_;
  std::string merged_topic_;
  std::string merged_frame_id_;

  int qos_depth_ = 10;
  bool best_effort_ = true;
  int sync_queue_size_ = 20;

  std::vector<double> extrinsic_back_to_front_;

  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub_front_;
  message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> sub_back_;
  std::shared_ptr<Synchronizer> sync_;

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_merged_;

  Eigen::Matrix4f T_front_back_ = Eigen::Matrix4f::Identity();
};
