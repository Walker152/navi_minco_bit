#pragma once

#include <array>
#include <atomic>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>  // <---- 使用动态变换
#include <tf2_ros/transform_listener.h>

namespace Sentry_BT {

class TransformUtils : public rclcpp::Node
{
public:
  TransformUtils();
  ~TransformUtils() override = default;

  // 发布动态TF变换（从base_link到gimbal）
  void publishDynamicTransform();

  // 坐标转换函数（从child系转换到map系）
  bool transformPoseToMap(const geometry_msgs::msg::Pose & input_pose,
    geometry_msgs::msg::Pose & output_pose,
    const std::string & child_frame);

  // 手动坐标转换函数（从gimbal系转换到map系，跳过tf2查询）
  bool transformGimbalToMap(const geometry_msgs::msg::Pose & input_pose,
    geometry_msgs::msg::Pose & output_pose);

  // 坐标转换函数（从map系转换到target系）
  bool transformMapPose(const geometry_msgs::msg::Pose & input_pose,
    geometry_msgs::msg::Pose & output_pose,
    const std::string & target_frame);

  void updateGimbalYawInit(float yaw);
  void NormalizeAngle(float & angle);
  void updateCurrentPose(const geometry_msgs::msg::Pose & pose);
  float getCurrentYawDeg();
  bool waitForTransform(const std::string & target_frame, const std::string & source_frame);
  void transformYaw(
    float yaw_in, float & yaw_out, const std::string & target_frame, const std::string & source_frame);

private:
  std::mutex current_pose_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  std::mutex gimbal_to_map_mutex_;
  std::array<double, 4> gimbal_to_map_rotation_{1.0, 0.0, 0.0, 1.0};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // 动态发布器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr tf_publish_timer_;
  std::atomic<float> latest_gimbal_yaw_deg_{0.0f};
  std::atomic<bool> gimbal_yaw_initialized_{false};
};

}  // namespace Sentry_BT
