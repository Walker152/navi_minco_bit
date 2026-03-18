#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>   // <---- 使用动态变换
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <atomic>

namespace Sentry_BT
{

class TransformUtils : public rclcpp::Node
{
public:
    TransformUtils();
    ~TransformUtils() override = default;
    
    // 发布动态TF变换（从base_link到gimbal）
    void publishDynamicTransform();
    
    // 坐标转换函数（从gimbal系转换到map系）
    bool transformPoseToMap(const geometry_msgs::msg::Pose& input_pose, 
                                  geometry_msgs::msg::Pose& output_pose,
                                  const std::string& child_frame);

private:
    void updateGimbalYaw(const std_msgs::msg::Float32::ConstSharedPtr &msg);

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;   // 动态发布器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr gimbal_yaw_sub_;
    rclcpp::TimerBase::SharedPtr tf_publish_timer_;
    std::atomic<float> latest_gimbal_yaw_deg_{0.0f};
};

} // namespace Sentry_BT