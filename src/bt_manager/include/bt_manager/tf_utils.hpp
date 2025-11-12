#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

namespace Sentry_BT
{

class TransformUtils : public rclcpp::Node
{
public:
    TransformUtils();
    ~TransformUtils() override = default;
    
    // 发布静态TF变换（从base_link到gimbal）
    void publishStaticTransform(const std_msgs::msg::Float32::ConstSharedPtr &msg);
    
    // 坐标转换函数（从gimbal系转换到base_link系）
    bool transformPoseToBaseLink(const geometry_msgs::msg::Pose& input_pose, 
                                geometry_msgs::msg::Pose& output_pose);
    
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<std_msgs::msg::Float32>::ConstSharedPtr gimbal_yaw_sub_;
};

} // namespace Sentry_BT