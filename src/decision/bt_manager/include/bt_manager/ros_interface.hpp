#pragma once

// ROS2 Normal Libraries
#include <rclcpp/rclcpp.hpp>

// ROS2 Message Libraries
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <mutex>

// Custom Messages
#include "ros_interfaces/msg/event_status.hpp"

// Project Headers
#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/tf_utils.hpp"
#include "bt_manager/utils/nav_zone.hpp"
namespace Sentry_BT
{
  class ros_interface : public rclcpp::Node
  {
  private:
    rclcpp::Subscription<ros_interfaces::msg::EventStatus>::SharedPtr event_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr outpost_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stance_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<Blackboard> blackboard_;

    geometry_msgs::msg::Pose current_pose_;
    mutable std::mutex current_pose_mutex_;
    void eventCallback(const ros_interfaces::msg::EventStatus::SharedPtr msg);

  public:
    ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr);
    ~ros_interface() override = default;

    void publishCmdVel(double linear_y, double angular_z);
    void publishPosition(int target_stance);
    geometry_msgs::msg::Pose getCurrentPose() const;

    bool TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose);
  };
}  // namespace Sentry_BT
