#pragma once

// ROS2 Normal Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS2 Message Libraries
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

// Custom Messages
#include "ros_interfaces/msg/event_status.hpp"

// Project Headers
#include "bt_manager/blackboard.hpp"
#include "bt_manager/tf_utils.hpp"
#include "nav_zone.hpp"

namespace Sentry_BT
{
  class ros_interface : public rclcpp::Node
  {
  private:
    rclcpp::Subscription<ros_interfaces::msg::EventStatus>::SharedPtr event_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr outpost_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<Blackboard> blackboard_;

    int nav_status_;
    geometry_msgs::msg::Pose current_pose_;
    void eventCallback(const ros_interfaces::msg::EventStatus::SharedPtr msg);

  public:
    ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr);
    ~ros_interface() override = default;

    bool publishNavigationGoal(const Sentry_BT::Point2D& goal);
    bool TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose);
  };
}  // namespace Sentry_BT
