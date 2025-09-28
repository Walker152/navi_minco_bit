#pragma once

// ROS2 Normal Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS2 Message Libraries
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

// Custom Messages
#include "robot_msgs/msg/event_status.hpp"

// Project Headers
#include "bt_manager/blackboard.hpp"
#include "bt_manager/tf_utils.hpp"
#include "nav_zone.hpp"
namespace Sentry_BT
{   
    class ros_interface: public rclcpp::Node
    {
    private:
        rclcpp::Subscription<robot_msgs::msg::EventStatus>::SharedPtr event_sub;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
        std::shared_ptr<Blackboard> blackboard_;

        int nav_status_;
        void eventCallback(const robot_msgs::msg::EventStatus::SharedPtr msg);
        
    public:
        ros_interface(std::shared_ptr<Blackboard> blackboard_ptr);
        ~ros_interface();

        bool publishNavigationGoal(const Sentry_BT::Point2D & goal);
        bool TransformPose(const geometry_msgs::msg::Pose & input_pose, geometry_msgs::msg::Pose & output_pose);
    };
}
    