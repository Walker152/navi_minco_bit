#include "../include/behavier_tree_com.h"
#include <plog/Log.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

void BehavierTreeCom::Init()
{
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  chassis_sender_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      std::bind(&BehavierTreeCom::sendChassisCtrlCB, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 1,
      std::bind(&BehavierTreeCom::odomCB, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "\nGot /cmd_vel message");
}

void BehavierTreeCom::odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr &odomPtr)
{
  this->odom_ = *odomPtr;
}

void BehavierTreeCom::sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr &velPtr)
{
  cmd_vel_ = *velPtr;

  float vx_mps = cmd_vel_.linear.x;
  float vy_mps = cmd_vel_.linear.y;
  float vw_rpm = 3.14;
  tf2::Quaternion q;
  tf2::fromMsg(odom_.pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  float current_yaw = static_cast<float>(yaw * 180.0 / M_PI);

  ChassisTarget target(vx_mps, vy_mps, vw_rpm,
                       odom_.pose.pose.position.x, odom_.pose.pose.position.y, current_yaw);

  Communication::send2stm32(target);
  RCLCPP_INFO(this->get_logger(), "success to send message to stm32!");

}