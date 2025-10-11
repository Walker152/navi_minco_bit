#pragma once
#include "com.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace ns_com
{
  class BehavierTreeCom : public rclcpp::Node
  {
  public:
    explicit BehavierTreeCom(const std::string& name);

    ~BehavierTreeCom() override = default;

  private:
    void Init();
    //
    void sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr& velPtr);
    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr& odomPtr);
    void desiredYawCB(const std_msgs::msg::Float32::ConstSharedPtr& yawPtr);
    void outpostCB(const std_msgs::msg::Bool::ConstSharedPtr& outpostPtr);
    //
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr outpost_sub_;
    //
    geometry_msgs::msg::Twist cmd_vel_;
    nav_msgs::msg::Odometry odom_;
    std_msgs::msg::Float32 gimbal_yaw_;
    bool outpost_status_ = false;
    // 数据链路层
    Communication com_;
  };

}  // namespace ns_com