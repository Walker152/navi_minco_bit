#pragma once
#include <memory>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robots_msgs/msg/chassis.hpp"
#include <serial/serial.h>
#include "sheriffos/util/enum_name.h"
#include "utils.h"
#include "tf2/LinearMath/Transform.h"
#include "../../RM_encoder/communication.hpp"
// 单例类 BehavierTreeCom，封装了底盘控制与里程计相关通信接口
class BehavierTreeCom : public rclcpp::Node
{
private:
  BehavierTreeCom() : rclcpp::Node("behavier_tree_com_node") {};

public:
  static BehavierTreeCom &getInstance()
  {
    static BehavierTreeCom instance;
    return instance;
  }
  ~BehavierTreeCom() = default;
  void Init();

private:
  void sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr &velPtr);
  void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr &odomPtr);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sender_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // 订阅yaw轴信息?
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_yaw_sub_;
  geometry_msgs::msg::Twist cmd_vel_;
  nav_msgs::msg::Odometry odom_;
};