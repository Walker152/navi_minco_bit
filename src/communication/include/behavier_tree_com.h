#pragma once
#include <geometry_msgs/Twist.h>
#include <robots_msgs/Chassis.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sheriffos/util/enum_name.h>
#include <time.h>
#include <iostream>
#include "utils.h"
#include "tf/tf.h"
#include "../RM_encoder/communication.hpp"
#include <optional>
class BehavierTreeCom
{
private:
  BehavierTreeCom() {};

public:
  static BehavierTreeCom &getInstance()
  {
    static BehavierTreeCom ins;
    return ins;
  }
  ~BehavierTreeCom() {};
  void Init();

private:
  void sendChassisCtrlCB(const geometry_msgs::TwistConstPtr &velPtr);
  void odomCB(const nav_msgs::OdometryConstPtr &odomPtr)
  {
    odom = *odomPtr;
  }

  ros::Subscriber chassis_sender;
  ros::Subscriber odom_sub;
  ros::Subscriber chassis_yaw_sub;

  geometry_msgs::Twist cmd_vel;
  nav_msgs::Odometry odom;
};