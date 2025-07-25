#include <../include/behavier_tree_com.h>
#include <fmt/core.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
void BehavierTreeCom::Init()
{
  ros::NodeHandle nh2;

  cmd_vel.linear.x = 0.;
  cmd_vel.linear.y = 0.;

  chassis_sender = nh2.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &BehavierTreeCom::sendChassisCtrlCB, this);
  odom_sub = nh2.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &BehavierTreeCom::odomCB, this);
}

void BehavierTreeCom::sendChassisCtrlCB(const geometry_msgs::TwistConstPtr &velPtr)
{
  cmd_vel = *velPtr;
  float vx_mps = cmd_vel.linear.x;
  float vy_mps = cmd_vel.linear.y;
  float vw_rpm = 3.14 * 2;
  float current_yaw = tf::getYaw(odom.pose.pose.orientation) * 180 / M_PI;
  if (isnan(current_yaw))
  {
    current_yaw = 0;
  }

  ChassisTarget target(vx_mps, vy_mps, vw_rpm, odom.pose.pose.position.x, odom.pose.pose.position.y, current_yaw);
  ROS_INFO("x:%.2f,y:%.2f,w:%.2f,yaw:%.2f", vx_mps, vy_mps, vw_rpm, current_yaw);
  Communication::send2stm32(target);
  ROS_INFO("success to send message to stm32!");
}