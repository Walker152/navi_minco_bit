#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "nav_zone.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <chrono>

namespace Sentry_BT
{
class SetCoordinate : public BT::SyncActionNode
{
public:
  SetCoordinate(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetTargetCoordinate : public BT::SyncActionNode
{
public:
  SetTargetCoordinate(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SelectPatrolPoint : public BT::SyncActionNode
{
public:
  SelectPatrolPoint(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int wait_time_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

// 速度控制节点
class DirectVelocityControl : public BT::StatefulActionNode
{
public:
  DirectVelocityControl(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  double linear_y_;
  double angular_z_;
  double duration_;
  rclcpp::Time start_time_;
  rclcpp::Time last_pub_time_;
};

//设定固定位置节点
class SetStairsPosition : public BT::SyncActionNode
{
public:
  SetStairsPosition(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace Sentry_BT