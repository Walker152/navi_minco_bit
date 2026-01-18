#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "nav_zone.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int32.hpp>

namespace Sentry_BT
{
class PublishNavigationGoal : public BT::SyncActionNode
{
public:
  PublishNavigationGoal(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

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

class WaitUntilStopped : public BT::StatefulActionNode
{
public:
  WaitUntilStopped(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class ChangePosition : public BT::SyncActionNode
{
public:
  ChangePosition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub_;
};

class JustProtect : public BT::SyncActionNode
{
public:
  JustProtect(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace Sentry_BT