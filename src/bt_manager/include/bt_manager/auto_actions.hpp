#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "bt_manager/blackboard.hpp"

namespace Sentry_BT
{

class SetHomeCoordinate : public BT::SyncActionNode
{
public:
  SetHomeCoordinate(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class PublishNavigationGoal : public BT::SyncActionNode
{
public:
  PublishNavigationGoal(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetBonusCoordinate : public BT::SyncActionNode
{
public:
  SetBonusCoordinate(const std::string& name, const BT::NodeConfiguration& config);
  
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

class SelectInspectionArea : public BT::SyncActionNode
{
public:
  SelectInspectionArea(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetAreaCoordinate : public BT::SyncActionNode
{
public:
  SetAreaCoordinate(const std::string& name, const BT::NodeConfiguration& config);
  
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

}  // namespace Sentry_BT