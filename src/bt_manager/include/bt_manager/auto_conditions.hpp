#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"

namespace Sentry_BT
{

class CheckRetreatCondition : public BT::ConditionNode
{
public:
  CheckRetreatCondition(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckFortBonusActive : public BT::ConditionNode
{
public:
  CheckFortBonusActive(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckTargetLocked : public BT::ConditionNode
{
public:
  CheckTargetLocked(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace robot_behavior_tree