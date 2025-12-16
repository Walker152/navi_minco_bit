#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include "nav_zone.hpp"

namespace Sentry_BT
{

class CheckRetreatCondition : public BT::ConditionNode
{
public:
  CheckRetreatCondition(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  double health_threshold;
};


class CheckOutpostRemained : public BT::ConditionNode
{
public:
  CheckOutpostRemained(const std::string& name, const BT::NodeConfiguration& config);
  
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

class CheckNavStatus : public BT::ConditionNode
{
public:
  CheckNavStatus(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckIfRetreating : public BT::ConditionNode
{
  public:
    CheckIfRetreating(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
}  // namespace robot_behavior_tree