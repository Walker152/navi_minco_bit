#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include "bt_manager/ros_interface.hpp"

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

// 检查是否在台阶区域的条件节点
class CheckInStairsZone : public BT::ConditionNode
{
public:
  CheckInStairsZone(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace Sentry_BT