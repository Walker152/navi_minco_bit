#pragma once

#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

namespace Sentry_BT {
class CheckAttackStanceCondition : public BT::ConditionNode
{
public:
  CheckAttackStanceCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool heat_attack_latched_ = false;
};

class CheckMoveStanceCondition : public BT::ConditionNode
{
public:
  CheckMoveStanceCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckDefendStanceCondition : public BT::ConditionNode
{
public:
  CheckDefendStanceCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckStanceRefreshRequired : public BT::ConditionNode
{
public:
  CheckStanceRefreshRequired(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
