#pragma once

#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

namespace Sentry_BT {
class CheckMPCondition : public BT::ConditionNode
{
public:
  CheckMPCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckAPCondition : public BT::ConditionNode
{
public:
  CheckAPCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckDPCondition : public BT::ConditionNode
{
public:
  CheckDPCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
