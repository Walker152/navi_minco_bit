#pragma once

#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/area.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

#include <string>

namespace Sentry_BT {
namespace detail {
inline bool compareByMode(const float lhs, const float rhs, const std::string & mode)
{
  if (mode == "greater") {
    return lhs >= rhs;
  }
  if (mode == "less") {
    return lhs < rhs;
  }
  return false;
}
}  // namespace detail

class CheckHeat : public BT::ConditionNode
{
public:
  CheckHeat(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckOutpostTarget : public BT::ConditionNode
{
public:
  CheckOutpostTarget(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckEngagedStatus : public BT::ConditionNode
{
public:
  CheckEngagedStatus(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckHealth : public BT::ConditionNode
{
public:
  CheckHealth(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckTargetDistance : public BT::ConditionNode
{
public:
  CheckTargetDistance(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckCrossZoneTransition : public BT::ConditionNode
{
public:
  CheckCrossZoneTransition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckCapacitorCapacity : public BT::ConditionNode
{
public:
  CheckCapacitorCapacity(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckStanceRefreshRequired : public BT::ConditionNode
{
public:
  CheckStanceRefreshRequired(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  SentryStance last_stance{SentryStance::DEFEND};
  std::chrono::steady_clock::time_point hold_start{};
};

}  // namespace Sentry_BT
