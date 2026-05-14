#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <limits>

namespace Sentry_BT {
class CheckRetreatCondition : public BT::ConditionNode
{
public:
  CheckRetreatCondition(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  double health_threshold;
};

class CheckOutpostRemained : public BT::ConditionNode
{
public:
  CheckOutpostRemained(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckManualOverride : public BT::ConditionNode
{
public:
  CheckManualOverride(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckOutpostSafeResponse : public BT::ConditionNode
{
public:
  CheckOutpostSafeResponse(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  float last_health_ = std::numeric_limits<float>::max();
  std::chrono::steady_clock::time_point last_health_change_time_;
  bool cooldown_active_ = false;
};

class CheckTargetLocked : public BT::ConditionNode
{
public:
  CheckTargetLocked(const std::string & name, const BT::NodeConfiguration & config);
  geometry_msgs::msg::Pose target_pose;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 检查是否在台阶区域的条件节点
class CheckInStairsZone : public BT::ConditionNode
{
public:
  CheckInStairsZone(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckNoAllyBelowStairs : public BT::ConditionNode
{
public:
  CheckNoAllyBelowStairs(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckAmmoLow : public BT::ConditionNode
{
public:
  CheckAmmoLow(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckTacticalModeCondition : public BT::ConditionNode
{
public:
  CheckTacticalModeCondition(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckOwnFortIdle : public BT::ConditionNode
{
public:
  CheckOwnFortIdle(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT