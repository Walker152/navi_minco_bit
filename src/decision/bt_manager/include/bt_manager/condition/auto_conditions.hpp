#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>

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

private:
  bool initialized_ = false;
  Point2D last_goal_;
  std::chrono::steady_clock::time_point last_goal_change_time_;
};

class CheckOutpostSafeResponse : public BT::ConditionNode
{
public:
  CheckOutpostSafeResponse(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_ = false;
  float last_health_ = 100.0f;
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

// 检查是否将要过隧道
class CheckWillThroughTunnel : public BT::ConditionNode
{
public:
  CheckWillThroughTunnel(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static bool last_state_;
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

class CheckEnemyBaseLowHp : public BT::ConditionNode
{
public:
  CheckEnemyBaseLowHp(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace Sentry_BT