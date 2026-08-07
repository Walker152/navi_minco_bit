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
};

class CheckGameTimeWindow : public BT::ConditionNode
{
public:
  CheckGameTimeWindow(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckBigEnergyActive : public BT::ConditionNode
{
public:
  CheckBigEnergyActive(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool energy_activated = false;
};

class CheckOutpostRemained : public BT::ConditionNode
{
public:
  CheckOutpostRemained(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class UpdateOutpostAttackState : public BT::ConditionNode
{
public:
  UpdateOutpostAttackState(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  enum class Phase
  {
    WAITING,
    AUTO_ATTACK,
    RETREAT,
    DONE
  };

  Phase phase_ = Phase::WAITING;
  int retreat_count_ = 0;
  int previous_enemy_outpost_health_ = -1;
  int previous_game_status_ = 0;
  bool pregame_reset_done_ = false;
  bool stairs_resource_triggered_ = false;
  std::chrono::steady_clock::time_point retreat_start_time_{};
};

class CheckOutpostAttackState : public BT::ConditionNode
{
public:
  CheckOutpostAttackState(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetEnemyOutpostDestroyed : public BT::ConditionNode
{
public:
  SetEnemyOutpostDestroyed(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckHeroGuardActive : public BT::ConditionNode
{
public:
  CheckHeroGuardActive(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetHeroGuardActive : public BT::ConditionNode
{
public:
  SetHeroGuardActive(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class UpdateHighlandFallbackState : public BT::ConditionNode
{
public:
  UpdateHighlandFallbackState(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  int previous_game_status_ = 0;
};

class CheckHighlandFallbackActive : public BT::ConditionNode
{
public:
  CheckHighlandFallbackActive(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckOwnOutpostAlive : public BT::ConditionNode
{
public:
  CheckOwnOutpostAlive(const std::string & name, const BT::NodeConfiguration & config);

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
  bool initialized_ = false;
};

class CheckTargetLocked : public BT::ConditionNode
{
public:
  CheckTargetLocked(const std::string & name, const BT::NodeConfiguration & config);
  geometry_msgs::msg::Pose target_pose;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckTargetArmorId : public BT::ConditionNode
{
public:
  CheckTargetArmorId(const std::string & name, const BT::NodeConfiguration & config);
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
