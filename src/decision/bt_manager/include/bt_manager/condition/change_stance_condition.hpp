#pragma once

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

#include <chrono>
#include <limits>
#include <string>

namespace Sentry_BT {
namespace detail {
inline bool compareByMode(const float lhs, const float rhs, const std::string & mode)
{
  if (mode == "greater") {
    return lhs >= rhs;
  }
  if (mode == "less") {
    return lhs <= rhs;
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

class CheckStanceCooldown : public BT::ConditionNode
{
public:
  CheckStanceCooldown(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckStanceEffectLimit : public BT::ConditionNode
{
public:
  CheckStanceEffectLimit(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

/* deprecated - replaced by accumulated time logic */
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

// 隧道变形控制：通过黑板标志位设置升降机构位置
class CheckTunnelDeformation : public BT::ConditionNode
{
public:
  CheckTunnelDeformation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  float last_health_ = 100.0f;
  bool health_initialized_ = false;
  rclcpp::Time last_hurt_time_;
};

class CheckInEnemyFortZone : public BT::ConditionNode
{
public:
  CheckInEnemyFortZone(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 敌方防守区受击响应：掉血后保持激活，连续一段时间未再掉血后退出。
class CheckEnemyDefenseHealthDrop : public BT::ConditionNode
{
public:
  CheckEnemyDefenseHealthDrop(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  float last_health_ = std::numeric_limits<float>::max();
  std::chrono::steady_clock::time_point last_health_drop_time_{};
  bool response_active_ = false;
  bool initialized_ = false;
};

// 检查操作手手动强化姿态覆盖是否应当生效。
// 判断 3 个条件:override_active && 能量达标 && 该强化姿态未超时。
// 隧道判断不在此,由树结构的优先级保证(隧道分支在前,短路后面分支)。
class CheckManualStanceOverride : public BT::ConditionNode
{
public:
  CheckManualStanceOverride(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 检查当前是否应该强化指定姿态（基于姿态累计时间、比赛剩余时间、强化配额、能量等）。
// 两级触发逻辑：
// 1. 一级触发：普通姿态累计时间 >= accumulated_threshold（效果降级前续命）
// 2. 二级触发：比赛剩余时间 <= fallback_game_time 且配额 >= 5s（保底开启，避免浪费）
// 任一触发条件满足 + 能量充足 + 配额足够 → SUCCESS
class CheckShouldEnhanceStance : public BT::ConditionNode
{
public:
  CheckShouldEnhanceStance(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
