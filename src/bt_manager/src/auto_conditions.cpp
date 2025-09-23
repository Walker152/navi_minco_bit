#include "bt_manager/blackboard.hpp"
#include "bt_manager/auto_conditions.hpp"

namespace Sentry_BT
{
// ------------------- CheckRetreatCondition -------------------
CheckRetreatCondition::CheckRetreatCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}  

BT::PortsList CheckRetreatCondition::providedPorts()
{
  return { BT::InputPort<float>("health") };
}

BT::NodeStatus CheckRetreatCondition::tick()
{
  // 从黑板获取当前健康值
  auto health = getInput<float>("health");
  if (!health)
  {
    throw BT::RuntimeError("missing required input [health]: ", health.error());
  }

  // 检查健康值是否低于阈值
  if (health.value() < 30.0)
  {
    return BT::NodeStatus::SUCCESS;   
  }
  else
  {
    return BT::NodeStatus::FAILURE;   
  }
}

// ------------------- CheckFortBonusActive -------------------
CheckFortBonusActive::CheckFortBonusActive(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckFortBonusActive::providedPorts()
{
  return { BT::InputPort<bool>("fort_bonus_active") };
}

BT::NodeStatus CheckFortBonusActive::tick()
{
  // 从黑板获取据点加成状态
  auto bonus_active = getInput<bool>("fort_bonus_active");
  if (!bonus_active)
  {
    throw BT::RuntimeError("missing required input [fort_bonus_active]: ", bonus_active.error());
  }

  // 检查据点加成是否激活
  if (bonus_active.value())
  {
    return BT::NodeStatus::SUCCESS;   
  }
  else
  {
    return BT::NodeStatus::FAILURE;   
  }
}

// ------------------- CheckTargetLocked -------------------
CheckTargetLocked::CheckTargetLocked(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}   

BT::PortsList CheckTargetLocked::providedPorts()
{
  return { BT::InputPort<bool>("target_locked") };
}

BT::NodeStatus CheckTargetLocked::tick()
{
  // 从黑板获取目标锁定状态
  auto target_locked = getInput<bool>("target_locked");
  if (!target_locked)
  {
    throw BT::RuntimeError("missing required input [target_locked]: ", target_locked.error());
  }

  // 检查目标是否锁定
  if (target_locked.value())
  {
    return BT::NodeStatus::SUCCESS;   
  }
  else
  {
    return BT::NodeStatus::FAILURE;   
  }
}

}  // namespace Sentry_BT
