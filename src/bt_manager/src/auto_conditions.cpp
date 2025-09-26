#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include <string>


namespace Sentry_BT
{
// ------------------- CheckRetreatCondition -------------------
CheckRetreatCondition::CheckRetreatCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}  

BT::PortsList CheckRetreatCondition::providedPorts()
{
  return { 
    BT::InputPort<float>("health_threshold"),
    BT::InputPort<float>("recovery_threshold"),
  };
}

BT::NodeStatus CheckRetreatCondition::tick()
{
  auto blackboard = config().blackboard;
  // 从XML获取健康阈值和恢复阈值
  auto health_threshold_ = getInput<float>("health_threshold");
  auto recovery_threshold_ = getInput<float>("recovery_threshold");
  if (!health_threshold_ || !recovery_threshold_)
  {
    throw BT::RuntimeError("missing required input [health_threshold] or [recovery_threshold]");
  }
  float health_threshold = health_threshold_.value();
  float recovery_threshold = recovery_threshold_.value();

  auto health = blackboard->get<float>("health");
  auto current_mode = blackboard->get<int>("current_mode");

  // 如果已经在撤退模式，检查是否应该继续撤退
  if (current_mode == Sentry_BT::NavMode::RETREAT) 
  {
    // 只有当血量恢复到安全水平才退出撤退
    if (health >= recovery_threshold) 
    {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
      return BT::NodeStatus::FAILURE; // 退出撤退
    }
    return BT::NodeStatus::SUCCESS; // 继续撤退
  }
  // 如果不在撤退模式，检查是否需要进入撤退
  else if (health < health_threshold) 
  {
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::RETREAT);
    return BT::NodeStatus::SUCCESS; // 进入撤退
  }

  return BT::NodeStatus::FAILURE; // 不需要撤退
}

// ------------------- CheckTargetLocked -------------------
CheckTargetLocked::CheckTargetLocked(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{   
}

BT::PortsList CheckTargetLocked::providedPorts()
{
  return { 
    BT::InputPort<bool>("target_valid"),
    BT::InputPort<bool>("target_in_range")
  };
}

BT::NodeStatus CheckTargetLocked::tick()
{
  auto blackboard = config().blackboard;
  
  auto target_valid = blackboard->get<bool>("target_valid");
  auto target_in_range = blackboard->get<bool>("target_in_range");

  // 检查目标是否有效且在范围内
  if (target_valid && target_in_range)
  {
    return BT::NodeStatus::SUCCESS;   
  }
  
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckFortBonusActive -------------------
CheckFortBonusActive::CheckFortBonusActive(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckFortBonusActive::providedPorts()
{
  return {};
}

BT::NodeStatus CheckFortBonusActive::tick()
{
  auto blackboard = config().blackboard;
  
  auto bonus_active = blackboard->get<bool>("bonus_active");

  // 检查据点加成是否激活
  return bonus_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckNavStatus -------------------
CheckNavStatus::CheckNavStatus(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckNavStatus::providedPorts()
{
  return {};
}

BT::NodeStatus CheckNavStatus::tick()
{
  auto blackboard = config().blackboard;
  // 从黑板获取导航状态
  auto nav_status = blackboard->get<int>("nav_status");

  std::cout << "Current navigation status: " << current_nav_status[nav_status] << std::endl;
  // 只有当导航空闲或失败时，才允许选择新的巡逻点
  if (nav_status == Sentry_BT::NavStatus::IDLE)
  {
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckIfRetreating -------------------
CheckIfRetreating::CheckIfRetreating(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckIfRetreating::providedPorts()
{
  return {};
}

BT::NodeStatus CheckIfRetreating::tick()
{
  auto blackboard = config().blackboard;

  // 从黑板获取当前模式
  auto current_mode = blackboard->get<int>("current_mode");

  std::cout << "Current mode: " << mode_names[current_mode] << std::endl;

  // 检查当前模式是否为撤退模式
  return (current_mode == Sentry_BT::NavMode::RETREAT) ? 
         BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}
} // namespace Sentry_BT