#include "bt_manager/condition/change_stance_condition.hpp"
using namespace color_text;
namespace Sentry_BT
{
// ------------------- CheckMPCondition -------------------
CheckMPCondition::CheckMPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckMPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckMPCondition::tick()
{
  std::cout << BLUE << "---------- CheckMPCondition ----------" << RESET << std::endl;
  auto blackboard = config().blackboard;
  blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::MOVE);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckAPCondition -------------------
CheckAPCondition::CheckAPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckAPCondition::tick()
{
  std::cout << BLUE << "---------- CheckAPCondition ----------" << RESET << std::endl;
  auto blackboard = config().blackboard;
  try
  {
    bool target_valid = blackboard->get<bool>("target_valid");
    bool outpost_msg = blackboard->get<bool>("outpost_msg");

    if(target_valid || outpost_msg)
    {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::ATTACK);
      return BT::NodeStatus::SUCCESS;
    }
  }
  catch(...)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckDPCondition -------------------
CheckDPCondition::CheckDPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckDPCondition::tick()
{
  std::cout << BLUE << "---------- CheckDPCondition ----------" << RESET << std::endl;
  auto blackboard = config().blackboard;
  try
  {
    float current_health = blackboard->get<float>("health");

    if(current_health <= 50.0f)
    {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::DEFEND);
      return BT::NodeStatus::SUCCESS;
    }
  }
  catch(...)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
