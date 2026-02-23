#include "bt_manager/condition/change_stance_condition.hpp"

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
  std::cout << "---------- CheckMPCondition ----------" << std::endl;
  auto blackboard = config().blackboard;
  try
  {
    bool enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
    int current_mode = blackboard->get<int>("current_mode");
    bool outpost_msg = blackboard->get<bool>("outpost_msg");

    bool condition_met = ((!enemy_outpost_destroyed) && (current_mode == Sentry_BT::NavMode::RESPONSE) &&
                          (!outpost_msg)) ||
                         (current_mode == Sentry_BT::NavMode::RETREAT) ||
                         (current_mode == Sentry_BT::NavMode::PATROL);

    std::cout << "Condition check: enemy_outpost_destroyed=" << (enemy_outpost_destroyed ? "true" : "false")
              << ", current_mode=" << mode_names[current_mode]
              << ", outpost_msg=" << (outpost_msg ? "true" : "false") << std::endl;
    if(condition_met)
    {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::MOVE);
      return BT::NodeStatus::SUCCESS;
    }
  }
  catch(...)
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
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
  std::cout << "---------- CheckAPCondition ----------" << std::endl;
  auto blackboard = config().blackboard;
  try
  {
    bool enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
    int current_mode = blackboard->get<int>("current_mode");
    bool outpost_msg = blackboard->get<bool>("outpost_msg");

    bool condition_met = ((!enemy_outpost_destroyed) && (current_mode == Sentry_BT::NavMode::RESPONSE) &&
                          outpost_msg) ||
                         (current_mode == Sentry_BT::NavMode::TRACING) ||
                         (current_mode == Sentry_BT::NavMode::PATROL);

    if(condition_met)
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
  std::cout << "---------- CheckDPCondition ----------" << std::endl;
  auto blackboard = config().blackboard;
  try
  {
    float current_health = blackboard->get<float>("health");
    bool condition_met = (current_health <= 30.0f);

    if(condition_met)
    {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", SentryStance::DEFEND);
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
