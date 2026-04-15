#include "bt_manager/condition/change_stance_condition.hpp"
using namespace color_text;
namespace Sentry_BT {
// ------------------- CheckMPCondition -------------------
CheckMPCondition::CheckMPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckMPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckMPCondition::tick()
{
  auto blackboard = config().blackboard;
  static bool last_condition_met = false;
  const bool condition_met = true;

  if (condition_met != last_condition_met) {
    std::cout << WHITE << "CheckMPCondition => MOVE fallback active" << RESET << std::endl;
    last_condition_met = condition_met;
  }

  blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::MOVE);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckAPCondition -------------------
CheckAPCondition::CheckAPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckAPCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    auto current_mode = blackboard->get<int>("current_mode");
    bool outpost_msg = blackboard->get<bool>("outpost_msg");
    const bool condition_met = current_mode == static_cast<int>(Sentry_BT::NavMode::TRACING) || outpost_msg;

    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? GREEN : YELLOW) << "CheckAPCondition => "
                << (condition_met ? "ATTACK enabled" : "ATTACK disabled") << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::ATTACK);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckDPCondition -------------------
CheckDPCondition::CheckDPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckDPCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    float current_health = blackboard->get<float>("health");
    const bool condition_met = current_health <= 50.0f;

    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? RED : WHITE) << "CheckDPCondition => "
                << (condition_met ? "DEFEND enabled" : "DEFEND disabled") << ", health=" << current_health
                << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::DEFEND);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
