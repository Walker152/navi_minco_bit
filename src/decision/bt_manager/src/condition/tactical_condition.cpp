#include "bt_manager/condition/tactical_condition.hpp"
#include "bt_manager/utils/log.hpp"

#include <sstream>

namespace Sentry_BT {
CheckDefendCondition::CheckDefendCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDefendCondition::providedPorts()
{
  return {BT::InputPort<int>("home_health_threshold", 1000, "Home HP threshold"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckDefendCondition::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto threshold = getInput<int>("home_health_threshold").value_or(1000);
  int home_health = 3000;
  try {
    home_health = blackboard->get<int>("home_health");
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }

  const bool condition_met = home_health < threshold;
  std::ostringstream oss;
  oss << "home_health=" << home_health << ", threshold=" << threshold;
  detail::logTransition(
    detail::TreeKind::TACTICAL, "CheckDefendCondition", condition_met, oss.str(), branch);

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckAttackCondition::CheckAttackCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAttackCondition::providedPorts()
{
  return {BT::InputPort<int>("home_health_threshold", 1000, "Home HP threshold"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckAttackCondition::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto control_mode = blackboard->get<ControlMode>("control_mode");
  const auto manual_goal = blackboard->get<Point2D>("manual_override_goal");
  const bool manual_attack_command =
    control_mode == ControlMode::MANUAL_CONTROL && enemy_fort_zone.contains(manual_goal);

  const bool condition_met = manual_attack_command;

  std::ostringstream oss;
  oss << ", manual_attack_command=" << manual_attack_command;

  detail::logTransition(
    detail::TreeKind::TACTICAL, "CheckAttackCondition", condition_met, oss.str(), branch);

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
