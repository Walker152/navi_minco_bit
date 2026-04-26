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

  const auto threshold = getInput<int>("home_health_threshold").value_or(1000);
  const bool enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
  const int small_energy_status = blackboard->get<int>("small_energy_status");
  const int big_energy_status = blackboard->get<int>("big_energy_status");
  const int home_health = blackboard->get<int>("home_health");

  const bool energy_active = small_energy_status > 0 || big_energy_status > 0;
  const bool condition_met = enemy_outpost_destroyed && energy_active && home_health > threshold;

  std::ostringstream oss;
  oss << "outpost_destroyed=" << enemy_outpost_destroyed << ", energy_active=" << energy_active
      << ", home_health=" << home_health << ", threshold=" << threshold;
  detail::logTransition(
    detail::TreeKind::TACTICAL, "CheckAttackCondition", condition_met, oss.str(), branch);

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
