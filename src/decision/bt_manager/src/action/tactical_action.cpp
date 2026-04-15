#include "bt_manager/action/tactical_action.hpp"

namespace Sentry_BT {
namespace {
TacticalMode parseTacticalMode(const std::string & mode)
{
  if (mode == "attack") {
    return TacticalMode::OFFENSIVE;
  }
  if (mode == "defend") {
    return TacticalMode::DEFENSIVE;
  }
  return TacticalMode::BALANCED;
}
}  // namespace

ChangeTacticalAction::ChangeTacticalAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ChangeTacticalAction::providedPorts()
{
  return {BT::InputPort<std::string>("mode")};
}

BT::NodeStatus ChangeTacticalAction::tick()
{
  auto blackboard = config().blackboard;

  const auto mode = getInput<std::string>("mode");
  const TacticalMode tactical_mode = mode ? parseTacticalMode(mode.value()) : TacticalMode::BALANCED;

  blackboard->set<TacticalMode>("tactical_mode", tactical_mode);
  return BT::NodeStatus::SUCCESS;
}

SetTacticalMode::SetTacticalMode(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetTacticalMode::providedPorts()
{
  return {BT::InputPort<std::string>("mode")};
}

BT::NodeStatus SetTacticalMode::tick()
{
  auto blackboard = config().blackboard;

  const auto mode = getInput<std::string>("mode");
  const TacticalMode tactical_mode = mode ? parseTacticalMode(mode.value()) : TacticalMode::BALANCED;

  blackboard->set<TacticalMode>("tactical_mode", tactical_mode);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
