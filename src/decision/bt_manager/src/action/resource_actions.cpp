#include "bt_manager/action/resource_actions.hpp"

#include <cstdint>

#include "bt_manager/utils/log.hpp"

namespace Sentry_BT {

// ------------------- RequestReviveAction -------------------
RequestReviveAction::RequestReviveAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RequestReviveAction::providedPorts()
{
  return {BT::InputPort<std::string>("revive_type", "free", "free or instant")};
}

BT::NodeStatus RequestReviveAction::tick()
{
  auto blackboard = config().blackboard;
  const std::string revive_type = getInput<std::string>("revive_type").value_or("free");
  if (revive_type == "free") {
    if (!blackboard->get<bool>("can_free_resurrect")) {
    detail::logTransition(detail::TreeKind::RESOURCE,
      "RequestReviveAction",
      false,
      "can_free_resurrect=false, revive_type=" + revive_type);
    return BT::NodeStatus::FAILURE;
  }
    blackboard->set<uint8_t>("revive_request", 1);
    detail::logTransition(
      detail::TreeKind::RESOURCE, "RequestReviveAction", true, "revive_type=free");
  } else if (revive_type == "instant") {
    const int cost = blackboard->get<int>("instant_resurrect_cost");
    const int coin = blackboard->get<int>("coin_remaining");
    if (coin < cost) {
      detail::logTransition(
        detail::TreeKind::RESOURCE,
        "RequestReviveAction",
        false,
        "revive_type=instant, cost=" + std::to_string(cost) + ", coin=" + std::to_string(coin));
      return BT::NodeStatus::FAILURE;
    }
    revive_request_num++;
    blackboard->set<uint8_t>("remote_revive_request", revive_request_num);
    detail::logTransition(
      detail::TreeKind::RESOURCE,
      "RequestReviveAction",
      true,
      "revive_type=instant, cost=" + std::to_string(cost));
  } else {
    detail::logTransition(
      detail::TreeKind::RESOURCE, "RequestReviveAction", false, "unknown revive_type=" + revive_type);
  }
  return BT::NodeStatus::SUCCESS;
}

// ------------------- RequestRemoteAmmoExchangeAction -------------------
RequestRemoteAmmoExchangeAction::RequestRemoteAmmoExchangeAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RequestRemoteAmmoExchangeAction::providedPorts()
{
  return {};
}

BT::NodeStatus RequestRemoteAmmoExchangeAction::tick()
{
  auto blackboard = config().blackboard;
  ammo_exchange_request_num++;
  blackboard->set<uint8_t>("remote_ammo_request", ammo_exchange_request_num);
  detail::logTransition(
    detail::TreeKind::RESOURCE,
    "RequestRemoteAmmoExchangeAction",
    true,
    "total_request_num=" + std::to_string(static_cast<int>(ammo_exchange_request_num)));
  return BT::NodeStatus::SUCCESS;
}

// ------------------- RequestRemoteHealthExchangeAction -------------------
RequestRemoteHealthExchangeAction::RequestRemoteHealthExchangeAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RequestRemoteHealthExchangeAction::providedPorts()
{
  return {};
}

BT::NodeStatus RequestRemoteHealthExchangeAction::tick()
{
  auto blackboard = config().blackboard;
  ++health_exchange_request_num;
  blackboard->set<uint8_t>("remote_health_request", health_exchange_request_num);
  detail::logTransition(
    detail::TreeKind::RESOURCE,
    "RequestRemoteHealthExchangeAction",
    true,
    "total_request_num=" + std::to_string(static_cast<int>(health_exchange_request_num)));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace Sentry_BT
