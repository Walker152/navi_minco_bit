#include "bt_manager/action/resource_actions.hpp"

#include <cstdint>
#include <iostream>

#include "bt_manager/utils/color_text.hpp"

using namespace color_text;

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
  if (!blackboard->get<bool>("can_free_resurrect")) {
    return BT::NodeStatus::FAILURE;
  }
  if (revive_type == "free") {
    blackboard->set<uint8_t>("revive_request", 1);
    std::cout << GREEN << "[RESOURCE_TREE]" << RESET << " RequestReviveAction => free revive" << std::endl;
  } else if (revive_type == "instant") {
    const int cost = blackboard->get<int>("instant_resurrect_cost");
    const int coin = blackboard->get<int>("coin_remaining");
    if (coin < cost) {
      std::cout << YELLOW << "[RESOURCE_TREE]" << RESET
                << " RequestReviveAction => instant revive cost=" << cost << " > coin=" << coin
                << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    revive_request_num++;
    blackboard->set<uint8_t>("remote_revive_request", revive_request_num);
    std::cout << BLUE << "[RESOURCE_TREE]" << RESET
              << " RequestReviveAction => instant revive, cost=" << cost << std::endl;
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
  std::cout << BLUE << "[RESOURCE_TREE]" << RESET
            << " RequestRemoteAmmoExchangeAction => request remote ammo exchange, total request num="
            << static_cast<int>(ammo_exchange_request_num) << std::endl;
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
  std::cout << BLUE << "[RESOURCE_TREE]" << RESET
            << " RequestRemoteHealthExchangeAction => request remote health exchange, total request num="
            << static_cast<int>(health_exchange_request_num) << std::endl;
  return BT::NodeStatus::SUCCESS;
}
}  // namespace Sentry_BT
