#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/action_node.h>

namespace Sentry_BT {

// ------------------- 资源管理动作节点 -------------------

class RequestReviveAction : public BT::StatefulActionNode
{
public:
  RequestReviveAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
};

class RequestRemoteAmmoExchangeAction : public BT::SyncActionNode
{
public:
  RequestRemoteAmmoExchangeAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  uint8_t ammo_exchange_request_num = 0;
};

class RequestRemoteHealthExchangeAction : public BT::SyncActionNode
{
public:
  RequestRemoteHealthExchangeAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  uint8_t health_exchange_request_num = 0;
};
}  // namespace Sentry_BT
