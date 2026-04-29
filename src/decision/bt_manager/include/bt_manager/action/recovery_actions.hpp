#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace Sentry_BT {

class SetTunnelRecoveryAttemptPoint : public BT::SyncActionNode
{
public:
  SetTunnelRecoveryAttemptPoint(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetTunnelRecoveryRetreatPoint : public BT::SyncActionNode
{
public:
  SetTunnelRecoveryRetreatPoint(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetGlobalVelocity : public BT::SyncActionNode
{
public:
  SetGlobalVelocity(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
