#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

#include <chrono>
#include <string>

namespace Sentry_BT {

class CheckTimeInZone : public BT::ConditionNode
{
public:
  CheckTimeInZone(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool was_in_zone_ = false;
  std::chrono::steady_clock::time_point entry_time_;
};

}  // namespace Sentry_BT
