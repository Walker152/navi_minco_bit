#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose.hpp>

namespace Sentry_BT {
class CheckTargetVisible : public BT::ConditionNode
{
public:
  CheckTargetVisible(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
