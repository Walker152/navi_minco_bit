#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"

namespace Sentry_BT {
class TrackTargetAction : public BT::StatefulActionNode
{
public:
  TrackTargetAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class SetGimbalPose : public BT::SyncActionNode
{
public:
  SetGimbalPose(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetGimbalPoseByAreaAction : public BT::SyncActionNode
{
public:
  SetGimbalPoseByAreaAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
