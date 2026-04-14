#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>

namespace Sentry_BT
{
class TrackTargetAction : public BT::StatefulActionNode
{
public:
	TrackTargetAction(const std::string& name, const BT::NodeConfiguration& config);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void onHalted() override;
};

class SetGimbalPose : public BT::SyncActionNode
{
public:
	SetGimbalPose(const std::string& name, const BT::NodeConfiguration& config);

	static BT::PortsList providedPorts();
	BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
