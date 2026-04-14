#include "bt_manager/condition/gimbal_condition.hpp"

namespace Sentry_BT
{
CheckTargetVisible::CheckTargetVisible(const std::string& name, const BT::NodeConfiguration& config)
	: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetVisible::providedPorts()
{
	return {
			BT::InputPort<bool>("target_valid"),
			BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),
			BT::InputPort<float>("gimbal_yaw")
	};
}

BT::NodeStatus CheckTargetVisible::tick()
{
	auto blackboard = config().blackboard;

	// Blackboard data skeleton for later business logic.
	try
	{
		const auto target_valid = blackboard->get<bool>("target_valid");
		const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
		const auto gimbal_yaw = blackboard->get<float>("gimbal_yaw");
		(void)target_valid;
		(void)target_pose;
		(void)gimbal_yaw;
	}
	catch(...)
	{
	}

	return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
