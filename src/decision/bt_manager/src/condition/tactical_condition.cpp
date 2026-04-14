#include "bt_manager/condition/tactical_condition.hpp"

namespace Sentry_BT
{
CheckDefendCondition::CheckDefendCondition(const std::string& name, const BT::NodeConfiguration& config)
	: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDefendCondition::providedPorts()
{
	return {
			BT::InputPort<float>("health"),
			BT::InputPort<int>("game_status"),
			BT::InputPort<bool>("target_valid")
	};
}

BT::NodeStatus CheckDefendCondition::tick()
{
	auto blackboard = config().blackboard;

	// Blackboard data skeleton for later business logic.
	try
	{
		const auto health = blackboard->get<float>("health");
		const auto game_status = blackboard->get<int>("game_status");
		const auto target_valid = blackboard->get<bool>("target_valid");
		(void)health;
		(void)game_status;
		(void)target_valid;
	}
	catch(...)
	{
	}

	return BT::NodeStatus::SUCCESS;
}

CheckAttackCondition::CheckAttackCondition(const std::string& name, const BT::NodeConfiguration& config)
	: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAttackCondition::providedPorts()
{
	return {
			BT::InputPort<float>("health"),
			BT::InputPort<int>("game_status"),
			BT::InputPort<bool>("target_valid"),
			BT::InputPort<bool>("enemy_outpost_destroyed")
	};
}

BT::NodeStatus CheckAttackCondition::tick()
{
	auto blackboard = config().blackboard;

	// Blackboard data skeleton for later business logic.
	try
	{
		const auto health = blackboard->get<float>("health");
		const auto game_status = blackboard->get<int>("game_status");
		const auto target_valid = blackboard->get<bool>("target_valid");
		const auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
		(void)health;
		(void)game_status;
		(void)target_valid;
		(void)enemy_outpost_destroyed;
	}
	catch(...)
	{
	}

	return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
