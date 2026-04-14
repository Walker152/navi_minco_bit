#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace Sentry_BT
{
class CheckDefendCondition : public BT::ConditionNode
{
public:
	CheckDefendCondition(const std::string& name, const BT::NodeConfiguration& config);

	static BT::PortsList providedPorts();
	BT::NodeStatus tick() override;
};

class CheckAttackCondition : public BT::ConditionNode
{
public:
	CheckAttackCondition(const std::string& name, const BT::NodeConfiguration& config);

	static BT::PortsList providedPorts();
	BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT

