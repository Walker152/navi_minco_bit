#include "bt_manager/condition/gimbal_condition.hpp"
#include <chrono>
using namespace color_text;

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
	geometry_msgs::msg::Pose target_pose;
  float gimbal_yaw = 0.0f;
 	bool target_valid = false;
  TacticalMode current_tactical_mode = TacticalMode::BALANCED;
	// Blackboard data skeleton for later business logic.
	try
	{
		target_valid = blackboard->get<bool>("target_valid");
		target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
		gimbal_yaw = blackboard->get<float>("gimbal_yaw");
    //current_tactical_mode = static_cast<TacticalMode>(blackboard->get<int>("current_tactical_mode")); // 先不区分战术模式，后续根据需要添加
	}
	catch(...)
	{
	}
	target_pose_ = target_pose;
	gimbal_yaw_ = gimbal_yaw;
	target_valid_ = target_valid;
  //current_tactical_mode_ = current_tactical_mode; // 先不区分战术模式，后续根据需要添加

	return BT::NodeStatus::SUCCESS;
}

// --------------------- CheckWillThroughTunnel ----------------------
// CheckWillThroughTunnel::CheckWillThroughTunnel(const std::string& name, const BT::NodeConfiguration& config)
//     : BT::ConditionNode(name, config)
// {
//   // 构造函数：初始化节点，不需要复杂操作 
// }

// BT::PortsList CheckWillThroughTunnel::providedPorts()
// {
//   return {}; // 不需要输入端口，直接从黑板读取信息
// }

// BT::NodeStatus CheckWillThroughTunnel::tick()
// {
//   auto blackboard = config().blackboard;
//   bool will_through_tunnel = false;
//   auto lifter_current_pos = blackboard->get<int>("lifter_current_pos");
//   will_through_tunnel = blackboard->get<bool>("through_tunnel");
//   static bool last_state_ = !will_through_tunnel;
//   if (last_state_ != will_through_tunnel)
//   {
//     std::cout << WHITE << "CheckWillThroughTunnel => "
//             << (will_through_tunnel ? "WILL_THROUGH_TUNNEL" : "WILL_NOT_THROUGH_TUNNEL")
//             << RESET << std::endl;
//     last_state_ = will_through_tunnel;
//   }
  
//   if (will_through_tunnel) {
//     if (lifter_current_pos == 0) {
//       blackboard->set<int>("desired_lifter_pos", 1); // 设置目标升降位置为 1(bottom)，准备过隧道
//     }
//   }
//   else {
//     if (lifter_current_pos != 0) {
//       blackboard->set<int>("desired_lifter_pos", 0); // 设置目标升降位置为 0(top)，准备不通过隧道
//     }
//   }
//   return BT::NodeStatus::SUCCESS;
// }

}  // namespace Sentry_BT
