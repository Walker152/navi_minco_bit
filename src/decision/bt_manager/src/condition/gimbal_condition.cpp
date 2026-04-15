#include "bt_manager/condition/gimbal_condition.hpp"
#include <chrono>
using namespace color_text;

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/nav_zone.hpp"

#include <cmath>
#include <iostream>

namespace Sentry_BT {
CheckTargetVisible::CheckTargetVisible(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetVisible::providedPorts()
{
  return {BT::InputPort<bool>("target_valid"),
    BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),
    BT::InputPort<float>("gimbal_yaw")};
}

BT::NodeStatus CheckTargetVisible::tick()
{
  auto blackboard = config().blackboard;

  try {
    const auto target_valid = blackboard->get<bool>("target_valid");
    static bool last_target_valid = !target_valid;
    if (target_valid != last_target_valid) {
      std::cout << "CheckTargetVisible => " << (target_valid ? "VISIBLE" : "LOST") << std::endl;
      last_target_valid = target_valid;
    }
    return target_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
}

CheckNearEnemyOutpost::CheckNearEnemyOutpost(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckNearEnemyOutpost::providedPorts()
{
  return {};
}

BT::NodeStatus CheckNearEnemyOutpost::tick()
{
  auto blackboard = config().blackboard;
  const auto pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const bool near_outpost = enemy_outpost_watch_zone.contains({pose.position.x, pose.position.y, 0.0});

  static bool last_near_outpost = !near_outpost;
  if (near_outpost != last_near_outpost) {
    std::cout << "CheckNearEnemyOutpost => " << (near_outpost ? "NEAR" : "FAR") << std::endl;
    last_near_outpost = near_outpost;
  }

  return near_outpost ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
