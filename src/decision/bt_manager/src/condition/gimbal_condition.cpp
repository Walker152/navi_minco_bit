#include "bt_manager/condition/gimbal_condition.hpp"

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

}  // namespace Sentry_BT
