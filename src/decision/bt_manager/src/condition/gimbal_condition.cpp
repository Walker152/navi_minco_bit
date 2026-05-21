#include "bt_manager/condition/gimbal_condition.hpp"
#include <cmath>
#include <sstream>

namespace Sentry_BT {
CheckTargetVisible::CheckTargetVisible(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetVisible::providedPorts()
{
  return {BT::InputPort<bool>("target_valid"),
    BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),
    BT::InputPort<float>("gimbal_yaw"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckTargetVisible::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto target_valid = blackboard->get<bool>("target_valid");
  detail::logTransition(detail::TreeKind::GIMBAL, "CheckTargetVisible", target_valid, "", branch);
  return target_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckNearEnemyOutpost::CheckNearEnemyOutpost(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckNearEnemyOutpost::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckNearEnemyOutpost::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const Sentry_BT::Point2D nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");
  const auto pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
  if (enemy_outpost_destroyed) {
    detail::logTransition(detail::TreeKind::GIMBAL, "CheckNearEnemyOutpost", false, "enemy outpost destroyed", branch);
    return BT::NodeStatus::FAILURE;
  }
  const bool near_outpost = enemy_outpost_watch_zone.contains({pose.position.x, pose.position.y, 0.0});

  std::ostringstream oss;
  oss << "pos=(" << pose.position.x << ", " << pose.position.y << ")";
  detail::logTransition(detail::TreeKind::GIMBAL, "CheckNearEnemyOutpost", near_outpost, oss.str(), branch);

  return near_outpost ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
