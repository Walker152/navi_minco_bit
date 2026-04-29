#include "bt_manager/action/recovery_actions.hpp"

#include "bt_manager/utils/area.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <iostream>

namespace Sentry_BT {

SetTunnelRecoveryAttemptPoint::SetTunnelRecoveryAttemptPoint(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetTunnelRecoveryAttemptPoint::providedPorts()
{
  return {};
}

BT::NodeStatus SetTunnelRecoveryAttemptPoint::tick()
{
  auto blackboard = config().blackboard;
  const int tunnel_idx = blackboard->get<int>("current_transform_zone_index");
  if (tunnel_idx < 0 || tunnel_idx >= static_cast<int>(tunnel_recovery_configs.size())) {
    std::cout << "[RECOVERY_TREE] SetTunnelRecoveryAttemptPoint invalid tunnel_idx=" << tunnel_idx
              << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto & goal = tunnel_recovery_configs[static_cast<std::size_t>(tunnel_idx)].forward_point;
  blackboard->set("nav_goal", goal);
  std::cout << "[RECOVERY_TREE] attempt point tunnel_idx=" << tunnel_idx << " goal=(" << goal.x << ", "
            << goal.y << ")" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

SetTunnelRecoveryRetreatPoint::SetTunnelRecoveryRetreatPoint(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetTunnelRecoveryRetreatPoint::providedPorts()
{
  return {};
}

BT::NodeStatus SetTunnelRecoveryRetreatPoint::tick()
{
  auto blackboard = config().blackboard;
  const int tunnel_idx = blackboard->get<int>("current_transform_zone_index");
  if (tunnel_idx < 0 || tunnel_idx >= static_cast<int>(tunnel_recovery_configs.size())) {
    std::cout << "[RECOVERY_TREE] SetTunnelRecoveryRetreatPoint invalid tunnel_idx=" << tunnel_idx
              << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  const auto & goal = tunnel_recovery_configs[static_cast<std::size_t>(tunnel_idx)].recovery_point;
  blackboard->set("nav_goal", goal);
  std::cout << "[RECOVERY_TREE] retreat point tunnel_idx=" << tunnel_idx << " goal=(" << goal.x
            << ", " << goal.y << ")" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

SetGlobalVelocity::SetGlobalVelocity(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetGlobalVelocity::providedPorts()
{
  return {
    BT::InputPort<double>("global_vx", 0.0, "Global linear velocity x"),
    BT::InputPort<double>("global_vy", 0.0, "Global linear velocity y"),
    BT::InputPort<bool>("use_tunnel_profile", true, "Use tunnel-indexed recovery velocity profile"),
  };
}

BT::NodeStatus SetGlobalVelocity::tick()
{
  double global_vx = getInput<double>("global_vx").value_or(0.0);
  double global_vy = getInput<double>("global_vy").value_or(0.0);
  const bool use_tunnel_profile = getInput<bool>("use_tunnel_profile").value_or(true);

  if (use_tunnel_profile) {
    const int tunnel_idx = config().blackboard->get<int>("current_transform_zone_index");
    if (tunnel_idx >= 0 && tunnel_idx < static_cast<int>(tunnel_recovery_configs.size())) {
      const auto & cfg = tunnel_recovery_configs[static_cast<std::size_t>(tunnel_idx)];
      global_vx = static_cast<double>(cfg.recovery_vx);
      global_vy = static_cast<double>(cfg.recovery_vy);
    }
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = global_vx;
  cmd_vel.linear.y = global_vy;
  cmd_vel.angular.z = 0.0;

  config().blackboard->set("cmd_vel", cmd_vel);

  std::cout << "[RECOVERY_TREE] SetGlobalVelocity cmd_vel=(vx=" << global_vx << ", vy=" << global_vy
            << ", wz=0.0)" << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
