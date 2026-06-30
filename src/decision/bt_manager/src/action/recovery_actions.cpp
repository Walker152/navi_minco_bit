#include "bt_manager/action/recovery_actions.hpp"

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <sstream>

namespace Sentry_BT {

TunnelTimeoutBackoutAction::TunnelTimeoutBackoutAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList TunnelTimeoutBackoutAction::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_s", 6.0, "Time in tunnel before backout starts"),
    BT::InputPort<double>("speed", 0.5, "Backout speed"),
    BT::InputPort<double>("max_backout_s", 12.0, "Maximum backout duration"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
  };
}

BT::NodeStatus TunnelTimeoutBackoutAction::onStart()
{
  // A pre-timeout FAILURE re-enters onStart on the next tree tick. Keep member timing intact.
  return onRunning();
}

BT::NodeStatus TunnelTimeoutBackoutAction::onRunning()
{
  const double timeout_s = getInput<double>("timeout_s").value_or(6.0);
  const double speed = getInput<double>("speed").value_or(0.5);
  const double max_backout_s = getInput<double>("max_backout_s").value_or(12.0);
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto now = std::chrono::steady_clock::now();
  const auto current_pose = config().blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const int tunnel_idx = findTunnelIndexByPose(current_pose);

  if (tunnel_idx < 0) {
    clearCmdVel();
    if (active_backout_) {
      geometry_msgs::msg::Twist stopped;
      logState(false, active_tunnel_idx_,
        std::chrono::duration<double>(now - entry_time_).count(), timeout_s, stopped,
        "exit_tunnel", branch);
      resetState();
      return BT::NodeStatus::SUCCESS;
    }
    resetState();
    geometry_msgs::msg::Twist stopped;
    logState(false, -1, 0.0, timeout_s, stopped, "outside_tunnel", branch);
    return BT::NodeStatus::FAILURE;
  }

  if (!was_in_tunnel_ || tunnel_idx != active_tunnel_idx_) {
    clearCmdVel();
    was_in_tunnel_ = true;
    active_backout_ = false;
    active_tunnel_idx_ = tunnel_idx;
    entry_time_ = now;
    backout_start_time_ = {};
    geometry_msgs::msg::Twist stopped;
    logState(false, tunnel_idx, 0.0, timeout_s, stopped, "tunnel_entry_or_index_change", branch);
    return BT::NodeStatus::FAILURE;
  }

  const double duration = std::chrono::duration<double>(now - entry_time_).count();
  if (!active_backout_ && duration < timeout_s) {
    clearCmdVel();
    geometry_msgs::msg::Twist stopped;
    logState(false, tunnel_idx, duration, timeout_s, stopped, "waiting_for_timeout", branch);
    return BT::NodeStatus::FAILURE;
  }

  if (!active_backout_) {
    active_backout_ = true;
    backout_start_time_ = now;
  }

  const double backout_duration =
    std::chrono::duration<double>(now - backout_start_time_).count();
  if (backout_duration > max_backout_s) {
    clearCmdVel();
    geometry_msgs::msg::Twist stopped;
    logState(false, tunnel_idx, duration, timeout_s, stopped, "max_backout_s_exceeded", branch);
    resetState();
    return BT::NodeStatus::FAILURE;
  }

  const auto cmd_vel = computeBackoutVelocity(tunnel_idx, speed, current_pose);
  setTunnelEscapeCommand(true, cmd_vel);
  logState(true, tunnel_idx, duration, timeout_s, cmd_vel, "active_backout", branch);
  return BT::NodeStatus::RUNNING;
}

void TunnelTimeoutBackoutAction::onHalted()
{
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double timeout_s = getInput<double>("timeout_s").value_or(6.0);
  const int tunnel_idx = active_tunnel_idx_;
  clearCmdVel();
  resetState();
  geometry_msgs::msg::Twist stopped;
  logState(false, tunnel_idx, 0.0, timeout_s, stopped, "halted", branch);
}

int TunnelTimeoutBackoutAction::findTunnelIndexByPose(
  const geometry_msgs::msg::Pose & pose) const
{
  const Point2D point{pose.position.x, pose.position.y, 0.0};
  for (std::size_t i = 0; i < tunnel_zone.size(); ++i) {
    if (tunnel_zone[i].contains(point)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

geometry_msgs::msg::Twist TunnelTimeoutBackoutAction::computeBackoutVelocity(
  const int tunnel_idx, const double speed,
  const geometry_msgs::msg::Pose & current_pose) const
{
  geometry_msgs::msg::Twist cmd_vel;
  const bool valid_tunnel_idx =
    tunnel_idx >= 0 && tunnel_idx < static_cast<int>(tunnel_recovery_configs.size());

  double yaw = 0.0;
  if (valid_tunnel_idx) {
    yaw = static_cast<double>(
      tunnel_recovery_configs[static_cast<std::size_t>(tunnel_idx)].tunnel_pass_yaw_target_rad);
  } else {
    const auto & q = current_pose.orientation;
    yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  const double v = std::abs(speed);
  cmd_vel.linear.x = v * std::cos(yaw);
  cmd_vel.linear.y = -v * std::sin(yaw);
  cmd_vel.angular.z = 0.0;
  return cmd_vel;
}

void TunnelTimeoutBackoutAction::setTunnelEscapeCommand(
  const bool active,
  const geometry_msgs::msg::Twist & cmd_vel)
{
  config().blackboard->set("tunnel_escape_active", active);
  config().blackboard->set("tunnel_escape_cmd_vel", cmd_vel);
  config().blackboard->set("cmd_vel", cmd_vel);
}

void TunnelTimeoutBackoutAction::clearCmdVel()
{
  geometry_msgs::msg::Twist stopped;
  setTunnelEscapeCommand(false, stopped);
}

void TunnelTimeoutBackoutAction::resetState()
{
  was_in_tunnel_ = false;
  active_backout_ = false;
  active_tunnel_idx_ = -1;
  entry_time_ = {};
  backout_start_time_ = {};
}

void TunnelTimeoutBackoutAction::logState(const bool active, const int tunnel_idx,
  const double duration, const double timeout_s, const geometry_msgs::msg::Twist & cmd_vel,
  const std::string & reason, const std::string & branch) const
{
  std::ostringstream oss;
  oss << std::boolalpha << "node=TunnelTimeoutBackoutAction"
      << ", active=" << active << ", tunnel_idx=" << tunnel_idx << ", duration=" << duration
      << ", timeout_s=" << timeout_s << ", active_backout=" << active_backout_
      << ", cmd_vel=(" << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", "
      << cmd_vel.angular.z << "), reason=" << reason << ", branch=" << branch;
  detail::logTransition(
    detail::TreeKind::RECOVERY, "TunnelTimeoutBackoutAction", active, oss.str(), branch);
}

}  // namespace Sentry_BT
