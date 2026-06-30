#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>

namespace Sentry_BT {

class TunnelTimeoutBackoutAction : public BT::StatefulActionNode
{
public:
  TunnelTimeoutBackoutAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int findTunnelIndexByPose(const geometry_msgs::msg::Pose & pose) const;
  geometry_msgs::msg::Twist computeBackoutVelocity(
    int tunnel_idx, double speed,
    const geometry_msgs::msg::Pose & current_pose) const;
  void setTunnelEscapeCommand(
    bool active,
    const geometry_msgs::msg::Twist & cmd_vel);
  void clearCmdVel();
  void resetState();
  void logState(bool active, int tunnel_idx, double duration, double timeout_s,
    const geometry_msgs::msg::Twist & cmd_vel, const std::string & reason,
    const std::string & branch) const;

  bool was_in_tunnel_ = false;
  bool active_backout_ = false;
  int active_tunnel_idx_ = -1;
  std::chrono::steady_clock::time_point entry_time_;
  std::chrono::steady_clock::time_point backout_start_time_;
};

}  // namespace Sentry_BT
