#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose.hpp>

namespace Sentry_BT {
class CheckTargetVisible : public BT::ConditionNode
{
public:
  CheckTargetVisible(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckNearEnemyOutpost : public BT::ConditionNode
{
public:
  CheckNearEnemyOutpost(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  geometry_msgs::msg::Pose target_pose_;
  float gimbal_yaw_;
  bool target_valid_;
  TacticalMode current_tactical_mode_;
};

// 检查是否将要过隧道
// class CheckWillThroughTunnel : public BT::ConditionNode
// {
// public:
//   CheckWillThroughTunnel(const std::string& name, const BT::NodeConfiguration& config);
//   static BT::PortsList providedPorts();
//   BT::NodeStatus tick() override;
// private:
//   static bool last_state_;
// };
}  // namespace Sentry_BT
