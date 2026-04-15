#include "bt_manager/action/gimbal_action.hpp"

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/nav_zone.hpp"

#include <iostream>

namespace Sentry_BT {
TrackTargetAction::TrackTargetAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList TrackTargetAction::providedPorts()
{
  return {BT::InputPort<bool>("target_valid"),
    BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),
    BT::InputPort<float>("gimbal_yaw")};
}

BT::NodeStatus TrackTargetAction::onStart()
{
  auto blackboard = config().blackboard;

  try {
    const auto target_valid = blackboard->get<bool>("target_valid");
    const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    if (!target_valid) {
      return BT::NodeStatus::FAILURE;
    }
    blackboard->set<float>("target_gimbal_pan", static_cast<float>(target_pose.position.x));
    blackboard->set<float>("target_gimbal_tilt", static_cast<float>(target_pose.position.z));
    return BT::NodeStatus::RUNNING;
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus TrackTargetAction::onRunning()
{
  auto blackboard = config().blackboard;

  try {
    const auto target_valid = blackboard->get<bool>("target_valid");
    if (!target_valid) {
      return BT::NodeStatus::FAILURE;
    }
    const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    blackboard->set<float>("target_gimbal_pan", static_cast<float>(target_pose.position.x));
    blackboard->set<float>("target_gimbal_tilt", static_cast<float>(target_pose.position.z));
    return BT::NodeStatus::RUNNING;
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
}

void TrackTargetAction::onHalted()
{
  // Reserved for future stop/cleanup logic when tracking is interrupted.
}

SetGimbalPose::SetGimbalPose(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetGimbalPose::providedPorts()
{
  return {BT::InputPort<float>("pan"), BT::InputPort<float>("tilt")};
}

BT::NodeStatus SetGimbalPose::tick()
{
  auto blackboard = config().blackboard;

  const auto pan = getInput<float>("pan");
  const auto tilt = getInput<float>("tilt");

  const float target_pan = pan ? pan.value() : 0.0f;
  const float target_tilt = tilt ? tilt.value() : 0.0f;

  blackboard->set<float>("target_gimbal_pan", target_pan);
  blackboard->set<float>("target_gimbal_tilt", target_tilt);
  return BT::NodeStatus::SUCCESS;
}

SetGimbalPoseByAreaAction::SetGimbalPoseByAreaAction(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetGimbalPoseByAreaAction::providedPorts()
{
  return {};
}

BT::NodeStatus SetGimbalPoseByAreaAction::tick()
{
  auto blackboard = config().blackboard;
  const auto pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  TacticalMode tactical_mode = TacticalMode::BALANCED;
  blackboard->get<TacticalMode>("tactical_mode", tactical_mode);

  float yaw_min = -180.0f;
  float yaw_max = 180.0f;
  float pitch = 0.0f;

  const Point2D p{pose.position.x, pose.position.y, 0.0};
  if (highland_zone.contains(p)) {
    yaw_min = -50.0f;
    yaw_max = 50.0f;
    pitch = 15.0f;
  } else if (own_defense_zone.contains(p)) {
    yaw_min = -120.0f;
    yaw_max = 20.0f;
    pitch = 8.0f;
  } else if (enemy_defense_zone.contains(p)) {
    yaw_min = -20.0f;
    yaw_max = 120.0f;
    pitch = 5.0f;
  } else {
    auto gimbal_it = tactical_gimbal_map.find(tactical_mode);
    if (gimbal_it != tactical_gimbal_map.end() && !gimbal_it->second.empty()) {
      const auto & rule = gimbal_it->second.front();
      yaw_min = rule.yaw_lower_bound_deg;
      yaw_max = rule.yaw_upper_bound_deg;
      pitch = rule.pitch_up ? 12.0f : 0.0f;
    }
  }

  blackboard->set<float>("scan_yaw_min_deg", yaw_min);
  blackboard->set<float>("scan_yaw_max_deg", yaw_max);
  blackboard->set<float>("scan_pitch_deg", pitch);

  static float last_yaw_min = 0.0f;
  static float last_yaw_max = 0.0f;
  static float last_pitch = 0.0f;
  if (yaw_min != last_yaw_min || yaw_max != last_yaw_max || pitch != last_pitch) {
    std::cout << "SetGimbalPoseByAreaAction => yaw[" << yaw_min << ", " << yaw_max << "], pitch=" << pitch
              << std::endl;
    last_yaw_min = yaw_min;
    last_yaw_max = yaw_max;
    last_pitch = pitch;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
