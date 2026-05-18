#include "bt_manager/action/gimbal_action.hpp"

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

  const auto target_valid = blackboard->get<bool>("target_valid");
  const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  if (!target_valid) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackTargetAction::onRunning()
{
  auto blackboard = config().blackboard;

  const auto target_valid = blackboard->get<bool>("target_valid");
  if (!target_valid) {
    return BT::NodeStatus::FAILURE;
  }
  const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  float target_yaw = std::atan2(target_pose.position.y - current_pose.position.y,
                       target_pose.position.x - current_pose.position.x) *
                     180.0f / static_cast<float>(M_PI);
  float gimbal_aim_yaw = 0.0f;
  auto transform_utils = blackboard->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  if (transform_utils) {
    transform_utils->transformYaw(target_yaw, gimbal_aim_yaw, "body", "map");
  } else {
    gimbal_aim_yaw = target_yaw;
  }
  blackboard->set<float>("scan_yaw_min_deg", gimbal_aim_yaw - 15.0f);
  blackboard->set<float>("scan_yaw_max_deg", gimbal_aim_yaw + 15.0f);
  return BT::NodeStatus::RUNNING;
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
  auto mode = blackboard->get<TacticalMode>("tactical_mode");
  auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");

  Point2D current_pt{current_pose.position.x, current_pose.position.y, 0.0};
  bool is_in_zone = false;
  PatrolZoneType target_zone;

  if (enemy_defense_zone.contains(current_pt)) {
    target_zone = PatrolZoneType::ENEMY_DEFENSE;
    is_in_zone = true;
  } else if (own_defense_zone.contains(current_pt)) {
    target_zone = PatrolZoneType::OWN_DEFENSE;
    is_in_zone = true;
  } else if (highland_zone.contains(current_pt)) {
    target_zone = PatrolZoneType::HIGHLAND;
    is_in_zone = true;
  }

  if (is_in_zone) {
    auto config = tactical_area_gimbal_map[mode][target_zone];
    auto tf_utils = blackboard->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
    float transformed_yaw = 0.0f;
    if (tf_utils) {
      tf_utils->transformYaw(config.scan_yaw_min, transformed_yaw, "body", "map");
      config.scan_yaw_min = transformed_yaw;
      tf_utils->transformYaw(config.scan_yaw_max, transformed_yaw, "body", "map");
      config.scan_yaw_max = transformed_yaw;
    }
    blackboard->set<float>("scan_yaw_min_deg", config.scan_yaw_min);
    blackboard->set<float>("scan_yaw_max_deg", config.scan_yaw_max);
    blackboard->set<bool>("use_limited_scan", true);
  } else {
    blackboard->set<float>("scan_yaw_min_deg", -180.0f);
    blackboard->set<float>("scan_yaw_max_deg", 180.0f);
    blackboard->set<bool>("use_limited_scan", false);
  }

  blackboard->set<PitchPos>("pitch_mode", PitchPos::DOWN);
  std::ostringstream oss;
  oss << "scan_yaw_range=["
      << (is_in_zone ? std::to_string(tactical_area_gimbal_map[mode][target_zone].scan_yaw_min) : "-180.0")
      << ", "
      << (is_in_zone ? std::to_string(tactical_area_gimbal_map[mode][target_zone].scan_yaw_max) : "180.0")
      << "]";
  detail::logTransition(detail::TreeKind::GIMBAL, "SetGimbalPoseByAreaAction", is_in_zone, oss.str(), "");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
