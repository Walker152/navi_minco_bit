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
  const auto enemy_outpost_remain = !blackboard->get<bool>("enemy_outpost_destroyed");
  auto ros_iface = blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  TacticalMode tactical_mode = TacticalMode::BALANCED;
  blackboard->get<TacticalMode>("tactical_mode", tactical_mode);

  float yaw_min = -180.0f;
  float yaw_max = 180.0f;
  float pitch_min = -10.0f;
  float pitch_max = 20.0f;

  const Point2D p{pose.position.x, pose.position.y, 0.0};
  auto gimbal_it = tactical_gimbal_map.find(tactical_mode);
  // if (highland_zone.contains(p)) {
  //   yaw_min = -50.0f;
  //   yaw_max = 50.0f;
  //   pitch_min = 15.0f;
  //   pitch_max = 60.0f;
  // } else if (own_defense_zone.contains(p)) {
  //   yaw_min = -120.0f;
  //   yaw_max = 20.0f;
  //   pitch_min = 8.0f;
  //   pitch_max = 60.0f;
  // } else if (enemy_defense_zone.contains(p)) {
  //   yaw_min = -20.0f;
  //   yaw_max = 120.0f;
  //   pitch_min = 5.0f;
  //   pitch_max = 60.0f;
  // } else {
  //   auto gimbal_it = tactical_gimbal_map.find(tactical_mode);
  //   if (gimbal_it != tactical_gimbal_map.end() && !gimbal_it->second.empty()) {
  //     const auto & rule = gimbal_it->second.front();
  //     yaw_min = rule.yaw_lower_bound_deg;
  //     yaw_max = rule.yaw_upper_bound_deg;
  //     pitch_min = rule.pitch_up ? 12.0f : 0.0f;
  //     pitch_max = rule.pitch_up ? 60.0f : 90.0f;
  //   }
  // }
  if (highland_zone.contains(p)) {
    if ((enemy_outpost_buff_zone.contains(p) || enemy_outpost_watch_zone.contains(p)) && enemy_outpost_remain) {
      // 前哨站区域内优先使用敌方前哨站的巡逻规则
      geometry_msgs::msg::Pose outpost_in_map_frame;
      outpost_in_map_frame.position.x = nav_points[2].x;
      outpost_in_map_frame.position.y = nav_points[2].y;
      geometry_msgs::msg::Pose outpost_in_body_frame =
      ros_iface->transformMapPose(outpost_in_map_frame, "body");
      float outpost_theta_rad =
      std::atan2(outpost_in_body_frame.position.y, outpost_in_body_frame.position.x);
      yaw_min = -50.0f + outpost_theta_rad * 180.0f / static_cast<float>(M_PI);
      yaw_max = 50.0f + outpost_theta_rad * 180.0f / static_cast<float>(M_PI);
      pitch_min = 15.0f;
      pitch_max = 60.0f;
    } else {
      if (gimbal_it != tactical_gimbal_map.end() && !gimbal_it->second.empty()) {
        const auto & rule = gimbal_it->second.front();
        yaw_min = rule.yaw_lower_bound_deg;
        yaw_max = rule.yaw_upper_bound_deg;
        pitch_min = rule.pitch_up ? 12.0f : -10.0f;
        pitch_max = rule.pitch_up ? 60.0f : 20.0f;
      }
    }
  } else if (own_defense_zone.contains(p)) {
    yaw_min = -180.0f;
    yaw_max = 180.0f;
  } else if (enemy_defense_zone.contains(p)) {
    yaw_min = -180.0f;
    yaw_max = 180.0f;
  }
  blackboard->set<float>("scan_yaw_min_deg", yaw_min);
  blackboard->set<float>("scan_yaw_max_deg", yaw_max);
  blackboard->set<float>("scan_pitch_min_deg", pitch_min);
  blackboard->set<float>("scan_pitch_max_deg", pitch_max);


  static float last_yaw_min = 0.0f;
  static float last_yaw_max = 0.0f;
  static float last_pitch_min = 0.0f;
  static float last_pitch_max = 0.0f;
  if (yaw_min != last_yaw_min || yaw_max != last_yaw_max || pitch_min != last_pitch_min || pitch_max != last_pitch_max) {
    std::cout << "SetGimbalPoseByAreaAction => yaw[" << yaw_min << ", " << yaw_max << "], pitch_min=" << pitch_min
              << ", pitch_max=" << pitch_max << std::endl;
    last_yaw_min = yaw_min;
    last_yaw_max = yaw_max;
    last_pitch_min = pitch_min;
    last_pitch_max = pitch_max;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
