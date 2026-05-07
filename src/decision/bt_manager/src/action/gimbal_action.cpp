#include "bt_manager/action/gimbal_action.hpp"

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/nav_zone.hpp"

#include <algorithm>
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
  const auto pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto enemy_outpost_remain = !blackboard->get<bool>("enemy_outpost_destroyed");
  const auto tactical_mode = blackboard->get<TacticalMode>("tactical_mode");

  float yaw_min = -180.0f;
  float yaw_max = 180.0f;
  PitchPos pitch_mode = PitchPos::DOWN;

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
    if ((enemy_outpost_buff_zone.contains(p) || enemy_outpost_watch_zone.contains(p)) &&
        enemy_outpost_remain) {
      // 前哨站区域内优先使用敌方前哨站的巡逻规则
      geometry_msgs::msg::Pose outpost_in_map_frame;
      outpost_in_map_frame.position.x = nav_points[2].x;
      outpost_in_map_frame.position.y = nav_points[2].y;
      geometry_msgs::msg::Pose outpost_in_body_frame;
      auto transform_utils = blackboard->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
      if (transform_utils) {
        transform_utils->transformMapPose(outpost_in_map_frame, outpost_in_body_frame, "body");
      }
      float outpost_theta_rad =
        std::atan2(outpost_in_body_frame.position.y, outpost_in_body_frame.position.x);
      yaw_min = -50.0f + outpost_theta_rad * 180.0f / static_cast<float>(M_PI);
      yaw_max = 50.0f + outpost_theta_rad * 180.0f / static_cast<float>(M_PI);
      pitch_mode = PitchPos::UP;  // 高地区域优先抬头观察
    } else {
      if (gimbal_it != tactical_gimbal_map.end() && !gimbal_it->second.empty()) {
        const auto & rule = gimbal_it->second.front();
        yaw_min = rule.yaw_lower_bound_deg;
        yaw_max = rule.yaw_upper_bound_deg;
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
  blackboard->set<PitchPos>("pitch_mode", pitch_mode);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
