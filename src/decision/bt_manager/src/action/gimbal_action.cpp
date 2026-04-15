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

  // Blackboard data skeleton for future gimbal tracking logic.
  try {
    const auto target_valid = blackboard->get<bool>("target_valid");
    const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    const auto gimbal_yaw = blackboard->get<float>("gimbal_yaw");
    (void)target_valid;
    (void)target_pose;
    (void)gimbal_yaw;
  } catch (...) {
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackTargetAction::onRunning()
{
  auto blackboard = config().blackboard;

  // Blackboard data skeleton for future continuous tracking updates.
  try {
    const auto target_valid = blackboard->get<bool>("target_valid");
    const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    const auto gimbal_yaw = blackboard->get<float>("gimbal_yaw");
    (void)target_valid;
    (void)target_pose;
    (void)gimbal_yaw;
  } catch (...) {
  }

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

  blackboard->set<float>("target_gimbal_pan", target_pan);
  blackboard->set<float>("target_gimbal_tilt", target_tilt);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
