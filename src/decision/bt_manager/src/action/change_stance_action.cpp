#include "bt_manager/action/change_stance_action.hpp"

using namespace color_text;
namespace Sentry_BT {
std::chrono::time_point<std::chrono::system_clock> ChangeStance::last_change_time_ =
  std::chrono::time_point<std::chrono::system_clock>::min();

// ------------------- SetGyroState -------------------
SetGyroState::SetGyroState(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetGyroState::providedPorts()
{
  return {BT::InputPort<bool>("use_gyro", "Whether to enable gyro mode"),
    BT::InputPort<float>("gyro_vel", "Gyro speed in rpm"),
    BT::InputPort<float>("tunnel_speed_y", "tunnel_speed_y")};
}

BT::NodeStatus SetGyroState::tick()
{
  auto blackboard = config().blackboard;
  const bool use_gyro = getInput<bool>("use_gyro").value_or(blackboard->get<bool>("use_gyro_mode"));
  const float gyro_vel = getInput<float>("gyro_vel").value_or(blackboard->get<float>("gyro_vel"));
  const float tunnel_speed_y = getInput<float>("tunnel_speed_y").value_or(blackboard->get<float>("tunnel_speed_y"));
  blackboard->set("use_gyro_mode", use_gyro);
  blackboard->set("gyro_vel", gyro_vel);
  blackboard->set("tunnel_speed_y", tunnel_speed_y);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- ChangeStance -------------------
ChangeStance::ChangeStance(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ChangeStance::providedPorts()
{
  return {BT::InputPort<std::string>("stance", "Target stance: ATTACK/DEFEND/MOVE")};
}

BT::NodeStatus ChangeStance::onStart()
{
  auto parse_stance = [](const std::string & value) -> Sentry_BT::SentryStance {
    if (value == "ATTACK" || value == "attack" || value == "1") {
      return SentryStance::ATTACK;
    }
    if (value == "DEFEND" || value == "defend" || value == "2") {
      return SentryStance::DEFEND;
    }
    if (value == "MOVE" || value == "move" || value == "3") {
      return SentryStance::MOVE;
    }
    return SentryStance::DEFEND;
  };

  auto blackboard = config().blackboard;
  const std::string stance_str = getInput<std::string>("stance").value_or("DEFEND");
  desired_stance_ = parse_stance(stance_str);
  current_stance_ = blackboard->get<SentryStance>("current_stance");

  if (current_stance_ == desired_stance_) {
    return BT::NodeStatus::SUCCESS;
  }
  return applyStanceChange();
}

BT::NodeStatus ChangeStance::onRunning()
{
  return applyStanceChange();
}

void ChangeStance::onHalted()
{
}

BT::NodeStatus ChangeStance::applyStanceChange()
{
  auto stance_to_string = [](SentryStance stance) -> std::string {
    const auto index = static_cast<size_t>(stance);
    if (index >= 1 && index <= stance_names.size()) {
      return stance_names[index - 1];
    }
    return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
  };

  auto blackboard = config().blackboard;
  current_stance_ = blackboard->get<SentryStance>("current_stance");
  if (current_stance_ == desired_stance_) {
    return BT::NodeStatus::SUCCESS;
  }

  blackboard->set<SentryStance>("desired_stance", desired_stance_);
  // blackboard->set<SentryStance>("current_stance", desired_stance_);
  // std::cout << MAGENTA << "[STANCE_TREE]" << GREEN << "Change from stance "
  //           << stance_to_string(current_stance_) << " to stance " << stance_to_string(desired_stance_)
  //           << RESET << std::endl;
  last_change_time_ = std::chrono::system_clock::now();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
