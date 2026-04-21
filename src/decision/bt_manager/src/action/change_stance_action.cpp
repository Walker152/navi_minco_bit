#include "bt_manager/action/change_stance_action.hpp"

#include <limits>
using namespace color_text;
namespace Sentry_BT {
std::chrono::time_point<std::chrono::system_clock> ChangeStance::last_change_time_ =
  std::chrono::time_point<std::chrono::system_clock>::min();

// ------------------- ChangeStance -------------------
ChangeStance::ChangeStance(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ChangeStance::providedPorts()
{
  return {BT::InputPort<std::string>("stance", "Target stance: ATTACK/DEFEND/MOVE"),
    BT::InputPort<bool>("use_gyro", "Whether to enable gyro mode"),
    BT::InputPort<float>("gyro_vel", "Gyro speed in rpm")};
}

BT::NodeStatus ChangeStance::tick()
{
  auto parse_stance = [](const std::string & value) -> Sentry_BT::SentryStance {
    if (value == "ATTACK" || value == "attack" || value == "1") {
      return Sentry_BT::SentryStance::ATTACK;
    }
    if (value == "DEFEND" || value == "defend" || value == "2") {
      return Sentry_BT::SentryStance::DEFEND;
    }
    if (value == "MOVE" || value == "move" || value == "3") {
      return Sentry_BT::SentryStance::MOVE;
    }
    return Sentry_BT::SentryStance::DEFEND;
  };

  auto stance_to_string = [](Sentry_BT::SentryStance stance) -> std::string {
    const auto index = static_cast<size_t>(stance);
    if (index >= 1 && index <= stance_names.size()) {
      return stance_names[index - 1];
    }
    return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
  };

  auto blackboard = config().blackboard;
  Sentry_BT::SentryStance desired_stance;
  Sentry_BT::SentryStance current_stance;
  try {
    const std::string stance_str = getInput<std::string>("stance").value_or("DEFEND");
    blackboard->set<Sentry_BT::SentryStance>("desired_stance", parse_stance(stance_str));

    const bool use_gyro =
      getInput<bool>("use_gyro").value_or(blackboard->get<bool>("use_gyro_mode"));
    blackboard->set("use_gyro_mode", use_gyro);

    const float gyro_vel =
      getInput<float>("gyro_vel").value_or(blackboard->get<float>("gyro_vel"));
    blackboard->set("gyro_vel", gyro_vel);

    desired_stance = blackboard->get<Sentry_BT::SentryStance>("desired_stance");
    current_stance = blackboard->get<Sentry_BT::SentryStance>("current_stance");
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }

  if (current_stance == desired_stance) {
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = std::chrono::system_clock::now();
  const double elapsed_seconds =
    (last_change_time_ == std::chrono::time_point<std::chrono::system_clock>::min())
      ? std::numeric_limits<double>::infinity()
      : std::chrono::duration<double>(now - last_change_time_).count();

  if (elapsed_seconds < 5.0) {
    return BT::NodeStatus::SUCCESS;
  }

  std::cout << GREEN << "Change from stance " << stance_to_string(current_stance) << " to stance "
            << stance_to_string(desired_stance) << RESET << std::endl;
  blackboard->set<Sentry_BT::SentryStance>("current_stance", desired_stance);
  last_change_time_ = now;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
