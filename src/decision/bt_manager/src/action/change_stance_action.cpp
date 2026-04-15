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
  return {};
}

BT::NodeStatus ChangeStance::tick()
{
  auto stance_to_string = [](Sentry_BT::SentryStance stance) -> std::string {
    const auto index = static_cast<size_t>(stance);
    if (index < stance_names.size()) {
      return stance_names[index];
    }
    return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
  };

  auto blackboard = config().blackboard;
  Sentry_BT::SentryStance desired_stance;
  Sentry_BT::SentryStance current_stance;
  try {
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
