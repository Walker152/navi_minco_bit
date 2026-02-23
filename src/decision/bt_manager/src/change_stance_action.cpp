#include "bt_manager/change_stance_action.hpp"

#include <limits>

namespace Sentry_BT
{
std::chrono::time_point<std::chrono::system_clock> ChangeStance::last_change_time_ =
    std::chrono::time_point<std::chrono::system_clock>::min();

// ------------------- ChangeStance -------------------
ChangeStance::ChangeStance(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ChangeStance::providedPorts()
{
  return {
      BT::InputPort<Sentry_BT::SentryStance>("desired_stance"),
      BT::InputPort<Sentry_BT::SentryStance>("current_stance"),
      BT::InputPort<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface")};
}

BT::NodeStatus ChangeStance::tick()
{
  auto desired_stance_input = getInput<Sentry_BT::SentryStance>("desired_stance");
  if(!desired_stance_input)
  {
    return BT::NodeStatus::FAILURE;
  }

  auto current_stance_input = getInput<Sentry_BT::SentryStance>("current_stance");
  if(!current_stance_input)
  {
    return BT::NodeStatus::FAILURE;
  }

  auto ros_iface_input = getInput<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  if(!ros_iface_input || !ros_iface_input.value())
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto desired_stance = desired_stance_input.value();
  const auto current_stance = current_stance_input.value();
  auto ros_iface = ros_iface_input.value();

  if(current_stance == desired_stance)
  {
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = std::chrono::system_clock::now();
  const double elapsed_seconds =
      (last_change_time_ == std::chrono::time_point<std::chrono::system_clock>::min())
          ? std::numeric_limits<double>::infinity()
          : std::chrono::duration<double>(now - last_change_time_).count();

  if(elapsed_seconds < 5.0)
  {
    return BT::NodeStatus::FAILURE;
  }

  ros_iface->publishPosition(static_cast<int>(desired_stance));
  last_change_time_ = now;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
