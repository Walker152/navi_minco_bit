#pragma once

#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>

namespace Sentry_BT {
class ChangeStance : public BT::SyncActionNode
{
public:
  ChangeStance(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static std::chrono::time_point<std::chrono::system_clock> last_change_time_;
};

}  // namespace Sentry_BT
