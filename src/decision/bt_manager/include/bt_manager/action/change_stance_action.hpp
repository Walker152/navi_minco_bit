#pragma once

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>

namespace Sentry_BT {
class SetGyroState : public BT::SyncActionNode
{
public:
  SetGyroState(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class ChangeStance : public BT::StatefulActionNode
{
public:
  ChangeStance(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static std::chrono::time_point<std::chrono::system_clock> getLastChangeTime()
  {
    return last_change_time_;
  }

private:
  BT::NodeStatus applyStanceChange();

  SentryStance desired_stance_{SentryStance::DEFEND};
  SentryStance current_stance_{SentryStance::DEFEND};

  static std::chrono::time_point<std::chrono::system_clock> last_change_time_;
};

class UpdateEnhanceTime : public BT::SyncActionNode
{
public:
  UpdateEnhanceTime(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  int last_game_time_{-1};
};

}  // namespace Sentry_BT
