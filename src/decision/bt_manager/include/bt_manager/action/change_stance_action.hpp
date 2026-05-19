#pragma once

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>
#include <cstddef>
#include <random>

namespace Sentry_BT {
class SetGyroState : public BT::SyncActionNode
{
public:
  SetGyroState(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool shouldReverseRotation(bool use_gyro, int ammo_purchase_total, int bullets_remaining);

  bool random_speed_enabled_{false};
  bool random_initialized_{false};
  float current_gyro_vel_{0.0f};
  std::size_t current_speed_index_{0};
  std::chrono::steady_clock::time_point last_speed_change_time_;
  bool reverse_initialized_{false};
  std::chrono::steady_clock::time_point reverse_start_time_;
  std::mt19937 rng_{std::random_device{}()};
};

class TunnelGyroAlignAction : public BT::ActionNodeBase
{
public:
  TunnelGyroAlignAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  float computeTunnelGyroVelPid(double yaw_error,
    float kp,
    float ki,
    float kd,
    float deadzone,
    float max_abs_gyro_vel,
    const std::chrono::steady_clock::time_point & now);
  void resetPidState();

  bool pid_initialized_ = false;
  double integral_error_ = 0.0;
  double last_error_ = 0.0;
  std::chrono::steady_clock::time_point last_pid_time_;
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
