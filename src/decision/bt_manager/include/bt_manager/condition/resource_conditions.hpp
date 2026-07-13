#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <limits>

namespace Sentry_BT {

// ------------------- 通用资源条件节点 -------------------

class CheckCoinRemaining : public BT::ConditionNode
{
public:
  CheckCoinRemaining(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckEngagedSafeResponse : public BT::ConditionNode
{
public:
  CheckEngagedSafeResponse(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  float last_health_ = std::numeric_limits<float>::max();
  std::chrono::steady_clock::time_point last_health_change_time_{};
  bool cooldown_active_ = false;
  bool initialized_ = false;
};

class CheckRemoteExchangeCooldown : public BT::ConditionNode
{
public:
  CheckRemoteExchangeCooldown(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  int last_exchange_count_ = 0;
  std::chrono::steady_clock::time_point last_exchange_time_{};
  bool initialized_ = false;
};

class CheckRemainingAmmoExchange : public BT::ConditionNode
{
public:
  CheckRemainingAmmoExchange(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckAttackFortHealthExchangeNeeded : public BT::ConditionNode
{
public:
  CheckAttackFortHealthExchangeNeeded(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool requested_ = false;
};

class CheckNormalExchangeCooldown : public BT::ConditionNode
{
public:
  CheckNormalExchangeCooldown(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool initialized_ = false;
  std::chrono::time_point<std::chrono::steady_clock> last_exchange_time_;
};

class CheckInZone : public BT::ConditionNode
{
public:
  CheckInZone(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckCanFreeResurrect : public BT::ConditionNode
{
public:
  CheckCanFreeResurrect(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckEnergyActive : public BT::ConditionNode
{
public:
  CheckEnergyActive(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class CheckCanActivateEnergy : public BT::ConditionNode
{
public:
  CheckCanActivateEnergy(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
