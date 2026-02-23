#include "bt_manager/change_stance_condition.hpp"

#include <iostream>

namespace Sentry_BT
{
// ------------------- CheckMPCondition -------------------
CheckMPCondition::CheckMPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckMPCondition::providedPorts()
{
  return {
      BT::InputPort<bool>("enemy_outpost_destroyed"),
      BT::InputPort<int>("current_mode"),
      BT::InputPort<int>("nav_status"),
      BT::InputPort<bool>("outpost_msg"),
      BT::InputPort<Sentry_BT::SentryStance>("current_stance"),
      BT::InputPort<float>("health"),
      BT::OutputPort<Sentry_BT::SentryStance>("desired_stance")};
}

BT::NodeStatus CheckMPCondition::tick()
{
  std::cout << "---------- CheckMPCondition ----------" << std::endl;
  auto msg_destroyed = getInput<bool>("enemy_outpost_destroyed");
  if(!msg_destroyed)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_current_mode = getInput<int>("current_mode");
  if(!msg_current_mode)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_nav_status = getInput<int>("nav_status");
  if(!msg_nav_status)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_outpost = getInput<bool>("outpost_msg");
  if(!msg_outpost)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_current_stance = getInput<Sentry_BT::SentryStance>("current_stance");
  if(!msg_current_stance)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_health = getInput<float>("health");
  if(!msg_health)
  {
    return BT::NodeStatus::FAILURE;
  }

  bool enemy_outpost_destroyed = msg_destroyed.value();
  int current_mode = msg_current_mode.value();
  int nav_status = msg_nav_status.value();
  bool outpost_msg = msg_outpost.value();
  auto current_stance = msg_current_stance.value();
  float current_health = msg_health.value();

  bool condition_met = ((!enemy_outpost_destroyed) && (current_mode == Sentry_BT::NavMode::RESPONSE) &&
                        (outpost_msg == false)) ||
                       ((current_mode == Sentry_BT::NavMode::RETREAT) && (nav_status == Sentry_BT::NavStatus::RUNNING)) ||
                       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::RUNNING));

  std::cout << "MP inputs => enemy_outpost_destroyed=" << enemy_outpost_destroyed << ", current_mode=" << current_mode
            << ", nav_status=" << nav_status << ", outpost_msg=" << outpost_msg
            << ", current_stance=" << static_cast<int>(current_stance) << ", health=" << current_health
            << ", result=" << (condition_met ? "SUCCESS" : "FAILURE") << std::endl;

  if(condition_met)
  {
    setOutput("desired_stance", Sentry_BT::SentryStance::MOVE);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckAPCondition -------------------
CheckAPCondition::CheckAPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAPCondition::providedPorts()
{
  return {
      BT::InputPort<bool>("enemy_outpost_destroyed"),
      BT::InputPort<int>("current_mode"),
      BT::InputPort<int>("nav_status"),
      BT::InputPort<bool>("outpost_msg"),
      BT::InputPort<Sentry_BT::SentryStance>("current_stance"),
      BT::InputPort<float>("health"),
      BT::OutputPort<Sentry_BT::SentryStance>("desired_stance")};
}

BT::NodeStatus CheckAPCondition::tick()
{
  std::cout << "---------- CheckAPCondition ----------" << std::endl;
  auto msg_destroyed = getInput<bool>("enemy_outpost_destroyed");
  if(!msg_destroyed)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_current_mode = getInput<int>("current_mode");
  if(!msg_current_mode)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_nav_status = getInput<int>("nav_status");
  if(!msg_nav_status)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_outpost = getInput<bool>("outpost_msg");
  if(!msg_outpost)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_current_stance = getInput<Sentry_BT::SentryStance>("current_stance");
  if(!msg_current_stance)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_health = getInput<float>("health");
  if(!msg_health)
  {
    return BT::NodeStatus::FAILURE;
  }

  bool enemy_outpost_destroyed = msg_destroyed.value();
  int current_mode = msg_current_mode.value();
  int nav_status = msg_nav_status.value();
  bool outpost_msg = msg_outpost.value();
  auto current_stance = msg_current_stance.value();
  float current_health = msg_health.value();

  bool condition_met = ((!enemy_outpost_destroyed) && (current_mode == Sentry_BT::NavMode::RESPONSE) &&
                        (outpost_msg == true)) ||
                       (current_mode == Sentry_BT::NavMode::TRACING) ||
                       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::IDLE)) ||
                       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::SUCCESS));

  if(condition_met)
  {
    setOutput("desired_stance", Sentry_BT::SentryStance::ATTACK);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckDPCondition -------------------
CheckDPCondition::CheckDPCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDPCondition::providedPorts()
{
  return {
      BT::InputPort<int>("current_mode"),
      BT::InputPort<int>("nav_status"),
      BT::InputPort<Sentry_BT::SentryStance>("current_stance"),
      BT::InputPort<float>("health"),
      BT::OutputPort<Sentry_BT::SentryStance>("desired_stance")};
}

BT::NodeStatus CheckDPCondition::tick()
{
  std::cout << "---------- CheckDPCondition ----------" << std::endl;
  auto msg_current_mode = getInput<int>("current_mode");
  if(!msg_current_mode)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_nav_status = getInput<int>("nav_status");
  if(!msg_nav_status)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_current_stance = getInput<Sentry_BT::SentryStance>("current_stance");
  if(!msg_current_stance)
  {
    return BT::NodeStatus::FAILURE;
  }
  auto msg_health = getInput<float>("health");
  if(!msg_health)
  {
    return BT::NodeStatus::FAILURE;
  }

  int current_mode = msg_current_mode.value();
  int nav_status = msg_nav_status.value();
  auto current_stance = msg_current_stance.value();
  float current_health = msg_health.value();

  bool condition_met =
      ((current_mode == Sentry_BT::NavStatus::FAILURE) && (current_health <= 30.0f) || current_health <= 25.0f);

  std::cout << "DP inputs => current_mode=" << current_mode << ", nav_status=" << nav_status
            << ", current_stance=" << static_cast<int>(current_stance) << ", health=" << current_health
            << ", result=" << (condition_met ? "SUCCESS" : "FAILURE") << std::endl;

  if(condition_met)
  {
    setOutput("desired_stance", SentryStance::DEFEND);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
