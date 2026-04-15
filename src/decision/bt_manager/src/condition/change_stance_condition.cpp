#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/utils/area.hpp"

#include <chrono>
using namespace color_text;
namespace Sentry_BT {
// ------------------- CheckMPCondition -------------------
CheckMPCondition::CheckMPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckMPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckMPCondition::tick()
{
  auto blackboard = config().blackboard;
  blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::MOVE);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckAPCondition -------------------
CheckAPCondition::CheckAPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckAPCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    auto current_mode = blackboard->get<int>("current_mode");
    bool outpost_msg = blackboard->get<bool>("outpost_msg");
    bool target_valid = blackboard->get<bool>("target_valid");
    const bool condition_met =
      current_mode == static_cast<int>(Sentry_BT::NavMode::TRACING) || outpost_msg || target_valid;

    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? GREEN : YELLOW) << "CheckAPCondition => "
                << (condition_met ? "ATTACK enabled" : "ATTACK disabled") << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::ATTACK);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckDPCondition -------------------
CheckDPCondition::CheckDPCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDPCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckDPCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
    const bool in_fort_area =
      own_defense_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});
    const int fort_status = blackboard->get<int>("fort_occupation_status");
    const bool condition_met = in_fort_area && fort_status > 0;

    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? RED : WHITE) << "CheckDPCondition => "
                << (condition_met ? "DEFEND enabled" : "DEFEND disabled") << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::DEFEND);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckStanceRefreshRequired -------------------
CheckStanceRefreshRequired::CheckStanceRefreshRequired(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckStanceRefreshRequired::providedPorts()
{
  return {BT::InputPort<int>("max_hold_seconds", 180, "Max hold seconds before refresh")};
}

BT::NodeStatus CheckStanceRefreshRequired::tick()
{
  auto blackboard = config().blackboard;
  const int max_hold = getInput<int>("max_hold_seconds").value_or(180);

  const auto current_stance = blackboard->get<Sentry_BT::SentryStance>("current_stance");
  static auto last_stance = current_stance;
  static auto hold_start = std::chrono::steady_clock::now();
  static bool refresh_pending = false;
  static Sentry_BT::SentryStance original_stance = Sentry_BT::SentryStance::MOVE;
  static Sentry_BT::SentryStance transient_stance = Sentry_BT::SentryStance::ATTACK;

  if (current_stance != last_stance) {
    hold_start = std::chrono::steady_clock::now();
    last_stance = current_stance;
  }

  if (!refresh_pending) {
    const auto hold_seconds =
      std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hold_start)
        .count();
    if (hold_seconds >= max_hold) {
      refresh_pending = true;
      original_stance = current_stance;
      transient_stance = (current_stance == Sentry_BT::SentryStance::MOVE) ? Sentry_BT::SentryStance::ATTACK
                                                                           : Sentry_BT::SentryStance::MOVE;
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", transient_stance);
      blackboard->set<bool>("stance_refresh_required", true);
      std::cout << YELLOW << "CheckStanceRefreshRequired => REFRESH_TRIGGERED" << RESET << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    blackboard->set<bool>("stance_refresh_required", false);
    return BT::NodeStatus::FAILURE;
  }

  if (current_stance == transient_stance) {
    blackboard->set<Sentry_BT::SentryStance>("desired_stance", original_stance);
    blackboard->set<bool>("stance_refresh_required", true);
    return BT::NodeStatus::SUCCESS;
  }

  if (current_stance == original_stance) {
    refresh_pending = false;
    hold_start = std::chrono::steady_clock::now();
    blackboard->set<bool>("stance_refresh_required", false);
    std::cout << GREEN << "CheckStanceRefreshRequired => REFRESH_FINISHED" << RESET << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  blackboard->set<bool>("stance_refresh_required", true);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
