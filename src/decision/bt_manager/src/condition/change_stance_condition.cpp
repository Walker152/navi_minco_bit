#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/utils/area.hpp"

#include <chrono>
using namespace color_text;
namespace Sentry_BT {
// ------------------- CheckAttackStanceCondition -------------------
CheckAttackStanceCondition::CheckAttackStanceCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAttackStanceCondition::providedPorts()
{
  return {BT::InputPort<int>("high_heat_threshold", 200, "Heat threshold to enter ATTACK"),
    BT::InputPort<int>("low_heat_threshold", 80, "Heat threshold to release ATTACK hysteresis"),
    BT::InputPort<float>("attack_gyro_vel", 80.0f, "Gyro rpm when ATTACK is active")};
}

BT::NodeStatus CheckAttackStanceCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    const int high_heat_threshold = getInput<int>("high_heat_threshold").value_or(200);
    const int low_heat_threshold = getInput<int>("low_heat_threshold").value_or(80);
    const float attack_gyro_vel = getInput<float>("attack_gyro_vel").value_or(80.0f);

    const int current_heat = blackboard->get<int>("current_heat");
    const bool outpost_msg = blackboard->get<bool>("outpost_msg");

    if (!heat_attack_latched_ && current_heat > high_heat_threshold) {
      heat_attack_latched_ = true;
    }
    if (heat_attack_latched_ && current_heat < low_heat_threshold) {
      heat_attack_latched_ = false;
    }

    blackboard->set("heat_attack_latched", heat_attack_latched_);

    const bool condition_met = heat_attack_latched_ || outpost_msg;
    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? GREEN : YELLOW) << "CheckAttackStanceCondition => "
                << (condition_met ? "ATTACK enabled" : "ATTACK disabled") << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::ATTACK);
      blackboard->set("use_gyro_mode", true);
      blackboard->set("gyro_vel", attack_gyro_vel);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckMoveStanceCondition -------------------
CheckMoveStanceCondition::CheckMoveStanceCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckMoveStanceCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckMoveStanceCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    auto current_mode = blackboard->get<int>("current_mode");
    const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
    const bool through_tunnel = blackboard->get<bool>("through_tunnel");
    const bool in_highland =
      highland_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});
    const bool in_own_defense =
      own_defense_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});

    const bool need_cross_to_highland = in_own_defense && through_tunnel;
    const bool need_go_home_supply = current_mode == static_cast<int>(Sentry_BT::NavMode::RETREAT);
    const bool condition_met = (need_cross_to_highland || need_go_home_supply) && !in_highland;

    static bool last_through_tunnel = false;
    if (through_tunnel != last_through_tunnel) {
      std::cout << CYAN << "CheckMoveStanceCondition => through_tunnel " << (through_tunnel ? "ON" : "OFF")
                << RESET << std::endl;
      last_through_tunnel = through_tunnel;
    }

    static bool last_condition_met = false;
    if (condition_met != last_condition_met) {
      std::cout << (condition_met ? GREEN : YELLOW) << "CheckMoveStanceCondition => "
                << (condition_met ? "MOVE enabled" : "MOVE disabled") << RESET << std::endl;
      last_condition_met = condition_met;
    }

    if (condition_met) {
      blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::MOVE);
      blackboard->set("use_gyro_mode", false);
      blackboard->set("gyro_vel", 0.0f);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckDefendStanceCondition -------------------
CheckDefendStanceCondition::CheckDefendStanceCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckDefendStanceCondition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckDefendStanceCondition::tick()
{
  auto blackboard = config().blackboard;
  try {
    const int current_mode = blackboard->get<int>("current_mode");
    const bool target_valid = blackboard->get<bool>("target_valid");
    const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
    const int fort_status = blackboard->get<int>("fort_occupation_status");
    const bool in_fort_area =
      own_defense_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});

    // 追踪模式统一归防御姿态；其余情况也作为默认兜底防御
    const bool defend_reason = current_mode == static_cast<int>(Sentry_BT::NavMode::TRACING) ||
                               target_valid || (in_fort_area && fort_status > 0);

    static int last_reason = -1;
    int reason = defend_reason ? 1 : 0;
    if (reason != last_reason) {
      std::cout << (defend_reason ? RED : WHITE) << "CheckDefendStanceCondition => DEFEND"
                << (defend_reason ? " (trace/target/fort)" : " (default)") << RESET << std::endl;
      last_reason = reason;
    }

    blackboard->set<Sentry_BT::SentryStance>("desired_stance", Sentry_BT::SentryStance::DEFEND);
    blackboard->set("use_gyro_mode", true);
    if (blackboard->get<float>("gyro_vel") <= 0.0f) {
      blackboard->set("gyro_vel", 80.0f);
    }
    return BT::NodeStatus::SUCCESS;
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
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
