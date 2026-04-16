#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
using namespace color_text;
namespace Sentry_BT {
// ------------------- CheckRetreatCondition -------------------
CheckRetreatCondition::CheckRetreatCondition(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckRetreatCondition::providedPorts()
{
  return {
    BT::InputPort<float>("health_threshold"),
    BT::InputPort<float>("recovery_threshold"),
    BT::InputPort<int>("ammo_threshold", 100, "Low ammo threshold"),
    BT::InputPort<int>("ammo_recovery_threshold", 100, "Ammo recovery threshold"),
  };
}

BT::NodeStatus CheckRetreatCondition::tick()
{
  auto blackboard = config().blackboard;
  auto health_threshold_ = getInput<float>("health_threshold");
  auto recovery_threshold_ = getInput<float>("recovery_threshold");
  auto ammo_threshold_ = getInput<int>("ammo_threshold");
  auto ammo_recovery_threshold_ = getInput<int>("ammo_recovery_threshold");
  if (!health_threshold_ || !recovery_threshold_ || !ammo_threshold_ || !ammo_recovery_threshold_) {
    throw BT::RuntimeError("missing required input [health_threshold] or [recovery_threshold]");
  }
  float health_threshold = health_threshold_.value();
  float recovery_threshold = recovery_threshold_.value();
  int ammo_threshold = ammo_threshold_.value();
  int ammo_recovery_threshold = ammo_recovery_threshold_.value();
  auto health = blackboard->get<float>("health");
  auto ammo = blackboard->get<int>("bullets_remaining");
  auto current_mode = blackboard->get<int>("current_mode");

  BT::NodeStatus result = BT::NodeStatus::FAILURE;
  if (current_mode == Sentry_BT::NavMode::RETREAT) {
    if (health >= recovery_threshold && ammo > ammo_recovery_threshold) {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
      result = BT::NodeStatus::FAILURE;
    } else {
      result = BT::NodeStatus::SUCCESS;
    }
  } else if (health < health_threshold || ammo < ammo_threshold) {
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::RETREAT);
    result = BT::NodeStatus::SUCCESS;
  }

  static BT::NodeStatus last_result = BT::NodeStatus::IDLE;
  if (result != last_result) {
    std::cout << WHITE << "CheckRetreatCondition => "
              << (result == BT::NodeStatus::SUCCESS ? "RETREAT_ACTIVE" : "RETREAT_INACTIVE")
              << ", health=" << health << ", threshold=" << health_threshold
              << ", recovery=" << recovery_threshold << ", ammo=" << ammo
              << ", ammo_threshold=" << ammo_threshold << RESET << std::endl;
    last_result = result;
  }

  return result;
}

// ------------------- CheckTargetLocked -------------------
CheckTargetLocked::CheckTargetLocked(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetLocked::providedPorts()
{
  return {
    BT::InputPort<bool>("target_valid"),
  };
}

BT::NodeStatus CheckTargetLocked::tick()
{
  auto blackboard = config().blackboard;

  static bool last_condition_met = false;
  static int tick_count = 0;
  static std::chrono::time_point<std::chrono::system_clock> last_seen_time =
    std::chrono::system_clock::now();

  bool target_valid = false;
  try {
    target_valid = blackboard->get<bool>("target_valid");
  } catch (...) {
    if (last_condition_met) {
      std::cout << YELLOW << "CheckTargetLocked => UNLOCKED (target_valid unavailable)" << RESET
                << std::endl;
      last_condition_met = false;
    }
    return BT::NodeStatus::FAILURE;
  }

  try {
    target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  } catch (...) {
    if (last_condition_met) {
      std::cout << YELLOW << "CheckTargetLocked => UNLOCKED (target_pose unavailable)" << RESET
                << std::endl;
      last_condition_met = false;
    }
    return BT::NodeStatus::FAILURE;
  }

  // rmuc
  //  Sentry_BT::Area_Square highland_area = {{6.7, 2.0}, {13.0, -1.8}};
  //  Sentry_BT::Area_Square enemy_outpost_area = {{8.5, 4.5}, {11.5, 2.8}};
  //  Sentry_BT::Area_Square own_outpost_area = {{8.5, -2.7}, {11.5, -4.2}};  //待修改

  TacticalMode tactical_mode = TacticalMode::BALANCED;
  blackboard->get<TacticalMode>("tactical_mode", tactical_mode);

  const Sentry_BT::Point2D target_point{target_pose.position.x, target_pose.position.y, 0.0};
  bool in_attack_area = false;

  if (tactical_mode == TacticalMode::OFFENSIVE) {
    // Offensive: highland + own_defense + enemy_defense
    in_attack_area = highland_zone.contains(target_point) || own_defense_zone.contains(target_point) ||
                     enemy_defense_zone.contains(target_point);
  } else {
    // Defensive and default(BALANCED): own_defense + highland
    in_attack_area = own_defense_zone.contains(target_point) || highland_zone.contains(target_point);
  }
  bool condition_met = false;

  if (in_attack_area && target_valid) {
    last_seen_time = std::chrono::system_clock::now();
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
    condition_met = true;
    tick_count = 0;
  } else if (in_attack_area) {
    auto now = std::chrono::system_clock::now();
    double lost_duration = std::chrono::duration<double>(now - last_seen_time).count();

    // 容忍 1.0 秒内的视觉丢失
    if (lost_duration < 1.0) {
      tick_count++;
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
      condition_met = true;
    } else {
      tick_count = 0;
    }
  } else {
    tick_count = 0;
  }

  if (condition_met != last_condition_met) {
    if (condition_met) {
      std::cout << GREEN << "CheckTargetLocked => LOCKED"
                << ", target_xy=(" << target_pose.position.x << ", " << target_pose.position.y << ")"
                << ", tactical_mode=" << static_cast<int>(tactical_mode)
                << RESET << std::endl;
    } else {
      std::cout << YELLOW << "CheckTargetLocked => UNLOCKED"
                << (in_attack_area ? "" : " (out of attack area)")
                << ", tactical_mode=" << static_cast<int>(tactical_mode) << RESET << std::endl;
    }
    last_condition_met = condition_met;
  }

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckOutpostRemained -------------------
CheckOutpostRemained::CheckOutpostRemained(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOutpostRemained::providedPorts()
{
  return {};
}

BT::NodeStatus CheckOutpostRemained::tick()
{
  auto blackboard = config().blackboard;

  auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
  const bool outpost_remained = !enemy_outpost_destroyed;

  static bool last_outpost_remained = !outpost_remained;
  if (outpost_remained != last_outpost_remained) {
    std::cout << BLUE << "CheckOutpostRemained => "
              << (outpost_remained ? "OUTPOST_REMAINED" : "OUTPOST_DESTROYED") << RESET << std::endl;
    last_outpost_remained = outpost_remained;
  }

  // 如果前哨站还在，切换到响应模式
  if (outpost_remained) {
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::RESPONSE);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckManualOverride -------------------
CheckManualOverride::CheckManualOverride(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckManualOverride::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_seconds", 4.0, "Manual override timeout when goal is unchanged"),
    BT::InputPort<double>("same_goal_eps", 0.05, "Position epsilon to consider goal unchanged")};
}

BT::NodeStatus CheckManualOverride::tick()
{
  auto blackboard = config().blackboard;
  const double timeout_seconds = getInput<double>("timeout_seconds").value_or(4.0);
  const double same_goal_eps = getInput<double>("same_goal_eps").value_or(0.05);

  const bool manual_active = blackboard->get<bool>("manual_override_active");
  const bool goal_valid = blackboard->get<bool>("manual_override_goal_valid");
  if (!manual_active || !goal_valid) {
    blackboard->set<Sentry_BT::ControlMode>("control_mode", Sentry_BT::ControlMode::AUTO);
    if (blackboard->get<int>("current_mode") == static_cast<int>(Sentry_BT::NavMode::MANUAL)) {
      blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
    }
    initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const auto manual_goal = blackboard->get<Sentry_BT::Point2D>("manual_override_goal");
  const auto now = std::chrono::steady_clock::now();

  if (!initialized_) {
    last_goal_ = manual_goal;
    last_goal_change_time_ = now;
    initialized_ = true;
  }

  const double delta = std::hypot(manual_goal.x - last_goal_.x, manual_goal.y - last_goal_.y);
  if (delta > same_goal_eps) {
    last_goal_ = manual_goal;
    last_goal_change_time_ = now;
  }

  const double unchanged_seconds =
    std::chrono::duration<double>(now - last_goal_change_time_).count();
  if (unchanged_seconds >= timeout_seconds) {
    blackboard->set("manual_override_active", false);
    blackboard->set("manual_override_goal_valid", false);
    blackboard->set<Sentry_BT::ControlMode>("control_mode", Sentry_BT::ControlMode::AUTO);
    blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
    initialized_ = false;
    return BT::NodeStatus::FAILURE;
  }

  blackboard->set<Sentry_BT::ControlMode>("control_mode", Sentry_BT::ControlMode::MANUAL_CONTROL);
  blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::MANUAL));
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckOutpostSafeResponse -------------------
CheckOutpostSafeResponse::CheckOutpostSafeResponse(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOutpostSafeResponse::providedPorts()
{
  return {
    BT::InputPort<double>("stable_seconds", 5.0, "Required stable-health duration to release cooldown"),
    BT::InputPort<double>("health_drop_threshold", 0.5, "Health drop threshold to trigger cooldown"),
    BT::InputPort<double>("health_change_eps", 0.1, "Health change epsilon"),
    BT::InputPort<bool>("require_response_mode", false, "Require current nav mode to be RESPONSE")};
}

BT::NodeStatus CheckOutpostSafeResponse::tick()
{
  auto blackboard = config().blackboard;
  const double stable_seconds = getInput<double>("stable_seconds").value_or(5.0);
  const double health_drop_threshold = getInput<double>("health_drop_threshold").value_or(0.5);
  const double health_change_eps = getInput<double>("health_change_eps").value_or(0.1);
  const bool require_response_mode = getInput<bool>("require_response_mode").value_or(false);

  const float health = blackboard->get<float>("health");
  int current_mode = blackboard->get<int>("current_mode");
  const bool outpost_remained = !blackboard->get<bool>("enemy_outpost_destroyed");
  const auto now = std::chrono::steady_clock::now();

  if (!initialized_) {
    last_health_ = health;
    last_health_change_time_ = now;
    initialized_ = true;
  }

  const bool health_changed = std::fabs(health - last_health_) > health_change_eps;
  if (health_changed) {
    last_health_change_time_ = now;
  }

  const bool health_dropped = (last_health_ - health) > health_drop_threshold;
  if (current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE) && health_dropped) {
    cooldown_active_ = true;
    blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
    current_mode = static_cast<int>(Sentry_BT::NavMode::PATROL);
  }

  last_health_ = health;

  if (!outpost_remained) {
    cooldown_active_ = false;
    blackboard->set("outpost_safe_cooldown_active", false);
    return BT::NodeStatus::FAILURE;
  }

  if (cooldown_active_) {
    const double stable_duration = std::chrono::duration<double>(now - last_health_change_time_).count();
    if (stable_duration < stable_seconds) {
      blackboard->set("outpost_safe_cooldown_active", true);
      return BT::NodeStatus::FAILURE;
    }
    cooldown_active_ = false;
  }

  blackboard->set("outpost_safe_cooldown_active", false);
  if (require_response_mode) {
    return current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE) ? BT::NodeStatus::SUCCESS
                                                                          : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// --------------------- CheckInStairsZone ----------------------
CheckInStairsZone::CheckInStairsZone(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  // 构造函数：初始化节点，不需要复杂操作
}

BT::PortsList CheckInStairsZone::providedPorts()
{
  return {};  // 不需要输入端口，直接从黑板读取位置
}

BT::NodeStatus CheckInStairsZone::tick()
{
  auto blackboard = config().blackboard;

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");

  double x = current_pose.position.x;
  double y = current_pose.position.y;

  bool in_stairs_zone = stairs_zone.contains({x, y, 0.0});

  static bool last_in_stairs_zone = false;
  if (in_stairs_zone != last_in_stairs_zone) {
    std::cout << WHITE << "CheckInStairsZone => " << (in_stairs_zone ? "IN_ZONE" : "OUT_OF_ZONE")
              << ", pos=(" << x << ", " << y << ")" << RESET << std::endl;
    last_in_stairs_zone = in_stairs_zone;
  }

  return in_stairs_zone ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckWillThroughTunnel ----------------------
CheckWillThroughTunnel::CheckWillThroughTunnel(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  // 构造函数：初始化节点，不需要复杂操作
}

BT::PortsList CheckWillThroughTunnel::providedPorts()
{
  return {};  // 不需要输入端口，直接从黑板读取信息
}

BT::NodeStatus CheckWillThroughTunnel::tick()
{
  auto blackboard = config().blackboard;
  bool will_through_tunnel = false;
  auto lifter_current_pos = blackboard->get<LifterPos>("lifter_current_pos");
  will_through_tunnel = blackboard->get<bool>("through_tunnel");
  static bool last_state_ = !will_through_tunnel;
  if (last_state_ != will_through_tunnel) {
    std::cout << WHITE << "CheckWillThroughTunnel => "
              << (will_through_tunnel ? "WILL_THROUGH_TUNNEL" : "WILL_NOT_THROUGH_TUNNEL") << RESET
              << std::endl;
    last_state_ = will_through_tunnel;
  }

  if (will_through_tunnel) {
    blackboard->set<LifterPos>(
      "desired_lifter_pos", LifterPos::BOTTOM);  // 设置目标升降位置为 1(bottom)，准备过隧道
  } else {
    blackboard->set<LifterPos>(
      "desired_lifter_pos", LifterPos::TOP);  // 设置目标升降位置为 0(top)，准备不通过隧道
  }
  return BT::NodeStatus::SUCCESS;
}

// --------------------- CheckNoAllyBelowStairs ----------------------
CheckNoAllyBelowStairs::CheckNoAllyBelowStairs(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckNoAllyBelowStairs::providedPorts()
{
  return {};
}

BT::NodeStatus CheckNoAllyBelowStairs::tick()
{
  auto blackboard = config().blackboard;
  const auto allies = blackboard->get<std::vector<AllyRobotInfo>>("allies_info");
  bool ally_below = false;
  for (const auto & ally : allies) {
    if (stairs_lower_safe_zone.contains({ally.position.x, ally.position.y, 0.0})) {
      ally_below = true;
      break;
    }
  }

  static bool last_clear = ally_below;
  const bool clear = !ally_below;
  if (clear != last_clear) {
    std::cout << WHITE << "CheckNoAllyBelowStairs => " << (clear ? "CLEAR" : "BLOCKED") << RESET
              << std::endl;
    last_clear = clear;
  }
  return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckAmmoLow ----------------------
CheckAmmoLow::CheckAmmoLow(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAmmoLow::providedPorts()
{
  return {BT::InputPort<int>("ammo_threshold", 100, "Low ammo threshold")};
}

BT::NodeStatus CheckAmmoLow::tick()
{
  auto blackboard = config().blackboard;
  const int threshold = getInput<int>("ammo_threshold").value_or(100);
  const int ammo = blackboard->get<int>("bullets_remaining");
  const bool low = ammo < threshold;

  static bool last_low = !low;
  if (low != last_low) {
    std::cout << WHITE << "CheckAmmoLow => " << (low ? "LOW" : "NORMAL") << ", ammo=" << ammo << std::endl;
    last_low = low;
  }
  return low ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckTacticalModeCondition ----------------------
CheckTacticalModeCondition::CheckTacticalModeCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTacticalModeCondition::providedPorts()
{
  return {BT::InputPort<std::string>("mode")};
}

BT::NodeStatus CheckTacticalModeCondition::tick()
{
  auto blackboard = config().blackboard;
  const std::string mode = getInput<std::string>("mode").value_or("normal");
  TacticalMode expected_mode = TacticalMode::BALANCED;
  if (mode == "attack") {
    expected_mode = TacticalMode::OFFENSIVE;
  } else if (mode == "defend") {
    expected_mode = TacticalMode::DEFENSIVE;
  }
  const auto current_mode = blackboard->get<TacticalMode>("tactical_mode");
  return current_mode == expected_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckOwnFortIdle ----------------------
CheckOwnFortIdle::CheckOwnFortIdle(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOwnFortIdle::providedPorts()
{
  return {};
}

BT::NodeStatus CheckOwnFortIdle::tick()
{
  auto blackboard = config().blackboard;
  const int fort_status = blackboard->get<int>("fort_occupation_status");
  const bool idle = fort_status == 0;
  return idle ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckEnemyBaseLowHp ----------------------
CheckEnemyBaseLowHp::CheckEnemyBaseLowHp(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEnemyBaseLowHp::providedPorts()
{
  return {BT::InputPort<int>("threshold", 1000, "Enemy base HP threshold")};
}

BT::NodeStatus CheckEnemyBaseLowHp::tick()
{
  static bool logged_once = false;
  if (!logged_once) {
    std::cout << YELLOW
              << "CheckEnemyBaseLowHp is disabled: enemy base HP is currently unavailable from IO"
              << RESET << std::endl;
    logged_once = true;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace Sentry_BT