#include "bt_manager/condition/auto_conditions.hpp"
#include <cmath>
#include <iostream>

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
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
  };
}

BT::NodeStatus CheckRetreatCondition::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  auto health_threshold_ = getInput<float>("health_threshold").value_or(50.0f);
  auto recovery_threshold_ = getInput<float>("recovery_threshold").value_or(50.0f);
  auto ammo_threshold_ = getInput<int>("ammo_threshold").value_or(100);
  auto ammo_recovery_threshold_ = getInput<int>("ammo_recovery_threshold").value_or(100);

  auto health = blackboard->get<float>("health");
  auto ammo = blackboard->get<int>("bullets_remaining");
  auto current_mode = blackboard->get<int>("current_mode");

  BT::NodeStatus result = BT::NodeStatus::FAILURE;
  if (current_mode == Sentry_BT::NavMode::RETREAT) {
    if (health >= recovery_threshold_ && ammo >= ammo_recovery_threshold_) {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
      result = BT::NodeStatus::FAILURE;
    } else {
      result = BT::NodeStatus::SUCCESS;
    }
  } else if (health < health_threshold_ || ammo < ammo_threshold_) {
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::RETREAT);
    result = BT::NodeStatus::SUCCESS;
  }

  const bool active = (result == BT::NodeStatus::SUCCESS);
  std::ostringstream retreat_detail;
  retreat_detail << "health=" << health << ", health_threshold=" << health_threshold_
                 << ", recovery_threshold=" << recovery_threshold_ << ", ammo=" << ammo
                 << ", ammo_threshold=" << ammo_threshold_
                 << ", ammo_recovery_threshold=" << ammo_recovery_threshold_
                 << ", current_mode=" << current_mode;
  detail::logTransition(
    detail::TreeKind::NAV, "CheckRetreatCondition", active, retreat_detail.str(), branch);

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
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
  };
}

BT::NodeStatus CheckTargetLocked::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  static int tick_count = 0;
  static std::chrono::time_point<std::chrono::system_clock> last_seen_time =
    std::chrono::system_clock::now();

  bool target_valid = false;
  try {
    target_valid = blackboard->get<bool>("target_valid");
  } catch (...) {
    detail::logTransition(
      detail::TreeKind::NAV, "CheckTargetLocked", false, "target_valid unavailable", branch);
    return BT::NodeStatus::FAILURE;
  }

  try {
    target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  } catch (...) {
    detail::logTransition(
      detail::TreeKind::NAV, "CheckTargetLocked", false, "target_pose unavailable", branch);
    return BT::NodeStatus::FAILURE;
  }

  const Sentry_BT::Point2D target_point{target_pose.position.x, target_pose.position.y, 0.0};
  const auto tactical_mode = blackboard->get<TacticalMode>("tactical_mode");
  const auto & include_areas = tracking_areas.at(tactical_mode);

  bool in_attack_area = false;
  for (std::size_t i = 0; i < include_areas.size(); ++i) {
    if (include_areas[i].contains(target_point)) {
      in_attack_area = true;
      break;
    }
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

  std::ostringstream lock_detail;
  lock_detail << "target_valid=" << target_valid << ", in_attack_area=" << in_attack_area << ", target_xy=("
              << target_pose.position.x << ", " << target_pose.position.y << ")"
              << ", tactical_mode=" << static_cast<int>(tactical_mode);
  detail::logTransition(
    detail::TreeKind::NAV, "CheckTargetLocked", condition_met, lock_detail.str(), branch);
  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckOutpostRemained -------------------
CheckOutpostRemained::CheckOutpostRemained(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOutpostRemained::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckOutpostRemained::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
  const bool outpost_remained = !enemy_outpost_destroyed;

  detail::logTransition(detail::TreeKind::NAV,
    "CheckOutpostRemained",
    outpost_remained,
    outpost_remained ? "enemy outpost remained" : "enemy outpost destroyed",
    branch);

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
  return {BT::InputPort<double>("timeout_seconds", 4.0, "Manual override timeout when goal is unchanged"),
    BT::InputPort<double>("same_goal_eps", 0.05, "Position epsilon to consider goal unchanged"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckManualOverride::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double timeout_seconds = getInput<double>("timeout_seconds").value_or(4.0);
  const double same_goal_eps = getInput<double>("same_goal_eps").value_or(0.05);

  const auto current_control_mode = blackboard->get<Sentry_BT::ControlMode>("control_mode");
  if (current_control_mode != ControlMode::MANUAL_CONTROL) {
    return BT::NodeStatus::FAILURE;
  }

  const auto manual_goal = blackboard->get<Sentry_BT::Point2D>("manual_override_goal");
  const auto now = std::chrono::steady_clock::now();

  last_goal_ = manual_goal;
  last_goal_change_time_ = now;

  const double delta = std::hypot(manual_goal.x - last_goal_.x, manual_goal.y - last_goal_.y);
  if (delta > same_goal_eps) {
    last_goal_ = manual_goal;
    last_goal_change_time_ = now;
  }

  const double unchanged_seconds = std::chrono::duration<double>(now - last_goal_change_time_).count();
  if (unchanged_seconds >= timeout_seconds) {
    blackboard->set<Sentry_BT::ControlMode>("control_mode", Sentry_BT::ControlMode::AUTO);
    blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
    detail::logTransition(
      detail::TreeKind::NAV, "CheckManualOverride", false, "manual timeout reached", branch);
    return BT::NodeStatus::FAILURE;
  }

  blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::MANUAL));
  {
    std::ostringstream manual_detail;
    manual_detail << "goal=(" << manual_goal.x << ", " << manual_goal.y << ")"
                  << ", unchanged_seconds=" << unchanged_seconds << ", timeout_seconds=" << timeout_seconds;
    detail::logTransition(detail::TreeKind::NAV, "CheckManualOverride", true, manual_detail.str(), branch);
  }
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
    BT::InputPort<bool>("require_response_mode", false, "Require current nav mode to be RESPONSE"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckOutpostSafeResponse::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double stable_seconds = getInput<double>("stable_seconds").value_or(5.0);
  const bool require_response_mode = getInput<bool>("require_response_mode").value_or(false);

  const float health = blackboard->get<float>("health");
  int current_mode = blackboard->get<int>("current_mode");
  const bool outpost_remained = !blackboard->get<bool>("enemy_outpost_destroyed");
  const auto now = std::chrono::steady_clock::now();

  const bool health_dropped = (last_health_ - health) > 1e-3;
  if (health_dropped) {
    last_health_change_time_ = now;
  }
  if (current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE) && health_dropped) {
    cooldown_active_ = true;
    blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
    current_mode = static_cast<int>(Sentry_BT::NavMode::PATROL);
  }

  last_health_ = health;

  if (!outpost_remained) {
    cooldown_active_ = false;
    blackboard->set("outpost_safe_cooldown_active", false);
    detail::logTransition(
      detail::TreeKind::NAV, "CheckOutpostSafeResponse", false, "enemy outpost destroyed", branch);
    return BT::NodeStatus::FAILURE;
  }

  if (cooldown_active_) {
    const double stable_duration = std::chrono::duration<double>(now - last_health_change_time_).count();
    if (stable_duration < stable_seconds) {
      blackboard->set("outpost_safe_cooldown_active", true);
      {
        std::ostringstream safe_detail;
        safe_detail << "cooldown active, stable_duration=" << stable_duration
                    << ", stable_seconds=" << stable_seconds;
        detail::logTransition(
          detail::TreeKind::NAV, "CheckOutpostSafeResponse", false, safe_detail.str(), branch);
      }
      return BT::NodeStatus::FAILURE;
    }
    cooldown_active_ = false;
  }

  blackboard->set("outpost_safe_cooldown_active", false);
  const bool active =
    require_response_mode ? (current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE)) : true;
  {
    std::ostringstream safe_detail;
    safe_detail << "health=" << health << ", current_mode=" << current_mode
                << ", require_response_mode=" << require_response_mode
                << ", cooldown_active=" << cooldown_active_;
    detail::logTransition(
      detail::TreeKind::NAV, "CheckOutpostSafeResponse", active, safe_detail.str(), branch);
  }
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckInStairsZone ----------------------
CheckInStairsZone::CheckInStairsZone(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  // 构造函数：初始化节点，不需要复杂操作
}

BT::PortsList CheckInStairsZone::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckInStairsZone::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");

  double x = current_pose.position.x;
  double y = current_pose.position.y;

  bool in_stairs_zone = false;
  for (const auto & zone : stairs_zone) {
    if (zone.contains({x, y, 0.0})) {
      in_stairs_zone = true;
      break;
    }
  }

  {
    std::ostringstream stairs_detail;
    stairs_detail << "pos=(" << x << ", " << y << ")";
    detail::logTransition(
      detail::TreeKind::NAV, "CheckInStairsZone", in_stairs_zone, stairs_detail.str(), branch);
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
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckWillThroughTunnel::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  auto lifter_current_pos = blackboard->get<LifterPos>("lifter_current_pos");
  const bool through_tunnel = blackboard->get<bool>("through_tunnel");
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  bool in_transform_zone = false;
  for (const auto & zone : transform_zone) {
    if (zone.contains({current_pose.position.x, current_pose.position.y, 0.0})) {
      in_transform_zone = true;
      break;
    }
  }

  const bool keep_bottom_locked = in_transform_zone && (lifter_current_pos == LifterPos::BOTTOM);
  const bool will_through_tunnel = through_tunnel || keep_bottom_locked;

  {
    std::ostringstream tunnel_detail;
    tunnel_detail << "through_tunnel=" << through_tunnel << ", in_transform_zone=" << in_transform_zone
                  << ", lifter_current_pos=" << static_cast<int>(lifter_current_pos);
    detail::logTransition(
      detail::TreeKind::NAV, "CheckWillThroughTunnel", will_through_tunnel, tunnel_detail.str(), branch);
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
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckNoAllyBelowStairs::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const auto allies = blackboard->get<std::vector<AllyRobotInfo>>("allies_info");
  bool ally_below = false;
  for (const auto & ally : allies) {
    for (const auto & zone : stairs_lower_safe_zone) {
      if (zone.contains({ally.position.position.x, ally.position.position.y, 0.0})) {
        ally_below = true;
        break;
      }
    }
    if (ally_below) {
      break;
    }
  }

  const bool clear = !ally_below;
  detail::logTransition(
    detail::TreeKind::NAV, "CheckNoAllyBelowStairs", clear, clear ? "area clear" : "ally detected", branch);
  return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckAmmoLow ----------------------
CheckAmmoLow::CheckAmmoLow(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAmmoLow::providedPorts()
{
  return {BT::InputPort<int>("ammo_threshold", 100, "Low ammo threshold"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckAmmoLow::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int threshold = getInput<int>("ammo_threshold").value_or(100);
  const int ammo = blackboard->get<int>("bullets_remaining");
  const bool low = ammo < threshold;

  {
    std::ostringstream ammo_detail;
    ammo_detail << "ammo=" << ammo << ", threshold=" << threshold;
    detail::logTransition(detail::TreeKind::NAV, "CheckAmmoLow", low, ammo_detail.str(), branch);
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
  return {BT::InputPort<std::string>("mode"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckTacticalModeCondition::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const std::string mode = getInput<std::string>("mode").value_or("normal");
  TacticalMode expected_mode = TacticalMode::BALANCED;
  if (mode == "attack") {
    expected_mode = TacticalMode::OFFENSIVE;
  } else if (mode == "defend") {
    expected_mode = TacticalMode::DEFENSIVE;
  }
  const auto current_mode = blackboard->get<TacticalMode>("tactical_mode");
  const bool active = (current_mode == expected_mode);
  {
    std::ostringstream tactical_detail;
    tactical_detail << "mode=" << mode << ", current_tactical_mode=" << static_cast<int>(current_mode)
                    << ", expected_mode=" << static_cast<int>(expected_mode);
    detail::logTransition(
      detail::TreeKind::NAV, "CheckTacticalModeCondition", active, tactical_detail.str(), branch);
  }
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckOwnFortIdle ----------------------
CheckOwnFortIdle::CheckOwnFortIdle(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOwnFortIdle::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckOwnFortIdle::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int fort_status = blackboard->get<int>("fort_occupation_status");
  const bool idle = fort_status == 0;
  {
    std::ostringstream fort_detail;
    fort_detail << "fort_occupation_status=" << fort_status;
    detail::logTransition(detail::TreeKind::NAV, "CheckOwnFortIdle", idle, fort_detail.str(), branch);
  }
  return idle ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckEnemyBaseLowHp ----------------------
CheckEnemyBaseLowHp::CheckEnemyBaseLowHp(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEnemyBaseLowHp::providedPorts()
{
  return {BT::InputPort<int>("threshold", 1000, "Enemy base HP threshold"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckEnemyBaseLowHp::tick()
{
  const std::string branch = getInput<std::string>("branch").value_or("");
  detail::logTransition(detail::TreeKind::NAV,
    "CheckEnemyBaseLowHp",
    false,
    "disabled: enemy base HP unavailable from IO",
    branch);
  return BT::NodeStatus::FAILURE;
}
}  // namespace Sentry_BT