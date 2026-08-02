#include "bt_manager/condition/change_stance_condition.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

#include "bt_manager/action/change_stance_action.hpp"

namespace Sentry_BT {

namespace {
std::string stanceName(SentryStance stance)
{
  const auto index = static_cast<size_t>(stance);
  if (index >= 1 && index <= stance_names.size()) {
    return stance_names[index - 1];
  }
  return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
}
}  // namespace

// ------------------- CheckHeat -------------------
CheckHeat::CheckHeat(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckHeat::providedPorts()
{
  return {BT::InputPort<float>("threshold", 200.0f, "Heat threshold"),
    BT::InputPort<std::string>("mode", "greater", "greater/less"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckHeat::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const float threshold = getInput<float>("threshold").value_or(200.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

  const int current_heat = blackboard->get<int>("current_heat");
  const bool active = detail::compareByMode(static_cast<float>(current_heat), threshold, mode);

  std::ostringstream oss;
  oss << "heat=" << current_heat << ", mode=" << mode << ", threshold=" << threshold;
  detail::logTransition(detail::TreeKind::STANCE, "CheckHeat", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckOutpostTarget -------------------
CheckOutpostTarget::CheckOutpostTarget(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOutpostTarget::providedPorts()
{
  return {BT::InputPort<float>("goal_distance_threshold", 0.8f, "Goal close-to-outpost threshold"),
    BT::InputPort<bool>("require_pitch_up", false, "Require gimbal pitch mode to be UP"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckOutpostTarget::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto current_mode = blackboard->get<NavMode>("current_mode");
  geometry_msgs::msg::Pose current_pose;
  Point2D nav_goal;
  current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  nav_goal = blackboard->get<Point2D>("nav_goal");

  const bool attacking_outpost_mode = current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE);
  const bool in_outpost_zone =
    enemy_outpost_watch_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});

  const float dist_threshold = getInput<float>("goal_distance_threshold").value_or(0.8f);
  const auto & outpost = nav_points[static_cast<size_t>(Sentry_BT::NavGoal::ENEMY_OUTPOST)];
  const bool nav_goal_is_outpost =
    std::hypot(nav_goal.x - outpost.x, nav_goal.y - outpost.y) <= static_cast<double>(dist_threshold);
  const bool require_pitch_up = getInput<bool>("require_pitch_up").value_or(false);
  const bool pitch_up = blackboard->get<PitchPos>("pitch_mode") == PitchPos::UP;
  const bool active = attacking_outpost_mode && in_outpost_zone && nav_goal_is_outpost &&
                      (!require_pitch_up || pitch_up);

  std::ostringstream oss;
  oss << "mode=" << current_mode << ", in_outpost_zone=" << in_outpost_zone
      << ", nav_goal_is_outpost=" << nav_goal_is_outpost
      << ", require_pitch_up=" << require_pitch_up << ", pitch_up=" << pitch_up;
  detail::logTransition(detail::TreeKind::STANCE, "CheckOutpostTarget", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckEngagedStatus -------------------
CheckEngagedStatus::CheckEngagedStatus(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEngagedStatus::providedPorts()
{
  return {BT::InputPort<bool>("expected", true, "Expected engaged status"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckEngagedStatus::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const bool expected_engaged = getInput<bool>("expected").value_or(true);

  const bool is_disengaged = blackboard->get<bool>("is_disengaged");
  const bool engaged = !is_disengaged;
  const bool active = (engaged == expected_engaged);

  std::ostringstream oss;
  oss << "engaged=" << engaged << ", expected=" << expected_engaged;
  detail::logTransition(detail::TreeKind::STANCE, "CheckEngagedStatus", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckHealth -------------------
CheckHealth::CheckHealth(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckHealth::providedPorts()
{
  return {BT::InputPort<float>("threshold", 50.0f, "Health threshold"),
    BT::InputPort<std::string>("mode", "greater", "greater/less"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckHealth::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto health = blackboard->get<float>("health");

  const float threshold = getInput<float>("threshold").value_or(50.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");
  const bool active = detail::compareByMode(health, threshold, mode);

  std::ostringstream oss;
  oss << "health=" << health << ", mode=" << mode << ", threshold=" << threshold;
  detail::logTransition(detail::TreeKind::STANCE, "CheckHealth", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckTargetDistance -------------------
CheckTargetDistance::CheckTargetDistance(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetDistance::providedPorts()
{
  return {BT::InputPort<float>("threshold", 1.0f, "Distance threshold in meters"),
    BT::InputPort<std::string>("mode", "greater", "greater/less"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckTargetDistance::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  const auto current_mode = blackboard->get<NavMode>("current_mode");
  const float threshold = getInput<float>("threshold").value_or(1.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

  if (current_mode != NavMode::TRACING) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckTargetDistance", false, "Not in TRACING mode", branch);
    return BT::NodeStatus::FAILURE;
  }
  const auto distance = static_cast<float>(std::hypot(
    target_pose.position.x - current_pose.position.x, target_pose.position.y - current_pose.position.y));
  const bool active = detail::compareByMode(distance, threshold, mode);

  std::ostringstream oss;
  oss << "distance=" << distance << ", mode=" << mode << ", threshold=" << threshold;
  detail::logTransition(detail::TreeKind::STANCE, "CheckTargetDistance", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckCrossZoneTransition -------------------
CheckCrossZoneTransition::CheckCrossZoneTransition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCrossZoneTransition::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckCrossZoneTransition::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");
  const bool through_tunnel = blackboard->get<bool>("through_tunnel");
  const bool current_in_tunnel = blackboard->get<bool>("current_in_tunnel");
  // const bool is_disengaged = blackboard->get<bool>("is_disengaged");

  const Point2D current_point{current_pose.position.x, current_pose.position.y, 0.0};
  const Point2D goal_point{nav_goal.x, nav_goal.y, 0.0};

  // Cross-zone detection: check whether current and goal are in different zones
  const bool current_in_highland = highland_zone.contains(current_point);
  const bool current_in_own = own_defense_zone.contains(current_point);
  const bool current_in_enemy = enemy_defense_zone.contains(current_point);
  const bool goal_in_highland = highland_zone.contains(goal_point);
  const bool goal_in_own = own_defense_zone.contains(goal_point);
  const bool goal_in_enemy = enemy_defense_zone.contains(goal_point);
  const bool current_in_named = current_in_highland || current_in_own || current_in_enemy;
  const bool goal_in_named = goal_in_highland || goal_in_own || goal_in_enemy;
  const bool same_zone = (current_in_highland && goal_in_highland) || (current_in_own && goal_in_own) ||
                         (current_in_enemy && goal_in_enemy) || (!current_in_named && !goal_in_named);
  const bool need_cross_zone = !same_zone;
  const bool in_transform_zone = blackboard->get<bool>("in_transform_zone");
  bool is_tunnel_journey = false;
  const int active_tunnel_idx = blackboard->get<int>("nearest_tunnel_idx");

  if (current_in_tunnel || (in_transform_zone && through_tunnel)) {
    is_tunnel_journey = true;
  }

  std::ostringstream oss;
  oss << "through_tunnel=" << through_tunnel << ", tunnel_idx=" << active_tunnel_idx
      << ", current_in_tunnel=" << current_in_tunnel << ", need_cross_zone=" << need_cross_zone;
  detail::logTransition(
    detail::TreeKind::STANCE, "CheckCrossZoneTransition", is_tunnel_journey, oss.str(), branch);

  return is_tunnel_journey ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckCapacitorCapacity -------------------
CheckCapacitorCapacity::CheckCapacitorCapacity(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCapacitorCapacity::providedPorts()
{
  return {BT::InputPort<float>("threshold", 30.0f, "Capacitor threshold percentage"),
    BT::InputPort<std::string>("mode", "less", "greater/less"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckCapacitorCapacity::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const float capacitor_capacity = static_cast<float>(blackboard->get<uint8_t>("capacitor_capacity"));

  const float threshold = getInput<float>("threshold").value_or(30.0f);
  const std::string mode = getInput<std::string>("mode").value_or("less");
  const bool active = detail::compareByMode(capacitor_capacity, threshold, mode);

  std::ostringstream oss;
  oss << "capacitor=" << capacitor_capacity << ", mode=" << mode << ", threshold=" << threshold;
  detail::logTransition(detail::TreeKind::STANCE, "CheckCapacitorCapacity", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckStanceCooldown -------------------
CheckStanceCooldown::CheckStanceCooldown(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckStanceCooldown::providedPorts()
{
  return {BT::InputPort<double>("cooldown", 5.0, "Cooldown seconds between stance changes"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckStanceCooldown::tick()
{
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double cooldown = getInput<double>("cooldown").value_or(5.0);

  const auto last_change = ChangeStance::getLastChangeTime();
  const auto now = std::chrono::system_clock::now();
  const double elapsed = (last_change == std::chrono::time_point<std::chrono::system_clock>::min())
                           ? std::numeric_limits<double>::infinity()
                           : std::chrono::duration<double>(now - last_change).count();

  const bool active = elapsed >= cooldown;
  std::ostringstream oss;
  oss << "elapsed=" << elapsed << ", cooldown=" << cooldown;
  detail::logTransition(detail::TreeKind::STANCE, "CheckStanceCooldown", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckStanceEffectLimit -------------------
CheckStanceEffectLimit::CheckStanceEffectLimit(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckStanceEffectLimit::providedPorts()
{
  return {BT::InputPort<std::string>("target_stance", "ATTACK", "Target stance to check"),
    BT::InputPort<int>("max_time_sec", 180, "Max accumulated seconds")};
}

BT::NodeStatus CheckStanceEffectLimit::tick()
{
  const auto blackboard = config().blackboard;
  std::string target = getInput<std::string>("target_stance").value_or("ATTACK");
  const int max_time = getInput<int>("max_time_sec").value_or(180);

  std::transform(target.begin(), target.end(), target.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  double accumulated = 0.0;
  bool recognized = true;
  if (target == "ATTACK") {
    accumulated = blackboard->get<double>("attack_accumulated_time");
  } else if (target == "DEFEND") {
    accumulated = blackboard->get<double>("defend_accumulated_time");
  } else if (target == "MOVE") {
    accumulated = blackboard->get<double>("move_accumulated_time");
  } else {
    recognized = false;
  }

  if (!recognized) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckStanceEffectLimit", true, "unknown target_stance", "");
    return BT::NodeStatus::SUCCESS;
  }

  const bool active = accumulated < static_cast<double>(max_time);
  std::ostringstream oss;
  oss << "target=" << target << ", accumulated=" << accumulated << ", max=" << max_time;
  detail::logTransition(detail::TreeKind::STANCE, "CheckStanceEffectLimit", active, oss.str(), "");

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckStanceRefreshRequired -------------------
/* deprecated - replaced by accumulated time logic */
CheckStanceRefreshRequired::CheckStanceRefreshRequired(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckStanceRefreshRequired::providedPorts()
{
  return {BT::InputPort<int>("max_hold_seconds", 180, "Max hold seconds before refresh"),
    BT::OutputPort<std::string>("target_stance", "Target stance to switch to when refresh is needed"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckStanceRefreshRequired::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int max_hold = getInput<int>("max_hold_seconds").value_or(180);
  const auto current_stance = blackboard->get<SentryStance>("current_stance");

  if (current_stance != last_stance) {
    hold_start = std::chrono::steady_clock::now();
    last_stance = current_stance;
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckStanceRefreshRequired", false, "stance changed, reset timer", branch);
    return BT::NodeStatus::FAILURE;
  }

  const auto hold_seconds =
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hold_start).count();
  if (hold_seconds >= max_hold) {
    SentryStance transient_stance = SentryStance::DEFEND;
    if (current_stance == SentryStance::DEFEND) {
      transient_stance = SentryStance::ATTACK;
    } else {
      transient_stance = SentryStance::DEFEND;
    }
    auto stance_to_string = [](SentryStance stance) -> std::string {
      switch (stance) {
      case SentryStance::ATTACK:
        return "ATTACK";
      case SentryStance::DEFEND:
        return "DEFEND";
      case SentryStance::MOVE:
        return "MOVE";
      case SentryStance::ENHANCED_ATTACK:
        return "ENHANCED_ATTACK";
      case SentryStance::ENHANCED_DEFEND:
        return "ENHANCED_DEFEND";
      case SentryStance::ENHANCED_MOVE:
        return "ENHANCED_MOVE";
      default:
        return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
      }
    };
    auto transient_stance_str = stance_to_string(transient_stance);
    setOutput<std::string>("target_stance", transient_stance_str);
    std::ostringstream oss;
    oss << "hold_seconds=" << hold_seconds << ", max_hold=" << max_hold
        << ", target_stance=" << transient_stance_str;
    detail::logTransition(detail::TreeKind::STANCE, "CheckStanceRefreshRequired", true, oss.str(), branch);
    return BT::NodeStatus::SUCCESS;
  }

  std::ostringstream oss;
  oss << "hold_seconds=" << hold_seconds << ", max_hold=" << max_hold;
  detail::logTransition(detail::TreeKind::STANCE, "CheckStanceRefreshRequired", false, oss.str(), branch);

  return BT::NodeStatus::FAILURE;
}

// ------------------- CheckTunnelDeformation -------------------
CheckTunnelDeformation::CheckTunnelDeformation(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTunnelDeformation::providedPorts()
{
  return {};
}

BT::NodeStatus CheckTunnelDeformation::tick()
{
  auto blackboard = config().blackboard;
  const bool through_tunnel = blackboard->get<bool>("through_tunnel");
  const bool current_in_tunnel = blackboard->get<bool>("current_in_tunnel");
  const bool is_disengaged = blackboard->get<bool>("is_disengaged");
  const bool in_transform_zone = blackboard->get<bool>("in_transform_zone");
  const auto lifter_current_pos = blackboard->get<LifterPos>("lifter_current_pos");
  const bool tunnel_yaw_aligned = blackboard->get<bool>("tunnel_yaw_aligned");
  const bool tunnel_prepare_active =
    current_in_tunnel || (in_transform_zone && through_tunnel);
  const bool tunnel_ready =
    tunnel_prepare_active &&
    lifter_current_pos == LifterPos::BOTTOM && tunnel_yaw_aligned;
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  LifterPos desired_pos = LifterPos::TOP;

  if (current_in_tunnel) {
    desired_pos = LifterPos::BOTTOM;
  } else if (in_transform_zone) {
    if (through_tunnel) {
      desired_pos = LifterPos::BOTTOM;
    }
  }

  // std::cout << " in_transform_zone=" << in_transform_zone << "current_pose= ("
  // << current_pose.position.x << ", " << current_pose.position.y << ")" << std::endl;
  blackboard->set<LifterPos>("desired_lifter_pos", desired_pos);
  blackboard->set("tunnel_prepare_active", tunnel_prepare_active);
  blackboard->set("tunnel_ready", tunnel_ready);
  if (!tunnel_prepare_active) {
    blackboard->set("tunnel_yaw_aligned", false);
  }
  detail::logTransition(detail::TreeKind::STANCE,
    "CheckTunnelDeformation",
    true,
    "through_tunnel=" + std::to_string(through_tunnel) + ", current_in_tunnel=" +
      std::to_string(current_in_tunnel) + ", is_disengaged=" + std::to_string(is_disengaged) +
      ", desired_pos=" + std::to_string(static_cast<int>(desired_pos)) +
      ", tunnel_ready=" + std::to_string(tunnel_ready),
    "");
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckInEnemyFortZone -------------------
CheckInEnemyFortZone::CheckInEnemyFortZone(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckInEnemyFortZone::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckInEnemyFortZone::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const Point2D p{current_pose.position.x, current_pose.position.y, 0.0};
  const bool in_enemy_fort = enemy_fort_engage_zone.contains(p);
  // for test
  // const bool in_enemy_fort = enemy_fort_zone.contains(p);

  std::ostringstream oss;
  oss << "pos=(" << current_pose.position.x << ", " << current_pose.position.y << ")";

  detail::logTransition(detail::TreeKind::STANCE, "CheckInEnemyFortZone", in_enemy_fort, oss.str(), branch);

  return in_enemy_fort ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckManualStanceOverride -------------------
CheckManualStanceOverride::CheckManualStanceOverride(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckManualStanceOverride::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckManualStanceOverride::tick()
{
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const bool override_active = blackboard->get<bool>("manual_stance_override_active");
  if (!override_active) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckManualStanceOverride", false, "override not active", branch);
    return BT::NodeStatus::FAILURE;
  }

  const auto override_stance = blackboard->get<SentryStance>("manual_override_stance");

  // 只处理强化姿态覆盖。若覆盖意图是普通姿态,不拦截自动决策。
  if (override_stance != SentryStance::ENHANCED_ATTACK &&
      override_stance != SentryStance::ENHANCED_DEFEND &&
      override_stance != SentryStance::ENHANCED_MOVE) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckManualStanceOverride", false,
      "override_stance=" + stanceName(override_stance) + " (not enhanced)", branch);
    return BT::NodeStatus::FAILURE;
  }

  // 能量不足时无法进入强化姿态。
  const auto energy_ratio = blackboard->get<EnergyRatio>("energy_ratio");
  if (energy_ratio == EnergyRatio::BELOW_1) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckManualStanceOverride", false, "energy insufficient", branch);
    return BT::NodeStatus::FAILURE;
  }

  // 该强化姿态剩余时间用尽。
  int remaining_sec = 0;
  switch (override_stance) {
  case SentryStance::ENHANCED_ATTACK:
    remaining_sec = blackboard->get<int>("enhanced_attack_remaining_time");
    break;
  case SentryStance::ENHANCED_DEFEND:
    remaining_sec = blackboard->get<int>("enhanced_defend_remaining_time");
    break;
  case SentryStance::ENHANCED_MOVE:
    remaining_sec = blackboard->get<int>("enhanced_move_remaining_time");
    break;
  default:
    break;
  }
  if (remaining_sec <= 0) {
    blackboard->set<bool>("manual_stance_override_active", false);
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckManualStanceOverride", false,
      "remaining_sec=" + std::to_string(remaining_sec) + " (timeout)", branch);
    return BT::NodeStatus::FAILURE;
  }

  // 所有条件满足,强化覆盖生效。
  std::ostringstream oss;
  oss << "override_stance=" << stanceName(override_stance)
      << ", remaining_sec=" << remaining_sec;
  detail::logTransition(detail::TreeKind::STANCE, "CheckManualStanceOverride", true, oss.str(), branch);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckShouldEnhanceStance -------------------
CheckShouldEnhanceStance::CheckShouldEnhanceStance(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckShouldEnhanceStance::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_stance", "Target stance: ATTACK/DEFEND/MOVE"),
    BT::InputPort<int>("accumulated_threshold", 170, "Accumulated time threshold for level-1 trigger (sec)"),
    BT::InputPort<int>("fallback_game_time", 60, "Game time threshold for level-2 trigger (sec)"),
    BT::InputPort<int>("min_remaining_sec", 1, "Minimum enhanced stance quota (sec)"),
    BT::InputPort<bool>("force_if_available", false, "Enhance whenever energy and quota are available"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")
  };
}

BT::NodeStatus CheckShouldEnhanceStance::tick()
{
  const auto blackboard = config().blackboard;
  const std::string target = getInput<std::string>("target_stance").value_or("ATTACK");
  const int acc_threshold = getInput<int>("accumulated_threshold").value_or(170);
  const int fallback_time = getInput<int>("fallback_game_time").value_or(60);
  const int min_remaining = getInput<int>("min_remaining_sec").value_or(1);
  const bool force_if_available = getInput<bool>("force_if_available").value_or(false);
  const std::string branch = getInput<std::string>("branch").value_or("");

  // 1. 获取对应的累计时间和强化姿态名称
  double accumulated_time = 0.0;
  SentryStance enhanced_stance = SentryStance::ENHANCED_ATTACK;

  if (target == "ATTACK") {
    accumulated_time = blackboard->get<double>("attack_accumulated_time");
    enhanced_stance = SentryStance::ENHANCED_ATTACK;
  } else if (target == "DEFEND") {
    accumulated_time = blackboard->get<double>("defend_accumulated_time");
    enhanced_stance = SentryStance::ENHANCED_DEFEND;
  } else if (target == "MOVE") {
    accumulated_time = blackboard->get<double>("move_accumulated_time");
    enhanced_stance = SentryStance::ENHANCED_MOVE;
  }

  // 2. 获取对应的强化配额
  int remaining_time = 0;
  switch (enhanced_stance) {
  case SentryStance::ENHANCED_ATTACK:
    remaining_time = blackboard->get<int>("enhanced_attack_remaining_time");
    break;
  case SentryStance::ENHANCED_DEFEND:
    remaining_time = blackboard->get<int>("enhanced_defend_remaining_time");
    break;
  case SentryStance::ENHANCED_MOVE:
    remaining_time = blackboard->get<int>("enhanced_move_remaining_time");
    break;
  default:
    break;
  }

  // 3. 检查能量
  const auto energy_ratio = blackboard->get<EnergyRatio>("energy_ratio");
  if (energy_ratio == EnergyRatio::BELOW_1) {
    detail::logTransition(
      detail::TreeKind::STANCE, "CheckShouldEnhanceStance", false,
      "target=" + target + ", energy_insufficient", branch);
    return BT::NodeStatus::FAILURE;
  }

  // 4. 检查配额
  if (remaining_time < min_remaining) {
    std::ostringstream oss;
    oss << "target=" << target << ", quota_exhausted: remaining=" << remaining_time
        << " < min=" << min_remaining;
    detail::logTransition(detail::TreeKind::STANCE, "CheckShouldEnhanceStance", false, oss.str(), branch);
    return BT::NodeStatus::FAILURE;
  }

  if (force_if_available) {
    std::ostringstream oss;
    oss << "target=" << target << ", should_enhance=true, quota=" << remaining_time
        << ", trigger=forced_available";
    detail::logTransition(detail::TreeKind::STANCE, "CheckShouldEnhanceStance", true, oss.str(), branch);
    return BT::NodeStatus::SUCCESS;
  }

  // 5. 触发条件判断（两级）
  const int game_time_remaining = blackboard->get<int>("game_time_remaining");

  // 一级触发：普通姿态累计快到 180s（效果降级前续命）
  // 暂时禁用一级触发，只保留二级保底触发
  // const bool level1_trigger = accumulated_time >= static_cast<double>(acc_threshold);
  const bool level1_trigger = false;  // 暂时禁用

  // 二级触发：比赛剩余 <= 保底时间，且配额 >= 5s（避免在最后几秒浪费开启）
  const bool level2_trigger = (game_time_remaining <= fallback_time && remaining_time >= 5);

  if (level1_trigger || level2_trigger) {
    std::ostringstream oss;
    oss << "target=" << target << ", should_enhance=true"
        << ", accumulated=" << accumulated_time
        << ", game_remaining=" << game_time_remaining
        << ", quota=" << remaining_time
        << ", trigger=" << (level1_trigger ? "L1(accumulated)" : "L2(fallback)");
    detail::logTransition(detail::TreeKind::STANCE, "CheckShouldEnhanceStance", true, oss.str(), branch);
    return BT::NodeStatus::SUCCESS;
  }

  std::ostringstream oss;
  oss << "target=" << target << ", not_triggered"
      << ", accumulated=" << accumulated_time << "/" << acc_threshold
      << ", game_remaining=" << game_time_remaining << "/" << fallback_time
      << ", quota=" << remaining_time;
  detail::logTransition(detail::TreeKind::STANCE, "CheckShouldEnhanceStance", false, oss.str(), branch);
  return BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
