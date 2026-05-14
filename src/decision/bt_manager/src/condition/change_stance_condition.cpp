#include "bt_manager/condition/change_stance_condition.hpp"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>

#include "bt_manager/action/change_stance_action.hpp"

namespace Sentry_BT {

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
  const auto & outpost = nav_points[static_cast<size_t>(Sentry_BT::NavGoal::OUTPOST)];
  const bool nav_goal_is_outpost =
    std::hypot(nav_goal.x - outpost.x, nav_goal.y - outpost.y) <= static_cast<double>(dist_threshold);
  const bool active = (attacking_outpost_mode && in_outpost_zone) && nav_goal_is_outpost;

  std::ostringstream oss;
  oss << "mode=" << current_mode << ", in_outpost_zone=" << in_outpost_zone
      << ", nav_goal_is_outpost=" << nav_goal_is_outpost;
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

  if(current_mode != NavMode::TRACING) {
    detail::logTransition(detail::TreeKind::STANCE, "CheckTargetDistance", false, "Not in TRACING mode", branch);
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
  const bool is_disengaged = blackboard->get<bool>("is_disengaged");

  const Point2D current_point{current_pose.position.x, current_pose.position.y, 0.0};
  const Point2D goal_point{nav_goal.x, nav_goal.y, 0.0};

  // Cross-zone detection: check whether current and goal are in different zones
  const bool current_in_highland = highland_zone.contains(current_point);
  const bool current_in_own = own_defense_zone.contains(current_point);
  const bool current_in_enemy = enemy_defense_zone.contains(current_point);
  const bool goal_in_highland = highland_zone.contains(goal_point);
  const bool goal_in_own = own_defense_zone.contains(goal_point);
  const bool goal_in_enemy = enemy_defense_zone.contains(goal_point);
  const bool same_zone = (current_in_highland && goal_in_highland) || (current_in_own && goal_in_own) ||
                         (current_in_enemy && goal_in_enemy);
  const bool need_cross_zone = !same_zone;
  const bool in_transform_zone = blackboard->get<bool>("in_transform_zone");
  bool is_tunnel_journey = false;
  const int active_tunnel_idx = blackboard->get<int>("nearest_tunnel_idx");

  if(current_in_tunnel) {
    is_tunnel_journey = true;
  } else if (in_transform_zone) {
    is_tunnel_journey = is_disengaged;
  } else {
    is_tunnel_journey = need_cross_zone && is_disengaged;
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

// ------------------- CheckEnhanceLimit -------------------
CheckEnhanceLimit::CheckEnhanceLimit(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEnhanceLimit::providedPorts()
{
  return {BT::InputPort<std::string>("target_stance", "ATTACK", "Target stance to check"),
    BT::InputPort<int>("max_time_sec", 180, "Max accumulated seconds")};
}

BT::NodeStatus CheckEnhanceLimit::tick()
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
      detail::TreeKind::STANCE, "CheckEnhanceLimit", true, "unknown target_stance", "");
    return BT::NodeStatus::SUCCESS;
  }

  const bool active = accumulated < static_cast<double>(max_time);
  std::ostringstream oss;
  oss << "target=" << target << ", accumulated=" << accumulated << ", max=" << max_time;
  detail::logTransition(detail::TreeKind::STANCE, "CheckEnhanceLimit", active, oss.str(), "");

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
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  LifterPos desired_pos = LifterPos::TOP;

  if (current_in_tunnel) {
    desired_pos = LifterPos::BOTTOM;
  } else if (in_transform_zone) {
    if(is_disengaged) {
      desired_pos = LifterPos::BOTTOM;
    }
  }

  // std::cout << " in_transform_zone=" << in_transform_zone << "current_pose= (" 
  // << current_pose.position.x << ", " << current_pose.position.y << ")" << std::endl;
  blackboard->set<LifterPos>("desired_lifter_pos", desired_pos);
  detail::logTransition(detail::TreeKind::STANCE, "CheckTunnelDeformation", true,
    "through_tunnel=" + std::to_string(through_tunnel) + ", current_in_tunnel=" + std::to_string(current_in_tunnel) +
    ", is_disengaged=" + std::to_string(is_disengaged) + ", desired_pos=" + std::to_string(static_cast<int>(desired_pos)), "");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
