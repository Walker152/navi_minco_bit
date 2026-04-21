#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/utils/area.hpp"

#include <chrono>
#include <cmath>

using namespace color_text;

namespace Sentry_BT {
namespace {
inline bool compareByMode(const float lhs, const float rhs, const std::string & mode)
{
  if (mode == "greater") {
    return lhs > rhs;
  }
  if (mode == "less") {
    return lhs < rhs;
  }
  return false;
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
    BT::InputPort<std::string>("mode", "greater", "greater/less")};
}

BT::NodeStatus CheckHeat::tick()
{
  const auto blackboard = config().blackboard;
  const float threshold = getInput<float>("threshold").value_or(200.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

  const int current_heat = blackboard->get<int>("current_heat");

  return compareByMode(static_cast<float>(current_heat), threshold, mode) ? BT::NodeStatus::SUCCESS
                                                                            : BT::NodeStatus::FAILURE;
}

// ------------------- CheckOutpostTarget -------------------
CheckOutpostTarget::CheckOutpostTarget(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckOutpostTarget::providedPorts()
{
  return {BT::InputPort<float>("goal_distance_threshold", 0.8f, "Goal close-to-outpost threshold")};
}

BT::NodeStatus CheckOutpostTarget::tick()
{
  const auto blackboard = config().blackboard;

  int current_mode = 0;
  geometry_msgs::msg::Pose current_pose;
  Sentry_BT::Point2D nav_goal;
  current_mode = blackboard->get<int>("current_mode");
  current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");

  const bool attacking_outpost_mode = current_mode == static_cast<int>(Sentry_BT::NavMode::RESPONSE);
  const bool in_outpost_zone =
    enemy_outpost_watch_zone.contains({current_pose.position.x, current_pose.position.y, 0.0});

  const float dist_threshold = getInput<float>("goal_distance_threshold").value_or(0.8f);
  const auto & outpost = nav_points[static_cast<size_t>(Sentry_BT::NavGoal::OUTPOST)];
  const bool nav_goal_is_outpost =
    std::hypot(nav_goal.x - outpost.x, nav_goal.y - outpost.y) <= static_cast<double>(dist_threshold);

  return (attacking_outpost_mode || in_outpost_zone || nav_goal_is_outpost) ? BT::NodeStatus::SUCCESS
                                                                              : BT::NodeStatus::FAILURE;
}

// ------------------- CheckEngagedStatus -------------------
CheckEngagedStatus::CheckEngagedStatus(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEngagedStatus::providedPorts()
{
  return {BT::InputPort<bool>("expected", true, "Expected engaged status")};
}

BT::NodeStatus CheckEngagedStatus::tick()
{
  const auto blackboard = config().blackboard;
  const bool expected_engaged = getInput<bool>("expected").value_or(true);

  const bool is_disengaged = blackboard->get<bool>("is_disengaged");
  const bool engaged = !is_disengaged;

  return (engaged == expected_engaged) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckHealth -------------------
CheckHealth::CheckHealth(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckHealth::providedPorts()
{
  return {BT::InputPort<float>("threshold", 50.0f, "Health threshold"),
    BT::InputPort<std::string>("mode", "greater", "greater/less")};
}

BT::NodeStatus CheckHealth::tick()
{
  const auto blackboard = config().blackboard;

  const float health = blackboard->get<float>("health");

  const float threshold = getInput<float>("threshold").value_or(50.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

  return compareByMode(health, threshold, mode) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckTargetDistance -------------------
CheckTargetDistance::CheckTargetDistance(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTargetDistance::providedPorts()
{
  return {BT::InputPort<float>("threshold", 1.0f, "Distance threshold in meters"),
    BT::InputPort<std::string>("mode", "greater", "greater/less")};
}

BT::NodeStatus CheckTargetDistance::tick()
{
  const auto blackboard = config().blackboard;

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");

  const float threshold = getInput<float>("threshold").value_or(1.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

  const float distance = static_cast<float>(
    std::hypot(target_pose.position.x - current_pose.position.x, target_pose.position.y - current_pose.position.y));
  return compareByMode(distance, threshold, mode) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckCrossZoneTransition -------------------
CheckCrossZoneTransition::CheckCrossZoneTransition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCrossZoneTransition::providedPorts()
{
  return {};
}

BT::NodeStatus CheckCrossZoneTransition::tick()
{
  const auto blackboard = config().blackboard;

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");

  const Point2D current_point{current_pose.position.x, current_pose.position.y, 0.0};
  const Point2D goal_point{nav_goal.x, nav_goal.y, 0.0};

  const bool current_in_highland = highland_zone.contains(current_point);
  const bool current_in_half =
    own_defense_zone.contains(current_point) || enemy_defense_zone.contains(current_point);
  const bool goal_in_highland = highland_zone.contains(goal_point);
  const bool goal_in_half = own_defense_zone.contains(goal_point) || enemy_defense_zone.contains(goal_point);

  const bool need_cross_zone = (current_in_half && goal_in_highland) || (current_in_highland && goal_in_half);
  return need_cross_zone ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    BT::InputPort<std::string>("mode", "less", "greater/less")};
}

BT::NodeStatus CheckCapacitorCapacity::tick()
{
  const auto blackboard = config().blackboard;

  const float capacitor_capacity = blackboard->get<float>("capacitor_capacity");

  const float threshold = getInput<float>("threshold").value_or(30.0f);
  const std::string mode = getInput<std::string>("mode").value_or("less");

  return compareByMode(capacitor_capacity, threshold, mode) ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
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
