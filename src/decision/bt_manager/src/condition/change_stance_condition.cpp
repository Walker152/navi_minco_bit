#include "bt_manager/condition/change_stance_condition.hpp"
#include <algorithm>
#include <cmath>
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

  const float threshold = getInput<float>("threshold").value_or(1.0f);
  const std::string mode = getInput<std::string>("mode").value_or("greater");

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
  return {
    BT::InputPort<float>("kp", 35.0f, "PID Kp for tunnel gyro control"),
    BT::InputPort<float>("ki", 0.0f, "PID Ki for tunnel gyro control"),
    BT::InputPort<float>("kd", 5.0f, "PID Kd for tunnel gyro control"),
    BT::InputPort<float>("deadzone_rad", 0.08f, "Yaw deadzone in radians"),
    BT::InputPort<float>("max_abs_gyro_vel", 120.0f, "Max absolute gyro velocity in rpm"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
    BT::OutputPort<bool>("use_gyro", "Enable gyro output"),
    BT::OutputPort<float>("gyro_vel", "Gyro velocity output")};
}

float CheckCrossZoneTransition::computeTunnelGyroVelPid(
  double yaw_error,
  float kp,
  float ki,
  float kd,
  float deadzone,
  float max_abs_gyro_vel,
  const std::chrono::steady_clock::time_point & now)
{
  if (!pid_initialized_) {
    pid_initialized_ = true;
    integral_error_ = 0.0;
    last_error_ = yaw_error;
    last_pid_time_ = now;
  }

  double dt = std::chrono::duration<double>(now - last_pid_time_).count();
  if (dt > 0.5) {
    integral_error_ = 0.0;
    last_error_ = yaw_error;
    dt = 1e-3;
  } else {
    dt = std::clamp(dt, 1e-3, 0.2);
  }
  last_pid_time_ = now;

  if (std::fabs(yaw_error) <= static_cast<double>(deadzone)) {
    integral_error_ = 0.0;
    last_error_ = yaw_error;
    return 0.0f;
  }

  integral_error_ += yaw_error * dt;
  const double derivative = (yaw_error - last_error_) / dt;
  const double pid_out_rad = static_cast<double>(kp) * yaw_error +
                                     static_cast<double>(ki) * integral_error_ +
                                     static_cast<double>(kd) * derivative;
  last_error_ = yaw_error;

  constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);
  const double pid_out_rpm = pid_out_rad * kRadPerSecToRpm;

  return static_cast<float>(
    std::clamp(pid_out_rpm, -static_cast<double>(max_abs_gyro_vel), static_cast<double>(max_abs_gyro_vel)));
}

void CheckCrossZoneTransition::resetPidState()
{
  pid_initialized_ = false;
  integral_error_ = 0.0;
  last_error_ = 0.0;
}

BT::NodeStatus CheckCrossZoneTransition::tick()
{
  auto wrapAngle = [](double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  };
  auto yawFromQuaternion = [](const geometry_msgs::msg::Quaternion & q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  };

  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const float kp = getInput<float>("kp").value_or(35.0f);
  const float ki = getInput<float>("ki").value_or(0.0f);
  const float kd = getInput<float>("kd").value_or(5.0f);
  const float deadzone = getInput<float>("deadzone_rad").value_or(0.08f);
  const float max_abs_gyro_vel = getInput<float>("max_abs_gyro_vel").value_or(120.0f);

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const auto nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");
  const bool through_tunnel = blackboard->get<bool>("through_tunnel");

  const Point2D current_point{current_pose.position.x, current_pose.position.y, 0.0};
  const Point2D goal_point{nav_goal.x, nav_goal.y, 0.0};

  const bool current_in_highland = highland_zone.contains(current_point);
  const bool current_in_own = own_defense_zone.contains(current_point);
  const bool current_in_enemy = enemy_defense_zone.contains(current_point);
  const bool goal_in_highland = highland_zone.contains(goal_point);
  const bool goal_in_own = own_defense_zone.contains(goal_point);
  const bool goal_in_enemy = enemy_defense_zone.contains(goal_point);
  const bool same_zone =
    (current_in_highland && goal_in_highland) || (current_in_own && goal_in_own) ||
    (current_in_enemy && goal_in_enemy);
  const bool need_cross_zone = !same_zone;
  bool current_in_tunnel = false;
  for (const auto & zone : tunnel_zone) {
    if (zone.contains(current_point)) {
      current_in_tunnel = true;
      break;
    }
  }
  const bool is_tunnel_journey = (need_cross_zone && through_tunnel) || current_in_tunnel;

  float computed_gyro_vel = 0.0f;
  bool enable_small_gyro = false;
  int active_tunnel_idx = -1;
  if (is_tunnel_journey) {
    if (current_in_tunnel) {
      int nearest_tunnel_idx = -1;
      double nearest_dist2 = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < tunnel_zone.size(); ++i) {
        const auto & zone = tunnel_zone[i];
        const double center_x = (zone.top_left.x + zone.bottom_right.x) * 0.5;
        const double center_y = (zone.top_left.y + zone.bottom_right.y) * 0.5;
        const double dx = current_point.x - center_x;
        const double dy = current_point.y - center_y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < nearest_dist2) {
          nearest_dist2 = dist2;
          nearest_tunnel_idx = static_cast<int>(i);
        }
        if (tunnel_zone[i].contains(current_point)) {
          active_tunnel_idx = static_cast<int>(i);
          break;
        }
      }
      if (active_tunnel_idx < 0) {
        active_tunnel_idx = nearest_tunnel_idx;
      }

      enable_small_gyro = true;

      const double base_target_yaw = static_cast<double>(
        tunnel_recovery_configs[static_cast<std::size_t>(active_tunnel_idx)].tunnel_pass_yaw_target_rad);
      const double current_yaw = yawFromQuaternion(current_pose.orientation);
      const double error_forward = wrapAngle(base_target_yaw - current_yaw);
      const double error_backward = wrapAngle(base_target_yaw + M_PI - current_yaw);
      const double error = (std::abs(error_forward) <= std::abs(error_backward))
                             ? error_forward
                             : error_backward;

      const auto now = std::chrono::steady_clock::now();
      computed_gyro_vel = computeTunnelGyroVelPid(error, kp, ki, kd, deadzone, max_abs_gyro_vel, now);
    } else {
      resetPidState();
      enable_small_gyro = false;
      computed_gyro_vel = 0.0f;
    }
  } else {
    resetPidState();
  }

  setOutput("use_gyro", enable_small_gyro);
  setOutput("gyro_vel", computed_gyro_vel);

  std::ostringstream oss;
    oss << "current_in_highland=" << current_in_highland << ", current_in_own=" << current_in_own
      << ", current_in_enemy=" << current_in_enemy << ", goal_in_highland=" << goal_in_highland
      << ", goal_in_own=" << goal_in_own << ", goal_in_enemy=" << goal_in_enemy
      << ", through_tunnel=" << through_tunnel << ", current_in_tunnel=" << current_in_tunnel
      << ", tunnel_idx=" << active_tunnel_idx
      << ", use_gyro_mode=" << enable_small_gyro << ", gyro_vel=" << computed_gyro_vel;
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

  const float capacitor_capacity =
    static_cast<float>(blackboard->get<uint8_t>("capacitor_capacity"));

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
  const double elapsed =
    (last_change == std::chrono::time_point<std::chrono::system_clock>::min())
      ? std::numeric_limits<double>::infinity()
      : std::chrono::duration<double>(now - last_change).count();

  const bool active = elapsed >= cooldown;
  std::ostringstream oss;
  oss << "elapsed=" << elapsed << ", cooldown=" << cooldown;
  detail::logTransition(detail::TreeKind::STANCE, "CheckStanceCooldown", active, oss.str(), branch);

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckStanceRefreshRequired -------------------
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

}  // namespace Sentry_BT
