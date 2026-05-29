#include "bt_manager/action/change_stance_action.hpp"

#include <algorithm>
#include <cmath>

using namespace color_text;
namespace Sentry_BT {
std::chrono::time_point<std::chrono::system_clock> ChangeStance::last_change_time_ =
  std::chrono::time_point<std::chrono::system_clock>::min();

// ------------------- SetGyroState -------------------
SetGyroState::SetGyroState(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetGyroState::providedPorts()
{
  return {
    BT::InputPort<bool>("use_gyro", "Whether to enable gyro mode"),
    BT::InputPort<float>("gyro_vel", "Gyro speed in rpm"),
    BT::InputPort<bool>("random_speed", false, "Enable random gyro speed switching"),
    BT::InputPort<float>("change_speed_duration", 0.0f, "Random speed change interval (s)"),
  };
}

bool SetGyroState::shouldReverseRotation(
  bool use_gyro, int ammo_purchase_total, int bullets_remaining, int game_time_remaining)
{
  if (!use_gyro) {
    reverse_initialized_ = false;
    return false;
  }

  constexpr int kInitialAmmoAllowance = 300;
  constexpr int kMatchDurationSec = 420;
  constexpr int kAmmoAllowanceIntervalSec = 60;
  constexpr int kAmmoAllowancePerInterval = 100;
  const int elapsed_time = std::max(0, kMatchDurationSec - game_time_remaining);
  const int time_bonus_ammo = (elapsed_time / kAmmoAllowanceIntervalSec) * kAmmoAllowancePerInterval;
  const int fired_count =
    std::max(0, kInitialAmmoAllowance + ammo_purchase_total + time_bonus_ammo - bullets_remaining);
  if (fired_count <= 200) {
    reverse_initialized_ = false;
    return false;
  }

  constexpr double kMinReverseHz = 0.1;
  constexpr double kMaxReverseHz = 0.2;
  const double ratio = std::clamp((static_cast<double>(fired_count) - 200.0) / 100.0, 0.0, 1.0);
  const double reverse_hz = kMinReverseHz + (kMaxReverseHz - kMinReverseHz) * ratio;

  const auto now = std::chrono::steady_clock::now();
  if (!reverse_initialized_) {
    reverse_initialized_ = true;
    reverse_start_time_ = now;
  }

  const double elapsed = std::chrono::duration<double>(now - reverse_start_time_).count();
  const auto phase = static_cast<int>(std::floor(elapsed * reverse_hz));
  return (phase % 2) == 0;
}

BT::NodeStatus SetGyroState::tick()
{
  auto blackboard = config().blackboard;
  const bool use_gyro = getInput<bool>("use_gyro").value_or(blackboard->get<bool>("use_gyro_mode"));
  const float gyro_vel = getInput<float>("gyro_vel").value_or(blackboard->get<float>("gyro_vel"));
  const bool random_speed = getInput<bool>("random_speed").value_or(false);
  const int ammo_purchase_total = blackboard->get<int>("ammo_purchase_total");
  const int bullets_remaining = blackboard->get<int>("bullets_remaining");
  const int game_time_remaining = blackboard->get<int>("game_time_remaining");

  random_initialized_ = false;
  random_speed_enabled_ = false;
  current_gyro_vel_ = gyro_vel;

  const bool reverse_now =
    shouldReverseRotation(use_gyro, ammo_purchase_total, bullets_remaining, game_time_remaining);
  // const bool reverse_now = false;
  const float base_speed = reverse_now ? -current_gyro_vel_ : current_gyro_vel_;
  float output_gyro_vel = base_speed;
  if (random_speed) {
    const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
    const auto & q = current_pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const float speed_scale = 1.0f + 0.3f * static_cast<float>(std::sin(yaw));
    output_gyro_vel = base_speed * speed_scale;
  }

  blackboard->set("use_gyro_mode", use_gyro);
  blackboard->set("gyro_vel", output_gyro_vel);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- TunnelGyroAlignAction -------------------
TunnelGyroAlignAction::TunnelGyroAlignAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

BT::PortsList TunnelGyroAlignAction::providedPorts()
{
  return {BT::InputPort<float>("kp", 35.0f, "PID Kp for tunnel gyro control"),
    BT::InputPort<float>("ki", 0.5f, "PID Ki for tunnel gyro control"),
    BT::InputPort<float>("kd", 5.0f, "PID Kd for tunnel gyro control"),
    BT::InputPort<float>("deadzone_rad", 0.03f, "Yaw deadzone in radians (~1.7 deg)"),
    BT::InputPort<float>("max_abs_gyro_vel", 120.0f, "Max absolute gyro velocity in rpm"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
    BT::OutputPort<bool>("use_gyro", "Enable gyro output"),
    BT::OutputPort<float>("gyro_vel", "Gyro velocity output")};
}

float TunnelGyroAlignAction::computeTunnelGyroVelPid(double yaw_error,
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
  if (dt > 2.0) {
    integral_error_ = 0.0;
    last_error_ = yaw_error;
    dt = 1e-3;
  } else {
    dt = std::clamp(dt, 1e-3, 0.2);
  }
  last_pid_time_ = now;

  const bool in_deadzone = std::fabs(yaw_error) <= static_cast<double>(deadzone);
  if (!in_deadzone) {
    integral_error_ += yaw_error * dt;
    integral_error_ = std::clamp(integral_error_, -2.0, 2.0);
  }

  const double error_for_pid = in_deadzone ? 0.0 : yaw_error;
  double delta_error = yaw_error - last_error_;
  while (delta_error > M_PI / 2.0) {
    delta_error -= M_PI;
  }
  while (delta_error < -M_PI / 2.0) {
    delta_error += M_PI;
  }
  const double derivative = in_deadzone ? 0.0 : (delta_error / dt);
  const double pid_out_rad = static_cast<double>(kp) * error_for_pid +
                             static_cast<double>(ki) * integral_error_ +
                             static_cast<double>(kd) * derivative;
  last_error_ = yaw_error;

  constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);
  const double pid_out_rpm = pid_out_rad * kRadPerSecToRpm;

  return static_cast<float>(
    std::clamp(pid_out_rpm, -static_cast<double>(max_abs_gyro_vel), static_cast<double>(max_abs_gyro_vel)));
}

void TunnelGyroAlignAction::resetPidState()
{
  pid_initialized_ = false;
  target_yaw_selected_ = false;
  selected_target_yaw_ = 0.0;
  integral_error_ = 0.0;
  last_error_ = 0.0;
}

BT::NodeStatus TunnelGyroAlignAction::tick()
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
  const float ki = getInput<float>("ki").value_or(0.5f);
  const float kd = getInput<float>("kd").value_or(5.0f);
  const float deadzone = getInput<float>("deadzone_rad").value_or(0.03f);
  const float max_abs_gyro_vel = getInput<float>("max_abs_gyro_vel").value_or(120.0f);

  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  const int active_tunnel_idx = blackboard->get<int>("nearest_tunnel_idx");
  if (active_tunnel_idx < 0) {
    resetPidState();
    detail::logTransition(
      detail::TreeKind::STANCE, "TunnelGyroAlignAction", false, "invalid tunnel_idx", branch);
    return BT::NodeStatus::FAILURE;
  }

  const double configured_target_yaw = static_cast<double>(
    tunnel_recovery_configs[static_cast<std::size_t>(active_tunnel_idx)].tunnel_pass_yaw_target_rad);
  const Point2D current_point{current_pose.position.x, current_pose.position.y, 0.0};
  if (!target_yaw_selected_) {
    selected_target_yaw_ =
      (enemy_defense_zone.contains(current_point) || own_defense_zone.contains(current_point))
        ? configured_target_yaw
        : wrapAngle(configured_target_yaw + M_PI);
    target_yaw_selected_ = true;
  }
  const double base_target_yaw = selected_target_yaw_;
  const double current_yaw = yawFromQuaternion(current_pose.orientation);
  const double yaw_error = wrapAngle(base_target_yaw - current_yaw);
  // std::cout << CYAN << "[STANCE_TREE]" << GREEN << "TunnelGyroAlignAction: tunnel_idx=" <<
  // active_tunnel_idx
  //           << ", current_yaw=" << current_yaw << ", target_yaw=" << base_target_yaw << ", yaw_error=" <<
  //           yaw_error
  //           << RESET << std::endl;
  // const double error_forward = wrapAngle(base_target_yaw - current_yaw);
  // const double error_backward = wrapAngle(base_target_yaw + M_PI - current_yaw);
  // double yaw_error = 0.0;
  // if (!pid_initialized_) {
  //   yaw_error = (std::abs(error_forward) <= std::abs(error_backward)) ? error_forward : error_backward;
  // } else {
  //   const double forward_abs = std::abs(error_forward);
  //   const double backward_abs = std::abs(error_backward);
  //   if (forward_abs < (backward_abs - 0.5)) {
  //     yaw_error = error_forward;
  //   } else if (backward_abs < (forward_abs - 0.5)) {
  //     yaw_error = error_backward;
  //   } else {
  //     const double forward_delta = std::abs(wrapAngle(error_forward - last_error_));
  //     const double backward_delta = std::abs(wrapAngle(error_backward - last_error_));
  //     yaw_error = (forward_delta <= backward_delta) ? error_forward : error_backward;
  //   }
  // }

  const auto now = std::chrono::steady_clock::now();
  const float computed_gyro_vel =
    computeTunnelGyroVelPid(yaw_error, kp, ki, kd, deadzone, max_abs_gyro_vel, now);

  blackboard->set("use_gyro_mode", true);
  blackboard->set("gyro_vel", computed_gyro_vel);
  setOutput("use_gyro", true);
  setOutput("gyro_vel", computed_gyro_vel);

  std::ostringstream oss;
  oss << "tunnel_idx=" << active_tunnel_idx << ", gyro_vel=" << computed_gyro_vel
      << ", yaw_error=" << yaw_error;
  detail::logTransition(detail::TreeKind::STANCE, "TunnelGyroAlignAction", true, oss.str(), branch);

  return BT::NodeStatus::RUNNING;
}

void TunnelGyroAlignAction::halt()
{
  resetPidState();
}

// ------------------- ChangeStance -------------------
ChangeStance::ChangeStance(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ChangeStance::providedPorts()
{
  return {BT::InputPort<std::string>("stance", "Target stance: ATTACK/DEFEND/MOVE")};
}

BT::NodeStatus ChangeStance::onStart()
{
  auto parse_stance = [](const std::string & value) -> Sentry_BT::SentryStance {
    if (value == "ATTACK" || value == "attack" || value == "1") {
      return SentryStance::ATTACK;
    }
    if (value == "DEFEND" || value == "defend" || value == "2") {
      return SentryStance::DEFEND;
    }
    if (value == "MOVE" || value == "move" || value == "3") {
      return SentryStance::MOVE;
    }
    return SentryStance::DEFEND;
  };

  auto blackboard = config().blackboard;
  const std::string stance_str = getInput<std::string>("stance").value_or("DEFEND");
  desired_stance_ = parse_stance(stance_str);
  const auto energy_ratio = blackboard->get<EnergyRatio>("energy_ratio");
  if (energy_ratio == EnergyRatio::BELOW_1) {
    desired_stance_ = SentryStance::MOVE;
  }
  current_stance_ = blackboard->get<SentryStance>("current_stance");

  if (current_stance_ == desired_stance_) {
    blackboard->set<SentryStance>("desired_stance", desired_stance_);
    return BT::NodeStatus::SUCCESS;
  }
  return applyStanceChange();
}

BT::NodeStatus ChangeStance::onRunning()
{
  return applyStanceChange();
}

void ChangeStance::onHalted()
{
}

BT::NodeStatus ChangeStance::applyStanceChange()
{
  auto stance_to_string = [](SentryStance stance) -> std::string {
    const auto index = static_cast<size_t>(stance);
    if (index >= 1 && index <= stance_names.size()) {
      return stance_names[index - 1];
    }
    return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
  };

  auto blackboard = config().blackboard;
  const auto energy_ratio = blackboard->get<EnergyRatio>("energy_ratio");
  if (energy_ratio == EnergyRatio::BELOW_1) {
    desired_stance_ = SentryStance::MOVE;
  }
  current_stance_ = blackboard->get<SentryStance>("current_stance");
  blackboard->set<SentryStance>("desired_stance", desired_stance_);
  if (current_stance_ == desired_stance_) {
    return BT::NodeStatus::SUCCESS;
  }

  // blackboard->set<SentryStance>("current_stance", desired_stance_);
  // std::cout << MAGENTA << "[STANCE_TREE]" << GREEN << "Change from stance "
  //           << stance_to_string(current_stance_) << " to stance " << stance_to_string(desired_stance_)
  //           << RESET << std::endl;
  last_change_time_ = std::chrono::system_clock::now();
  return BT::NodeStatus::SUCCESS;
}

// ------------------- UpdateEnhanceTime -------------------
UpdateEnhanceTime::UpdateEnhanceTime(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList UpdateEnhanceTime::providedPorts()
{
  return {};
}

BT::NodeStatus UpdateEnhanceTime::tick()
{
  auto blackboard = config().blackboard;
  const int current_time = blackboard->get<int>("game_time_remaining");
  const int game_status = blackboard->get<int>("game_status");

  constexpr int kMaxGameTime = 420;
  const bool game_active = (game_status == 4) || (current_time > 0 && current_time <= kMaxGameTime);

  if (last_game_time_ < 0) {
    last_game_time_ = current_time;
    return BT::NodeStatus::SUCCESS;
  }

  if (!game_active) {
    last_game_time_ = current_time;
    return BT::NodeStatus::SUCCESS;
  }

  const int raw_delta = last_game_time_ - current_time;
  last_game_time_ = current_time;

  constexpr int kMaxReasonableDelta = 5;
  if (raw_delta <= 0 || raw_delta > kMaxReasonableDelta) {
    return BT::NodeStatus::SUCCESS;
  }

  const double delta = static_cast<double>(raw_delta);
  const auto current_stance = blackboard->get<SentryStance>("current_stance");

  switch (current_stance) {
  case SentryStance::ATTACK: {
    const double value = blackboard->get<double>("attack_accumulated_time");
    blackboard->set("attack_accumulated_time", value + delta);
    break;
  }
  case SentryStance::DEFEND: {
    const double value = blackboard->get<double>("defend_accumulated_time");
    blackboard->set("defend_accumulated_time", value + delta);
    break;
  }
  case SentryStance::MOVE: {
    const double value = blackboard->get<double>("move_accumulated_time");
    blackboard->set("move_accumulated_time", value + delta);
    break;
  }
  default:
    break;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
