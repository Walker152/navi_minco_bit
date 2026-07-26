#include "bt_manager/action/change_stance_action.hpp"

#include <algorithm>
#include <cmath>

using namespace color_text;
namespace Sentry_BT {
namespace {
std::string stanceToString(SentryStance stance)
{
  const auto index = static_cast<size_t>(stance);
  if (index >= 1 && index <= stance_names.size()) {
    return stance_names[index - 1];
  }
  return "UNKNOWN(" + std::to_string(static_cast<int>(stance)) + ")";
}

bool isLegalStance(SentryStance stance)
{
  const int value = static_cast<int>(stance);
  return value >= 1 && value <= 6;
}

// 统一的 ChangeStance 状态日志:
// - 非法 current_stance:借 logTransition 的 active 去重实现边沿日志(进入/退出各一条,中间沉默)。
// - 合法时:打"到位(reached)"或"切换中(from->to)",同样靠 active 去重仅在翻转时输出。
// node_name 用节点实例名,使各 ChangeStance 分支独立去重、互不干扰。
void logStanceState(const std::string & node_name, SentryStance current, SentryStance desired)
{
  const bool illegal = !isLegalStance(current);
  detail::logTransition(detail::TreeKind::STANCE, node_name + " illegal_current_stance", illegal,
    "current=" + stanceToString(current), "");
  if (illegal) {
    return;  // 非法期间不打正常到位/切换日志,避免 UNKNOWN 噪声
  }
  const bool reached = (current == desired);
  detail::logTransition(detail::TreeKind::STANCE, node_name, reached,
    reached ? ("stance=" + stanceToString(desired) + ", reached")
            : ("from=" + stanceToString(current) + " to=" + stanceToString(desired)),
    "");
}
}  // namespace

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
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
    BT::InputPort<bool>(
      "reuse_existing_target", false, "Reuse the alignment target already stored on the blackboard"),
    BT::OutputPort<bool>("tunnel_align_active", "Enable chassis tunnel yaw alignment"),
    BT::OutputPort<float>("tunnel_align_angle_deg", "Absolute chassis alignment yaw in degrees")};
}

void TunnelGyroAlignAction::resetAlignmentState()
{
  target_yaw_selected_ = false;
  selected_target_yaw_ = 0.0;
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
  const auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const bool reuse_existing_target = getInput<bool>("reuse_existing_target").value_or(false);

  blackboard->set("use_gyro_mode", false);
  blackboard->set("gyro_vel", 0.0f);

  int active_tunnel_idx = -1;
  float target_yaw_deg = blackboard->get<float>("tunnel_align_angle_deg");
  if (!reuse_existing_target) {
    const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
    active_tunnel_idx = blackboard->get<int>("nearest_tunnel_idx");
    if (active_tunnel_idx < 0) {
      resetAlignmentState();
      blackboard->set("tunnel_align_active", false);
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
    target_yaw_deg = static_cast<float>(selected_target_yaw_ * 180.0 / M_PI);
  }

  blackboard->set("tunnel_align_active", true);
  blackboard->set("tunnel_align_angle_deg", target_yaw_deg);
  setOutput("tunnel_align_active", true);
  setOutput("tunnel_align_angle_deg", target_yaw_deg);

  std::ostringstream oss;
  oss << "tunnel_idx=" << active_tunnel_idx << ", target_yaw_deg=" << target_yaw_deg
      << ", reused_target=" << reuse_existing_target;
  detail::logTransition(detail::TreeKind::STANCE, "TunnelGyroAlignAction", true, oss.str(), branch);

  return BT::NodeStatus::RUNNING;
}

void TunnelGyroAlignAction::halt()
{
  resetAlignmentState();
  config().blackboard->set("tunnel_align_active", false);
}

// ------------------- ChangeStance -------------------
ChangeStance::ChangeStance(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ChangeStance::providedPorts()
{
  return {BT::InputPort<std::string>(
    "stance", "Target stance: ATTACK/DEFEND/MOVE/ENHANCED_ATTACK/ENHANCED_DEFEND/ENHANCED_MOVE")};
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
    if (
      value == "ENHANCED_ATTACK" || value == "enhanced_attack" || value == "enhance_attack" ||
      value == "POWER_ATTACK" || value == "4") {
      return SentryStance::ENHANCED_ATTACK;
    }
    if (
      value == "ENHANCED_DEFEND" || value == "enhanced_defend" || value == "enhance_defend" ||
      value == "POWER_DEFEND" || value == "5") {
      return SentryStance::ENHANCED_DEFEND;
    }
    if (
      value == "ENHANCED_MOVE" || value == "enhanced_move" || value == "enhance_move" ||
      value == "POWER_MOVE" || value == "6") {
      return SentryStance::ENHANCED_MOVE;
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
    logStanceState(name(), current_stance_, desired_stance_);
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
  auto blackboard = config().blackboard;
  const auto energy_ratio = blackboard->get<EnergyRatio>("energy_ratio");
  if (energy_ratio == EnergyRatio::BELOW_1) {
    desired_stance_ = SentryStance::MOVE;
  }
  current_stance_ = blackboard->get<SentryStance>("current_stance");
  blackboard->set<SentryStance>("desired_stance", desired_stance_);
  logStanceState(name(), current_stance_, desired_stance_);
  if (current_stance_ == desired_stance_) {
    return BT::NodeStatus::SUCCESS;
  }

  last_change_time_ = std::chrono::system_clock::now();
  return BT::NodeStatus::SUCCESS;
}

// ------------------- UpdateStanceDuration -------------------
UpdateStanceDuration::UpdateStanceDuration(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList UpdateStanceDuration::providedPorts()
{
  return {};
}

BT::NodeStatus UpdateStanceDuration::tick()
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

  // 仅根据裁判系统 game_time_remaining 的差值累计姿态停留时长。
  // 强化姿态剩余时间完全由裁判系统反馈管理；强化态仍计入对应的普通姿态桶。
  switch (current_stance) {
  case SentryStance::ATTACK:
  case SentryStance::ENHANCED_ATTACK: {
    const double value = blackboard->get<double>("attack_accumulated_time");
    blackboard->set("attack_accumulated_time", value + delta);
    break;
  }
  case SentryStance::DEFEND:
  case SentryStance::ENHANCED_DEFEND: {
    const double value = blackboard->get<double>("defend_accumulated_time");
    blackboard->set("defend_accumulated_time", value + delta);
    break;
  }
  case SentryStance::MOVE:
  case SentryStance::ENHANCED_MOVE: {
    const double value = blackboard->get<double>("move_accumulated_time");
    blackboard->set("move_accumulated_time", value + delta);
    break;
  }
  default:
    break;
  }

  return BT::NodeStatus::SUCCESS;
}

// ------------------- ApplyManualStanceOverride -------------------
ApplyManualStanceOverride::ApplyManualStanceOverride(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ApplyManualStanceOverride::providedPorts()
{
  return {};
}

BT::NodeStatus ApplyManualStanceOverride::tick()
{
  auto blackboard = config().blackboard;

  // 注:此节点应与 CheckManualStanceOverride 配合使用,CheckManualStanceOverride 已判断所有条件。
  // 此处直接读取操作手指定的强化姿态并写入 desired_stance,不再重复判断。
  const auto override_stance = blackboard->get<SentryStance>("manual_override_stance");
  blackboard->set<SentryStance>("desired_stance", override_stance);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
