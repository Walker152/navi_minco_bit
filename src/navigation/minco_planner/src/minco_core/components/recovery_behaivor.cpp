#include "minco_core/components/recovery_behaivor.hpp"

#include <cmath>
#include <iostream>

namespace minco_planner {

RecoverServer::RecoverServer() = default;

RecoverServer::~RecoverServer() = default;

void RecoverServer::configure(const Config & config)
{
  config_.fail_threshold = (config.fail_threshold > 0) ? config.fail_threshold : 1;
  config_.cooldown_sec =
    (std::isfinite(config.cooldown_sec) && config.cooldown_sec >= 0.0) ? config.cooldown_sec : 0.0;
  config_.recovery_window_sec =
    (std::isfinite(config.recovery_window_sec) && config.recovery_window_sec > 0.0)
      ? config.recovery_window_sec
      : 0.5;
  config_.escape_speed =
    (std::isfinite(config.escape_speed) && config.escape_speed > 0.0) ? config.escape_speed : 0.4;
}

void RecoverServer::reset()
{
  consecutive_failures_ = 0;
  last_failure_time_ = -1.0;
  last_recovery_start_time_ = -1.0;
  last_recovery_end_time_ = -1.0;
  recovery_active_ = false;
  recovery_goal_active_ = false;
}

bool RecoverServer::onReplanFailure(double now_s)
{
  if (!isTimeValid(now_s)) {
    return false;
  }

  ++consecutive_failures_;
  last_failure_time_ = now_s;

  if (shouldTryRecovery(now_s)) {
    startRecovery(now_s);
    return true;
  }

  return false;
}

void RecoverServer::onReplanSuccess()
{
  consecutive_failures_ = 0;
  last_failure_time_ = -1.0;
}

void RecoverServer::setMissionGoal(const geometry_msgs::msg::PoseStamped & mission_goal)
{
  mission_goal_ = mission_goal;
  has_mission_goal_ = true;
  reset();
}

void RecoverServer::clearMissionGoal()
{
  has_mission_goal_ = false;
  recovery_goal_active_ = false;
  reset();
}

RecoverServer::RecoveryDecision RecoverServer::handleReplanFailure(double now_s,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const EsdfQueryFunc & esdf_func,
  Eigen::Vector2d & escape_vel_out)
{
  if (!onReplanFailure(now_s)) {
    return RecoveryDecision::NONE;
  }

  if (calculateEscapeVelocity(current_pose, esdf_func, escape_vel_out)) {
    recovery_goal_active_ = true;
    return RecoveryDecision::DO_ESCAPE;
  }

  return RecoveryDecision::ENTER_EMER_STOP;
}

void RecoverServer::startRecovery(double now_s)
{
  if (!isTimeValid(now_s)) {
    return;
  }

  recovery_active_ = true;
  last_recovery_start_time_ = now_s;
}

void RecoverServer::finishRecovery(bool success, double now_s)
{
  (void)success;
  if (!isTimeValid(now_s)) {
    return;
  }

  recovery_active_ = false;
  last_recovery_end_time_ = now_s;

  if (success) {
    consecutive_failures_ = 0;
    last_failure_time_ = -1.0;
  }
}

bool RecoverServer::shouldTryRecovery(double now_s) const
{
  if (!isTimeValid(now_s) || recovery_active_) {
    return false;
  }

  if (consecutive_failures_ < config_.fail_threshold) {
    return false;
  }

  if (last_recovery_end_time_ < 0.0) {
    return true;
  }

  return (now_s - last_recovery_end_time_) >= config_.cooldown_sec;
}

bool RecoverServer::inRecovery(double now_s) const
{
  if (!recovery_active_ || !isTimeValid(now_s) || last_recovery_start_time_ < 0.0) {
    return false;
  }

  return (now_s - last_recovery_start_time_) <= config_.recovery_window_sec;
}

int32_t RecoverServer::consecutiveFailures() const
{
  return consecutive_failures_;
}

bool RecoverServer::isTimeValid(double now_s) const
{
  return std::isfinite(now_s) && now_s >= 0.0;
}

bool RecoverServer::calculateEscapeVelocity(const geometry_msgs::msg::PoseStamped & current_pose,
  const EsdfQueryFunc & esdf_func,
  Eigen::Vector2d & escape_vel_out) const
{
  if (!esdf_func) {
    return false;
  }

  const double cx = current_pose.pose.position.x;
  const double cy = current_pose.pose.position.y;
  constexpr double h = 0.05;

  const double fx_plus = esdf_func(Eigen::Vector3d(cx + h, cy, 0.0));
  const double fx_minus = esdf_func(Eigen::Vector3d(cx - h, cy, 0.0));
  const double fy_plus = esdf_func(Eigen::Vector3d(cx, cy + h, 0.0));
  const double fy_minus = esdf_func(Eigen::Vector3d(cx, cy - h, 0.0));
  if (!std::isfinite(fx_plus) || !std::isfinite(fx_minus) || !std::isfinite(fy_plus) ||
      !std::isfinite(fy_minus)) {
    escape_vel_out.setZero();
    return false;
  }

  const double grad_x = (fx_plus - fx_minus) / (2.0 * h);
  const double grad_y = (fy_plus - fy_minus) / (2.0 * h);
  Eigen::Vector2d grad(grad_x, grad_y);

  const double grad_norm = grad.norm();
  if (!std::isfinite(grad_norm)) {
    escape_vel_out.setZero();
    return false;
  }

  if (grad_norm < 1e-3) {
    constexpr int num_samples = 36;        // 采样数量，每 10 度采一个点
    constexpr double search_radius = 0.4;  // 搜索半径 0.4 米（稍微大于机器人的膨胀半径）

    double max_esdf = -std::numeric_limits<double>::max();
    double best_angle = 0.0;
    bool found_finite_sample = false;

    for (int i = 0; i < num_samples; ++i) {
      const double angle = (2.0 * M_PI * i) / num_samples;
      const double sample_x = cx + search_radius * std::cos(angle);
      const double sample_y = cy + search_radius * std::sin(angle);

      const double current_esdf = esdf_func(Eigen::Vector3d(sample_x, sample_y, 0.0));
      if (!std::isfinite(current_esdf)) {
        continue;
      }
      found_finite_sample = true;

      if (current_esdf > max_esdf) {
        max_esdf = current_esdf;
        best_angle = angle;
      }
    }

    if (!found_finite_sample) {
      escape_vel_out.setZero();
      return false;
    }

    // 朝着最开阔（ESDF 最大）的角度输出逃逸速度
    escape_vel_out.x() = config_.escape_speed * std::cos(best_angle);
    escape_vel_out.y() = config_.escape_speed * std::sin(best_angle);

    std::cout
      << "\033[33m[MincoPlanner] Flat gradient detected, executing circular search recovery! Max ESDF: "
      << max_esdf << " at angle " << best_angle * 180.0 / M_PI << " deg\033[0m" << std::endl;
    return true;
  }

  escape_vel_out = (grad / grad_norm) * config_.escape_speed;

  return true;
}

}  // namespace minco_planner
