#include "minco_core/recovery_behaivor.hpp"

#include <cmath>

namespace minco_planner
{

RecoverServer::RecoverServer() = default;

RecoverServer::~RecoverServer() = default;

void RecoverServer::configure(int32_t fail_threshold, double cooldown_sec, double recovery_window_sec)
{
	fail_threshold_ = (fail_threshold > 0) ? fail_threshold : 1;
	cooldown_sec_ = (std::isfinite(cooldown_sec) && cooldown_sec >= 0.0) ? cooldown_sec : 0.0;
	recovery_window_sec_ =
		(std::isfinite(recovery_window_sec) && recovery_window_sec > 0.0) ? recovery_window_sec : 0.5;
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

RecoverServer::RecoveryDecision RecoverServer::handleReplanFailure(
	double now_s,
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

  if (consecutive_failures_ < fail_threshold_) {
    return false;
  }

  if (last_recovery_end_time_ < 0.0) {
    return true;
  }

  return (now_s - last_recovery_end_time_) >= cooldown_sec_;
}

bool RecoverServer::inRecovery(double now_s) const
{
  if (!recovery_active_ || !isTimeValid(now_s) || last_recovery_start_time_ < 0.0) {
    return false;
  }

  return (now_s - last_recovery_start_time_) <= recovery_window_sec_;
}

int32_t RecoverServer::consecutiveFailures() const
{
	return consecutive_failures_;
}

bool RecoverServer::isTimeValid(double now_s) const
{
	return std::isfinite(now_s) && now_s >= 0.0;
}

bool RecoverServer::calculateEscapeVelocity(
	const geometry_msgs::msg::PoseStamped & current_pose,
	const EsdfQueryFunc & esdf_func,
	Eigen::Vector2d & escape_vel_out) const
{
	if (!esdf_func) {
		return false;
	}

	const double cx = current_pose.pose.position.x;
	const double cy = current_pose.pose.position.y;

	double max_esdf = -1e9;
	Eigen::Vector2d best_dir(0.0, 0.0);

	// 从 0.2m 到 0.5m，步长 0.05m 进行多圈遍历搜索
	for (double r = 0.2; r <= 0.5 + 1e-5; r += 0.05) {
		// 每圈探测 8 个方向 (45度间隔)
		for (int i = 0; i < 8; ++i) {
			double angle = i * (M_PI / 4.0);
			Eigen::Vector2d dir(std::cos(angle), std::sin(angle));
			// 计算当前采样点的世界坐标
			Eigen::Vector3d sample_pt(cx + dir.x() * r, cy + dir.y() * r, 0.0);

			// 查询该点的 ESDF 距离
			double dist = esdf_func(sample_pt);

			// 记录拥有最大 ESDF 距离（最空旷/最安全）的方向
			if (dist > max_esdf) {
				max_esdf = dist;
				best_dir = dir; // 锁存最优逃逸方向
			}
		}
	}

	// 如果所有采样点都在障碍物极深处（例如 ESDF < -10.0），说明定位漂移或陷入死局
	if (max_esdf < -10.0) {
		return false;
	}

	const double escape_speed = 0.4;
	escape_vel_out = best_dir * escape_speed;

	return true;
}

}  // namespace minco_planner
