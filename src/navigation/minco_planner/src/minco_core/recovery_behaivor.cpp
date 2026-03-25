#include "minco_core/recovery_behaivor.hpp"

#include <cmath>
#include <iostream>

namespace minco_planner
{

RecoverServer::RecoverServer() = default;

RecoverServer::~RecoverServer() = default;

void RecoverServer::configure(const Config & config)
{
	config_.fail_threshold = (config.fail_threshold > 0) ? config.fail_threshold : 1;
	config_.cooldown_sec =
		(std::isfinite(config.cooldown_sec) && config.cooldown_sec >= 0.0) ? config.cooldown_sec : 0.0;
	config_.recovery_window_sec =
		(std::isfinite(config.recovery_window_sec) && config.recovery_window_sec > 0.0) ?
		config.recovery_window_sec :
		0.5;
	config_.search_min_dist =
		(std::isfinite(config.search_min_dist) && config.search_min_dist >= 0.0) ?
		config.search_min_dist :
		0.2;
	config_.search_step =
		(std::isfinite(config.search_step) && config.search_step > 1e-6) ? config.search_step : 0.05;
	config_.search_max_dist =
		(std::isfinite(config.search_max_dist) &&
		config.search_max_dist >= config_.search_min_dist + config_.search_step) ?
		config.search_max_dist :
		(config_.search_min_dist + config_.search_step);
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

	// 在可配置半径区间内进行多圈遍历搜索
	for (double r = config_.search_min_dist; r <= config_.search_max_dist + 1e-5;
		r += config_.search_step)
	{
		// 每圈探测 8 个方向 (45度间隔)
		for (int i = 0; i < 8; ++i) {
			double angle = i * (M_PI / 4.0);
			Eigen::Vector2d dir(std::cos(angle), std::sin(angle));
			// 计算当前采样点的世界坐标
			Eigen::Vector3d sample_pt(cx + dir.x() * r, cy + dir.y() * r, 0.0);

			// 查询该点的 ESDF 距离
			double dist = esdf_func(sample_pt);
			// std::cout << "Sample point: (" << sample_pt.x() << ", " << sample_pt.y() << "), ESDF: " << dist << std::endl;
			// 记录拥有最大 ESDF 距离（最空旷/最安全）的方向
			if (dist > max_esdf) {
				max_esdf = dist;
				best_dir = dir; // 锁存最优逃逸方向
			}
		}
	}
	// std::cout << "Max ESDF in search area: " << max_esdf << std::endl;
	// std::cout << "Best escape direction: " << best_dir.transpose() << std::endl;
	// 如果所有采样点都在障碍物极深处（例如 ESDF < -10.0），说明定位漂移或陷入死局
	if (max_esdf < -10.0) {
		return false;
	}

	escape_vel_out = best_dir * config_.escape_speed;

	return true;
}

}  // namespace minco_planner
