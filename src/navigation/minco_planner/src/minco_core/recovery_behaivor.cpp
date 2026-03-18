#include "minco_core/recovery_behaivor.hpp"

#include <array>
#include <cmath>
#include <utility>

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

bool RecoverServer::isRecoveryGoalActive() const
{
	return recovery_goal_active_;
}

RecoverServer::RecoveryDecision RecoverServer::handleReplanFailure(
	double now_s,
	const geometry_msgs::msg::PoseStamped & current_pose,
	const PathFeasibilityChecker & checker,
	geometry_msgs::msg::PoseStamped & recovery_goal_out)
{
	if (!onReplanFailure(now_s)) {
		return RecoveryDecision::NONE;
	}

	if (tryBuildRecoveryGoal(current_pose, checker, recovery_goal_out)) {
		recovery_goal_active_ = true;
		return RecoveryDecision::USE_RECOVERY_GOAL;
	}

	return RecoveryDecision::ENTER_EMER_STOP;
}

RecoverServer::RecoveryGoalReachAction RecoverServer::onRecoveryGoalReached(
	double current_speed,
	double stop_speed_threshold,
	geometry_msgs::msg::PoseStamped & mission_goal_out)
{
	if (!recovery_goal_active_) {
		return RecoveryGoalReachAction::NONE;
	}

	if (!std::isfinite(current_speed) || current_speed >= stop_speed_threshold) {
		return RecoveryGoalReachAction::NONE;
	}

	recovery_goal_active_ = false;

	if (has_mission_goal_) {
		mission_goal_out = mission_goal_;
		return RecoveryGoalReachAction::RESUME_MISSION;
	}

	return RecoveryGoalReachAction::FINISH_NO_GOAL;
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

bool RecoverServer::tryBuildRecoveryGoal(
	const geometry_msgs::msg::PoseStamped & current_pose,
	const PathFeasibilityChecker & checker,
	geometry_msgs::msg::PoseStamped & recovery_goal_out) const
{
	if (!checker || !has_mission_goal_) {
		return false;
	}

	const double cx = current_pose.pose.position.x;
	const double cy = current_pose.pose.position.y;
	const double gx = mission_goal_.pose.position.x;
	const double gy = mission_goal_.pose.position.y;

	double dir_x = gx - cx;
	double dir_y = gy - cy;
	double norm = std::hypot(dir_x, dir_y);
	if (!std::isfinite(norm) || norm < 1e-3) {
		dir_x = 1.0;
		dir_y = 0.0;
		norm = 1.0;
	}
	dir_x /= norm;
	dir_y /= norm;

	const double lat_x = -dir_y;
	const double lat_y = dir_x;
	const double d = 0.8;
	const std::array<std::pair<double, double>, 4> offsets = {
		std::make_pair(-d, 0.0),
		std::make_pair(-0.7 * d, 0.8 * d),
		std::make_pair(-0.7 * d, -0.8 * d),
		std::make_pair(-1.2 * d, 0.0)};

	for (const auto & off : offsets) {
		geometry_msgs::msg::PoseStamped candidate = current_pose;
		candidate.header.stamp = current_pose.header.stamp;
		candidate.pose.position.x = cx + off.first * dir_x + off.second * lat_x;
		candidate.pose.position.y = cy + off.first * dir_y + off.second * lat_y;
		candidate.pose.position.z = 0.0;
		candidate.pose.orientation.w = 1.0;

		if (checker(current_pose, candidate)) {
			recovery_goal_out = candidate;
			return true;
		}
	}

	return false;
}

}  // namespace minco_planner
