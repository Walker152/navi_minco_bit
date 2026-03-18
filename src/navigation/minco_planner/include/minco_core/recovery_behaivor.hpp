#ifndef MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_
#define MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_

#include <cstdint>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace minco_planner
{

class RecoverServer
{
public:
  enum class RecoveryDecision {
    NONE,
    USE_RECOVERY_GOAL,
    ENTER_EMER_STOP
  };

  enum class RecoveryGoalReachAction {
    NONE,
    RESUME_MISSION,
    FINISH_NO_GOAL
  };

  using PathFeasibilityChecker = std::function<bool(
      const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &)>;

  RecoverServer();
  ~RecoverServer();

  // Configure recovery trigger and timing parameters.
  void configure(int32_t fail_threshold, double cooldown_sec, double recovery_window_sec);

  // Clear all runtime state.
  void reset();

  // Register replan result.
  // Returns true when this failure causes recovery mode to start.
  bool onReplanFailure(double now_s);
  void onReplanSuccess();

  // Mission goal and recovery-goal lifecycle.
  void setMissionGoal(const geometry_msgs::msg::PoseStamped & mission_goal);
  void clearMissionGoal();
  bool isRecoveryGoalActive() const;

  // Unified replan-failure handling with escape-goal generation.
  RecoveryDecision handleReplanFailure(
    double now_s,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PathFeasibilityChecker & checker,
    geometry_msgs::msg::PoseStamped & recovery_goal_out);

  // Process arrival at recovery goal and decide next action.
  RecoveryGoalReachAction onRecoveryGoalReached(
    double current_speed,
    double stop_speed_threshold,
    geometry_msgs::msg::PoseStamped & mission_goal_out);

  // Manual control hooks.
  void startRecovery(double now_s);
  void finishRecovery(bool success, double now_s);

  // Query status.
  bool shouldTryRecovery(double now_s) const;
  bool inRecovery(double now_s) const;
  int32_t consecutiveFailures() const;

private:
  bool isTimeValid(double now_s) const;
  bool tryBuildRecoveryGoal(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PathFeasibilityChecker & checker,
    geometry_msgs::msg::PoseStamped & recovery_goal_out) const;

  int32_t fail_threshold_{3};
  double cooldown_sec_{2.0};
  double recovery_window_sec_{1.5};

  int32_t consecutive_failures_{0};
  double last_failure_time_{-1.0};
  double last_recovery_start_time_{-1.0};
  double last_recovery_end_time_{-1.0};
  bool recovery_active_{false};

  bool has_mission_goal_{false};
  geometry_msgs::msg::PoseStamped mission_goal_;
  bool recovery_goal_active_{false};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_
