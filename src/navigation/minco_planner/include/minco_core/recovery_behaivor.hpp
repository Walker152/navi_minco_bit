#ifndef MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_
#define MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace minco_planner
{

class RecoverServer
{
public:
  struct Config
  {
    int32_t fail_threshold{3};
    double cooldown_sec{2.0};
    double recovery_window_sec{1.5};
    double search_min_dist{0.2};
    double search_max_dist{0.5};
    double search_step{0.05};
    double escape_speed{0.4};
  };

  enum class RecoveryDecision {
    NONE,
    DO_ESCAPE,
    ENTER_EMER_STOP
  };

  using Ptr = std::shared_ptr<RecoverServer>;
  using EsdfQueryFunc = std::function<double(const Eigen::Vector3d &)>;
  RecoverServer();
  ~RecoverServer();

  // Configure recovery trigger and timing parameters.
  void configure(const Config & config);

  // Clear all runtime state.
  void reset();

  // Returns true when this failure causes recovery mode to start.
  bool onReplanFailure(double now_s);
  void onReplanSuccess();

  // Mission goal and recovery-goal lifecycle.
  void setMissionGoal(const geometry_msgs::msg::PoseStamped & mission_goal);
  void clearMissionGoal();

  // Unified replan-failure handling with escape velocity generation.
  RecoveryDecision handleReplanFailure(
    double now_s,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const EsdfQueryFunc & esdf_func,
    Eigen::Vector2d & escape_vel_out);

  // Manual control hooks.
  void startRecovery(double now_s);
  void finishRecovery(bool success, double now_s);

  // Query status.
  bool shouldTryRecovery(double now_s) const;
  bool inRecovery(double now_s) const;
  int32_t consecutiveFailures() const;

private:
  bool isTimeValid(double now_s) const;
  bool calculateEscapeVelocity(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const EsdfQueryFunc & esdf_func,
    Eigen::Vector2d & escape_vel_out) const;

  Config config_{};

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
