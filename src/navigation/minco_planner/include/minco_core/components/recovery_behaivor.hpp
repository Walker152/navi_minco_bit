#ifndef MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_
#define MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class RecoverServer
{
public:
  // === Internal Types ===
  // --- Static Configuration ---
  struct Config
  {
    int32_t fail_threshold{3};
    double cooldown_sec{2.0};
    double recovery_window_sec{1.5};
    double escape_speed{0.4};
  };

  // --- Recovery Decision ---
  enum class RecoveryDecision
  {
    NONE,
    DO_ESCAPE,
    ENTER_EMER_STOP
  };

  // --- Public Type Aliases ---
  using Ptr = std::shared_ptr<RecoverServer>;
  using EsdfQueryFunc = std::function<double(const Eigen::Vector3d &)>;

  // === Constructor & Lifecycle ===
  RecoverServer();
  ~RecoverServer();

  // === Core Planning Interfaces ===
  // --- Configuration and Reset ---
  // Configure recovery trigger and timing parameters.
  void configure(const Config & config);

  // Clear all runtime state.
  void reset();

  // --- Replan Failure/Success Hooks ---
  bool onReplanFailure(double now_s);
  void onReplanSuccess();

  // --- Goal Lifecycle Management ---
  void setMissionGoal(const geometry_msgs::msg::PoseStamped & mission_goal);
  void clearMissionGoal();

  // --- Recovery Decision Pipeline ---
  RecoveryDecision handleReplanFailure(double now_s,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const EsdfQueryFunc & esdf_func,
    Eigen::Vector2d & escape_vel_out);

  // --- Manual Recovery Control ---
  void startRecovery(double now_s);
  void finishRecovery(bool success, double now_s);

  // === Utility & Helper Functions ===
  // --- State Queries ---
  // Query status.
  bool shouldTryRecovery(double now_s) const;
  bool inRecovery(double now_s) const;
  int32_t consecutiveFailures() const;

private:
  // === Utility & Helper Functions ===
  // --- Internal Validation and Computation ---
  bool isTimeValid(double now_s) const;
  bool calculateEscapeVelocity(const geometry_msgs::msg::PoseStamped & current_pose,
    const EsdfQueryFunc & esdf_func,
    Eigen::Vector2d & escape_vel_out) const;

  // === Configurations & Parameters ===
  Config config_{};

  // === State Variables & Caches ===
  // --- Failure and Recovery Runtime State ---
  int32_t consecutive_failures_{0};
  double last_failure_time_{-1.0};
  double last_recovery_start_time_{-1.0};
  double last_recovery_end_time_{-1.0};
  bool recovery_active_{false};

  // --- Goal and Recovery Goal Cache ---
  bool has_mission_goal_{false};
  geometry_msgs::msg::PoseStamped mission_goal_;
  bool recovery_goal_active_{false};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__RECOVERY_BEHAIVOR_HPP_
