#ifndef MINCO_PLANNER__MINCO_FSM_HPP_
#define MINCO_PLANNER__MINCO_FSM_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class MincoPlanner;
class RecoverServer;

class MincoFsm
{
public:
  // === Internal Types ===
  enum class State
  {
    INIT,
    WAIT_GOAL,
    GENERATE_TRAJ,
    FOLLOW_TRAJ,
    RECOVERING,
  };

  using PlannerPtr = std::shared_ptr<MincoPlanner>;
  using RecoveryPtr = std::shared_ptr<RecoverServer>;

  // === Constructor & Lifecycle ===
  MincoFsm(const PlannerPtr & planner, const RecoveryPtr & recovery_server);

  // === Core Planning Interfaces ===
  void callMainFsmOnce();
  void cancelGoal();

  // === Utility & Helper Functions ===
  State getState() const { return state_; }

private:
  // === Utility & Helper Functions ===
  // --- State Transition ---
  void changeState(const char * caller, State new_state);

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  PlannerPtr planner_;
  RecoveryPtr recovery_server_;

  // === State Variables & Caches ===
  // --- FSM State ---
  State state_{State::INIT};
  State last_state_{State::INIT};

  // --- Goal Lifecycle ---
  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;

  // --- Motion Tracking Cache ---
  bool has_last_pose_{false};
  double last_pose_x_{0.0};
  double last_pose_y_{0.0};
  double traveled_dist_{0.0};

  // --- Recovery / Emergency Runtime ---
  bool stop_published_{false};
  bool goal_stop_published_{false};
  double emer_stop_start_time_{0.0};
  Eigen::Vector2d current_escape_vel_{0.0, 0.0};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_FSM_HPP_
