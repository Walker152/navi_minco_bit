#include "minco_core/minco_fsm.hpp"

#include "minco_core/minco_planner.hpp"

#include <cmath>
#include <iostream>

namespace minco_planner
{

namespace
{
const char * stateToString(MincoFsm::State s)
{
  switch (s) {
    case MincoFsm::State::INIT: return "INIT";
    case MincoFsm::State::WAIT_GOAL: return "WAIT_GOAL";
    case MincoFsm::State::GENERATE_TRAJ: return "GENERATE_TRAJ";
    case MincoFsm::State::FOLLOW_TRAJ: return "FOLLOW_TRAJ";
    case MincoFsm::State::EMER_STOP: return "EMER_STOP";
    default: return "UNKNOWN";
  }
}
}  // namespace

MincoFsm::MincoFsm(const PlannerPtr & planner)
: planner_(planner)
{
}

void MincoFsm::changeState(const char * caller, State new_state)
{
  if (state_ == new_state) {
    return;
  }

  std::cout << "[MincoFSM] [" << (caller ? caller : "?") << "] change state from ["
            << stateToString(state_) << "] to [" << stateToString(new_state) << "]" << std::endl;

  last_state_ = state_;
  state_ = new_state;
  stop_published_ = false;
}

void MincoFsm::callMainFsmOnce()
{
  if (!planner_) {
    return;
  }

  // Consume latest goal (createPlan only sets this flag).
  geometry_msgs::msg::PoseStamped new_goal;
  if (planner_->consumePendingGoal(new_goal)) {
    goal_ = new_goal;
    has_goal_ = true;
    changeState("NewGoal", State::GENERATE_TRAJ);
  }

  // Get current robot pose.
  geometry_msgs::msg::PoseStamped current_pose;
  const bool has_odom = planner_->getRobotPose(current_pose);

  // Track traveled distance since last replan (for FOLLOW_TRAJ trigger).
  if (has_odom && state_ == State::FOLLOW_TRAJ) {
    if (!has_last_pose_) {
      has_last_pose_ = true;
      last_pose_x_ = current_pose.pose.position.x;
      last_pose_y_ = current_pose.pose.position.y;
    } else {
      const double dx = current_pose.pose.position.x - last_pose_x_;
      const double dy = current_pose.pose.position.y - last_pose_y_;
      const double ds = std::hypot(dx, dy);
      if (std::isfinite(ds)) {
        traveled_dist_ += ds;
      }
      last_pose_x_ = current_pose.pose.position.x;
      last_pose_y_ = current_pose.pose.position.y;
    }
  }

  switch (state_) {
    case State::INIT: {
      if (!has_odom) {
        return;
      }
      changeState("INIT", State::WAIT_GOAL);
      break;
    }

    case State::WAIT_GOAL: {
      if (!has_goal_) {
        return;
      }
      changeState("WAIT_GOAL", State::GENERATE_TRAJ);
      break;
    }

    case State::GENERATE_TRAJ: {
      if (!has_goal_) {
        changeState("GENERATE_TRAJ", State::WAIT_GOAL);
        return;
      }
      if (!has_odom) {
        changeState("GENERATE_TRAJ", State::INIT);
        return;
      }

      if (!planner_->PlanGlobalPath(current_pose, goal_)) {
        changeState("PlanGlobalPath", State::EMER_STOP);
        return;
      }
      if (!planner_->ReplanLocal(current_pose)) {
        changeState("ReplanLocal", State::EMER_STOP);
        return;
      }

      traveled_dist_ = 0.0;
      has_last_pose_ = false;
      changeState("GENERATE_TRAJ", State::FOLLOW_TRAJ);
      break;
    }

    case State::FOLLOW_TRAJ: {
      if (!has_goal_) {
        changeState("FOLLOW_TRAJ", State::WAIT_GOAL);
        return;
      }
      if (!has_odom) {
        changeState("FOLLOW_TRAJ", State::INIT);
        return;
      }

      bool need_replan = false;
      const double now_s = planner_->nowSeconds();

      // Trigger 1: trajectory time exhausted.
      if (planner_->isTrajectoryTimeExpired(now_s)) {
        need_replan = true;
      }

      // Trigger 2: traveled distance over 0.8 * lookahead_dist.
      if (traveled_dist_ > 0.8 * planner_->getLookaheadDist()) {
        need_replan = true;
      }

      // Trigger 3: async safety alert.
      if (!planner_->isTrajSafe()) {
        need_replan = true;
      }

      if (!need_replan) {
        return;
      }

      if (!planner_->ReplanLocal(current_pose)) {
        changeState("FOLLOW_REPLAN", State::EMER_STOP);
        return;
      }

      traveled_dist_ = 0.0;
      has_last_pose_ = false;
      return;
    }

    case State::EMER_STOP: {
      if (!has_odom) {
        return;
      }
      if (!stop_published_) {
        planner_->publishEmergencyStop(current_pose);
        stop_published_ = true;
      }
      // After emergency stop, wait for next goal.
      has_goal_ = false;
      changeState("EMER_STOP", State::WAIT_GOAL);
      break;
    }

    default:
      break;
  }
}

}  // namespace minco_planner
