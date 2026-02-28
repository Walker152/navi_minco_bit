#include "minco_core/minco_fsm.hpp"

// C++ standard library
#include <cmath>
#include <iostream>

// Project
#include "minco_core/minco_planner.hpp"

namespace minco_planner {

// -----------------------------------------------------------------------------
// 1) Construction / Destruction
// -----------------------------------------------------------------------------

MincoFsm::MincoFsm(const PlannerPtr & planner)
: planner_(planner)
{
}

void MincoFsm::cancelGoal()
{
  std::cout << "\033[33m[MincoFSM] Goal Cancelled! Forcing Stop.\033[0m" << std::endl;
  has_goal_ = false;
  changeState("CANCEL_GOAL", State::EMER_STOP);
}

// -----------------------------------------------------------------------------
// 2) Core business interface
// -----------------------------------------------------------------------------

void MincoFsm::callMainFsmOnce()
{
  if (!planner_) {
    return;
  }

  // Consume latest goal (createPlan only sets this flag).
  if (state_ != State::EMER_STOP) {
    geometry_msgs::msg::PoseStamped new_goal;
    if (planner_->consumePendingGoal(new_goal)) {
      goal_ = new_goal;
      has_goal_ = true;
      has_last_pose_ = false;
      changeState("NewGoal", State::GENERATE_TRAJ);
    }
  }

  // Get current robot pose.
  geometry_msgs::msg::PoseStamped current_pose;
  const bool has_odom = planner_->getRobotPose(current_pose);

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

      // 容差限停检测：到达终点且速度足够低
      if (planner_->checkGoalReached(current_pose)) {
        if (planner_->getCurrentSpeed() < 0.2) {
          std::cout << "\033[32m[MincoFSM] Destination Arrived! Stopping replan.\033[0m"
                    << std::endl;
          has_goal_ = false;
          changeState("GOAL_REACHED", State::WAIT_GOAL);
          return;
        }
      }

      const double now_s = planner_->nowSeconds();

      bool need_replan =
        planner_->isTrajectoryTimeExpired(now_s) ||
        !planner_->isTrajSafe();

      // 震动过滤：使用相对于上次规划起点的绝对位移，而非里程累加。
      if (!has_last_pose_) {
        has_last_pose_ = true;
        last_pose_x_ = current_pose.pose.position.x;
        last_pose_y_ = current_pose.pose.position.y;
      } else {
        const double dist_from_last_plan = std::hypot(
          current_pose.pose.position.x - last_pose_x_,
          current_pose.pose.position.y - last_pose_y_);
        if (std::isfinite(dist_from_last_plan) && dist_from_last_plan >= 0.5) {
          need_replan = true;
          has_last_pose_ = false;
        }
      }

      // 如果剩余轨迹时间不足 0.2 秒，立即触发重规划防止断供顿挫
      double remain_time = planner_->getTrajectoryRemainTime();
      if (remain_time < 0.2) {
        need_replan = true;
        has_last_pose_ = false;
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

      // 1) First run: publish independent brake trajectory.
      if (!stop_published_) {
        planner_->publishEmergencyStop(current_pose);
        stop_published_ = true;
        emer_stop_start_time_ = planner_->nowSeconds();
        has_goal_ = false;  // After emergency stop, wait for next goal.
      }

      // 2) Timeout protection: avoid deadlock.
      const double now_s = planner_->nowSeconds();
      if (std::isfinite(now_s) && std::isfinite(emer_stop_start_time_) &&
          (now_s - emer_stop_start_time_) > 5.0) {
        std::cout << "[MincoFSM] EMER_STOP timeout (>5s). Forcing reset to WAIT_GOAL." << std::endl;
        has_goal_ = false;
        changeState("EMER_TIMEOUT", State::WAIT_GOAL);
        return;
      }

      // 3) Blocking wait until fully stopped.
      const double speed = planner_->getCurrentSpeed();
      if (std::isfinite(speed) && speed > 0.1) {
        return;
      }

      // 4) Recovery: stopped, check safety before leaving EMER_STOP.
      std::cout << "[MincoFSM] Robot stopped and safe." << std::endl;
      has_goal_ = false;
      changeState("EMER_SAFE", State::WAIT_GOAL);
      return;
    }

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// 3) Helpers
// -----------------------------------------------------------------------------

namespace {
[[maybe_unused]]
const char * StateToString(MincoFsm::State s)
{
  switch (s) {
    case MincoFsm::State::INIT:
      return "INIT";
    case MincoFsm::State::WAIT_GOAL:
      return "WAIT_GOAL";
    case MincoFsm::State::GENERATE_TRAJ:
      return "GENERATE_TRAJ";
    case MincoFsm::State::FOLLOW_TRAJ:
      return "FOLLOW_TRAJ";
    case MincoFsm::State::EMER_STOP:
      return "EMER_STOP";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

void MincoFsm::changeState(const char * caller, State new_state)
{
  if (state_ == new_state) {
    return;
  }

  (void)caller;
  
  // std::cout << "[MincoFSM] [" << (caller ? caller : "?") << "] change state from ["
  //           << StateToString(state_) << "] to [" << StateToString(new_state) << "]" << std::endl;

  last_state_ = state_;
  state_ = new_state;
  stop_published_ = false;
}

}  // namespace minco_planner
