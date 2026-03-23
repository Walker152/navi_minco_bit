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
  recovery_server_.configure(3, 2.0, 3.0);
}

void MincoFsm::cancelGoal()
{
  has_goal_ = false;
  recovery_server_.clearMissionGoal();
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
      recovery_server_.setMissionGoal(new_goal);
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
        if (recovery_server_.isRecoveryGoalActive()) {
          geometry_msgs::msg::PoseStamped mission_goal;
          const auto action = recovery_server_.onRecoveryGoalReached(
            planner_->getCurrentSpeed().head<2>().norm(), 0.1, mission_goal);
          if (action == RecoverServer::RecoveryGoalReachAction::RESUME_MISSION) {
            goal_ = mission_goal;
            has_goal_ = true;
            changeState("RECOVERY_GOAL_DONE", State::GENERATE_TRAJ);
          } else if (action == RecoverServer::RecoveryGoalReachAction::FINISH_NO_GOAL) {
            has_goal_ = false;
            changeState("RECOVERY_GOAL_DONE_NO_MISSION", State::WAIT_GOAL);
          }
          return;
        }

        if (!goal_stop_published_) {
          planner_->publishEmergencyStop(current_pose);
          goal_stop_published_ = true;
        }

        if (planner_->getCurrentSpeed().head<2>().norm() < 0.1) {
          has_goal_ = false;
          changeState("GOAL_REACHED", State::WAIT_GOAL);
        }

        return;
      }

      goal_stop_published_ = false;

      const double now_s = planner_->nowSeconds();

      bool need_replan =
        planner_->isTrajectoryTimeExpired(now_s) ||
        !planner_->isTrajSafe();

      // 强制高频重规划：1Hz刷新轨迹，避免轨迹“卡死”不更新。
      if (has_odom) {
        static double last_replan_time = 0.0;
        const double current_time = planner_->nowSeconds();
        if (current_time - last_replan_time > 1.0) {
          need_replan = true;
          last_replan_time = current_time;
        }
      }

      if (!need_replan) {
        return;
      }

      if (!planner_->ReplanLocal(current_pose)) {
        geometry_msgs::msg::PoseStamped recovery_goal;
        const auto decision = recovery_server_.handleReplanFailure(
          planner_->nowSeconds(),
          current_pose,
          [this](const geometry_msgs::msg::PoseStamped & start,
                 const geometry_msgs::msg::PoseStamped & goal) {
            return planner_ && planner_->PlanGlobalPath(start, goal);
          },
          recovery_goal);

        if (decision == RecoverServer::RecoveryDecision::USE_RECOVERY_GOAL) {
          goal_ = recovery_goal;
          has_goal_ = true;
          changeState("FOLLOW_REPLAN_RECOVERY_GOAL", State::GENERATE_TRAJ);
          return;
        }

        if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
          changeState("FOLLOW_REPLAN_RECOVERY", State::EMER_STOP);
          return;
        }

        changeState("FOLLOW_REPLAN", State::GENERATE_TRAJ);
        return;
      }

      recovery_server_.onReplanSuccess();

      traveled_dist_ = 0.0;
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
        has_goal_ = false;
        recovery_server_.clearMissionGoal();
        changeState("EMER_TIMEOUT", State::WAIT_GOAL);
        return;
      }

      // 3) Still try to find safe path to recover without fully stopping.
      if (planner_->ReplanLocal(current_pose)) {
        recovery_server_.finishRecovery(true, now_s);
         changeState("EMER_RECOVER", State::FOLLOW_TRAJ);
         return;
      }

      // 4) Blocking wait until fully stopped.
      const Eigen::Vector3d speed = planner_->getCurrentSpeed();
      if (std::isfinite(speed.head<2>().norm()) && speed.head<2>().norm() > 0.1) {
        return;
      }

      // 5) Recovery: stopped, check safety before leaving EMER_STOP.
      recovery_server_.finishRecovery(false, now_s);
      has_goal_ = false;
      recovery_server_.clearMissionGoal();
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
  goal_stop_published_ = false;
}

}  // namespace minco_planner
