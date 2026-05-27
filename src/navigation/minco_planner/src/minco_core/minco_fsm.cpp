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

MincoFsm::MincoFsm(const PlannerPtr & planner, const RecoveryPtr & recovery_server)
: planner_(planner), recovery_server_(recovery_server)
{
}

void MincoFsm::cancelGoal()
{
  has_goal_ = false;
  if (recovery_server_) {
    recovery_server_->clearMissionGoal();
  }
  // Disabled to prevent zero-velocity deadlock
  // changeState("CANCEL_GOAL", State::EMER_STOP);
  changeState("CANCEL_GOAL", State::WAIT_GOAL);
}

// -----------------------------------------------------------------------------
// 2) Core business interface
// -----------------------------------------------------------------------------

void MincoFsm::callMainFsmOnce()
{
  if (!planner_ || !recovery_server_) {
    return;
  }

  // Consume latest goal (createPlan only sets this flag).
  // Disabled to prevent zero-velocity deadlock
  // if (state_ != State::EMER_STOP && state_ != State::RECOVERING) {
  if (state_ != State::RECOVERING) {
    geometry_msgs::msg::PoseStamped new_goal;
    if (planner_->consumePendingGoal(new_goal)) {
      goal_ = new_goal;
      has_goal_ = true;
      recovery_server_->setMissionGoal(new_goal);
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

    auto handle_generate_replan_failure = [this, &current_pose](
                                            const char * escape_reason, const char * emer_reason) {
      Eigen::Vector2d escape_vel;
      const auto decision = recovery_server_->handleReplanFailure(
        planner_->nowSeconds(),
        current_pose,
        [this](const Eigen::Vector3d & p) {
          return planner_->getEsdfDistance(p);
        },
        escape_vel);

      if (decision == RecoverServer::RecoveryDecision::DO_ESCAPE) {
        current_escape_vel_ = escape_vel;
        changeState(escape_reason, State::RECOVERING);
        return;
      }

      if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
        // Disabled to prevent zero-velocity deadlock
        // changeState(emer_reason, State::EMER_STOP);
        changeState(emer_reason, State::GENERATE_TRAJ);
        return;
      }

      // Recovery threshold not reached yet: stay in GENERATE_TRAJ and keep accumulating failures.
      // Disabled to prevent zero-velocity deadlock
      // if (!stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   stop_published_ = true;
      // }
    };

    if (!planner_->PlanGlobalPath(current_pose, goal_)) {
      handle_generate_replan_failure(
        "GLOBAL_SEARCH_FAIL_TRIGGER_RECOVERING", "GLOBAL_SEARCH_FAIL_RECOVERY_FAIL");
      return;
    }
    if (!planner_->ReplanLocal(current_pose)) {
      Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
      // double dist = planner_->getEsdfDistance(cur_p);
      // if (dist < 0.25) {
      handle_generate_replan_failure("GEN_STUCK_TRIGGER_RECOVERING", "GENERATE_RECOVERY_FAIL");
      // return;
      // }
      // recovery_server_->onReplanSuccess();
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
      // if (!goal_stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   goal_stop_published_ = true;
      // }

      if (planner_->getCurrentSpeed().head<2>().norm() < 0.3) {
        has_goal_ = false;
        changeState("GOAL_REACHED", State::WAIT_GOAL);
      }

      return;
    }

    // Disabled to prevent zero-velocity deadlock
    // goal_stop_published_ = false;

    const double now_s = planner_->nowSeconds();

    bool need_replan = planner_->isTrajectoryTimeExpired(now_s) || !planner_->isTrajSafe();

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
      // P0: If the old trajectory still has remaining time, keep following it.
      if (!planner_->isTrajectoryTimeExpired(now_s)) {
        return;
      }

      // 1. 失败诊断：区分是”前方路被挡”还是”自身被卡死”
      Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
      double dist = planner_->getEsdfDistance(cur_p);

      // 2. 诊断为安全 (ESDF >= 0.25m)：纯粹前方路障，立即绕路
      if (dist >= 0.25) {
        // recovery_server_->onReplanSuccess();  // 清空失败计数
        changeState("PATH_BLOCKED_DETOUR", State::GENERATE_TRAJ);
        return;
      }

      // 3. 诊断为危险 (ESDF < 0.25m)：陷入死角，请求推离自救
      Eigen::Vector2d escape_vel;
      const auto decision = recovery_server_->handleReplanFailure(
        planner_->nowSeconds(),
        current_pose,
        [this](const Eigen::Vector3d & p) {
          return planner_->getEsdfDistance(p);
        },
        escape_vel);

      if (decision == RecoverServer::RecoveryDecision::DO_ESCAPE) {
        current_escape_vel_ = escape_vel;
        changeState("STUCK_TRIGGER_RECOVERING", State::RECOVERING);
        return;
      }

      if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
        // Disabled to prevent zero-velocity deadlock
        // changeState("FOLLOW_REPLAN_RECOVERY", State::EMER_STOP);
        changeState("FOLLOW_REPLAN_RECOVERY", State::GENERATE_TRAJ);
        return;
      }

      // 4. 处于 NONE 状态
      // Disabled to prevent zero-velocity deadlock
      // if (!stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   stop_published_ = true;
      // }
      return;
    }

    recovery_server_->onReplanSuccess();

    traveled_dist_ = 0.0;
    return;
  }

  case State::RECOVERING: {
    if (!has_odom) {
      return;
    }

    const double now_s = planner_->nowSeconds();
    Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
    double dist = planner_->getEsdfDistance(cur_p);

    // 条件1: 成功挤出泥坑 (ESDF 距离恢复安全)
    if (dist > 0.40) {
      recovery_server_->finishRecovery(true, now_s);
      changeState("ESCAPE_SUCCESS", State::GENERATE_TRAJ);
      return;
    }

    // 条件2: 挣扎超时保护
    if (!recovery_server_->inRecovery(now_s)) {
      recovery_server_->finishRecovery(false, now_s);
      // Disabled to prevent zero-velocity deadlock
      // changeState("ESCAPE_TIMEOUT", State::EMER_STOP);
      changeState("ESCAPE_TIMEOUT", State::GENERATE_TRAJ);
      return;
    }

    // 条件3: 持续高频下发伪指令覆盖 MPC
    planner_->publishEscapeCommand(current_pose, current_escape_vel_);
    return;
  }

    // Disabled to prevent zero-velocity deadlock
    /*
    case State::EMER_STOP: {
      if (!has_odom) {
        return;
      }

      // 1) First run: publish independent brake trajectory.
      if (!stop_published_) {
        planner_->publishEmergencyStop(current_pose);
        stop_published_ = true;
        emer_stop_start_time_ = planner_->nowSeconds();
      }

      // 2) Timeout protection: avoid deadlock.
      const double now_s = planner_->nowSeconds();
      if (std::isfinite(now_s) && std::isfinite(emer_stop_start_time_) &&
          (now_s - emer_stop_start_time_) > 2.0) {
        // Keep mission goal so FSM can retry planning automatically after emergency stop timeout.
        changeState("EMER_TIMEOUT", has_goal_ ? State::GENERATE_TRAJ : State::WAIT_GOAL);
        return;
      }

      // 3) Still try to find safe path to recover without fully stopping.
      bool recover_possible = false;
      if (planner_->PlanGlobalPath(current_pose, goal_)) {
        if (planner_->ReplanLocal(current_pose)) {
          recover_possible = true;
        }
      }

      if (recover_possible) {
        recovery_server_->finishRecovery(true, now_s);
        changeState("EMER_RECOVER", State::FOLLOW_TRAJ);
        return;
      }

      // 4) Blocking wait until fully stopped.
      const Eigen::Vector3d speed = planner_->getCurrentSpeed();
      if (std::isfinite(speed.head<2>().norm()) && speed.head<2>().norm() > 0.1) {
        return;
      }

      // 5) Recovery: stopped, check safety before leaving EMER_STOP.
      recovery_server_->finishRecovery(false, now_s);
      // Keep mission goal so robot can continue navigating once it is safe/stopped.
      changeState("EMER_SAFE", has_goal_ ? State::GENERATE_TRAJ : State::WAIT_GOAL);
      return;
    }
    */

  default:
    break;
  }
}

// -----------------------------------------------------------------------------
// 3) Helpers
// -----------------------------------------------------------------------------

namespace {
[[maybe_unused]] const char * StateToString(MincoFsm::State s)
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
  case MincoFsm::State::RECOVERING:
    return "RECOVERING";
  // Disabled to prevent zero-velocity deadlock
  // case MincoFsm::State::EMER_STOP:
  //   return "EMER_STOP";
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

  // Leaving recovery-related states: clear debug visualization and reset recovery runtime.
  // Disabled to prevent zero-velocity deadlock
  // if ((state_ == State::RECOVERING || state_ == State::EMER_STOP) && new_state != State::RECOVERING &&
  //     new_state != State::EMER_STOP) {
  if (state_ == State::RECOVERING && new_state != State::RECOVERING) {
    planner_->clearRecoveryDebugVisualization();
    recovery_server_->reset();
  }

  (void)caller;

  // std::cout << "[MincoFSM] [" << (caller ? caller : "?") << "] change state from ["
  //           << StateToString(state_) << "] to [" << StateToString(new_state) << "]" << std::endl;

  last_state_ = state_;
  state_ = new_state;
  // Disabled to prevent zero-velocity deadlock
  // stop_published_ = false;
  // goal_stop_published_ = false;
}

}  // namespace minco_planner
