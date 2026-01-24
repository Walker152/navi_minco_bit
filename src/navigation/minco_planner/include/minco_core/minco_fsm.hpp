#ifndef MINCO_PLANNER__MINCO_FSM_HPP_
#define MINCO_PLANNER__MINCO_FSM_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace minco_planner
{

class MincoPlanner;

class MincoFsm
{
public:
  enum class State {
    INIT,
    WAIT_GOAL,
    GENERATE_TRAJ,
    FOLLOW_TRAJ,
    EMER_STOP
  };

  using PlannerPtr = std::shared_ptr<MincoPlanner>;

  explicit MincoFsm(const PlannerPtr & planner);

  void callMainFsmOnce();

  State getState() const {return state_;}

private:
  void changeState(const char * caller, State new_state);

  PlannerPtr planner_;
  State state_{State::INIT};
  State last_state_{State::INIT};

  bool has_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;

  bool has_last_pose_{false};
  double last_pose_x_{0.0};
  double last_pose_y_{0.0};
  double traveled_dist_{0.0};

  bool stop_published_{false};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_FSM_HPP_
