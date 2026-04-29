#pragma once

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace color_text;
namespace Sentry_BT {
class MapsToPoseAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MapsToPoseAction(
    const std::string & xml_tag_name, const std::string & action_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<Sentry_BT::PatrolPoint>("nav_goal")});
  }

  bool readGoalFromBlackboard(Sentry_BT::Point2D & nav_goal)
  {
    auto blackboard = config().blackboard;
    return blackboard->get<Sentry_BT::Point2D>("nav_goal", nav_goal) || getInput("nav_goal", nav_goal);
  }

  void setActionGoal(const Sentry_BT::Point2D & nav_goal)
  {
    goal_.pose.header.frame_id = "map";
    goal_.pose.pose.position.x = nav_goal.x;
    goal_.pose.pose.position.y = nav_goal.y;
    goal_.pose.pose.orientation.w = 1.0;
  }

  bool isGoalChanged(const Sentry_BT::Point2D & nav_goal) const
  {
    if (!has_last_goal_) {
      return true;
    }
    const double dx = nav_goal.x - last_goal_.x;
    const double dy = nav_goal.y - last_goal_.y;
    return std::hypot(dx, dy) > 0.01;  // 1cm threshold for action-goal update
  }

  bool selectNextPatrolGoalFromBlackboard(Sentry_BT::Point2D & next_goal)
  {
    auto blackboard = config().blackboard;

    int patrol_index = 0;
    if (!blackboard->get<int>("patrol_index", patrol_index)) {
      blackboard->set("patrol_index", patrol_index);
    }

    const auto & patrol_points = Sentry_BT::patrol_points_normal;
    if (patrol_points.empty()) {
      return false;
    }

    if (patrol_index < 0 || patrol_index >= static_cast<int>(patrol_points.size())) {
      patrol_index = 0;
    }

    next_goal = patrol_points[patrol_index].position;
    const int next_index = (patrol_index + 1) % static_cast<int>(patrol_points.size());

    blackboard->set("patrol_index", next_index);
    blackboard->set("nav_goal", next_goal);
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
    return true;
  }

  bool isPatrolNavigationContext() const
  {
    int current_mode = -1;
    config().blackboard->get<int>("current_mode", current_mode);
    return current_mode == Sentry_BT::NavMode::PATROL;
  }

  void on_tick() override
  {
    auto blackboard = config().blackboard;
    Sentry_BT::Point2D nav_goal;
    if (!readGoalFromBlackboard(nav_goal)) {
      throw BT::RuntimeError("missing required input [nav_goal]");
    }

    setActionGoal(nav_goal);

    if (isGoalChanged(nav_goal)) {
      last_goal_ = nav_goal;
      has_last_goal_ = true;
      nav_start_time_ = std::chrono::steady_clock::now();

      std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] send initial goal=(" << nav_goal.x
                << ", " << nav_goal.y << ")" << RESET << std::endl;
    }
  }

  void on_wait_for_result(
    std::shared_ptr<const typename nav2_msgs::action::NavigateToPose::Feedback>) override
  {
    auto blackboard = config().blackboard;

    Sentry_BT::Point2D nav_goal;
    if (!readGoalFromBlackboard(nav_goal)) {
      return;
    }

    if (isPatrolNavigationContext()) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed_sec = std::chrono::duration<double>(now - nav_start_time_).count();
      if (elapsed_sec > 15.0) {
        Sentry_BT::Point2D timeout_goal;
        if (selectNextPatrolGoalFromBlackboard(timeout_goal)) {
          setActionGoal(timeout_goal);
          goal_updated_ = true;
          last_goal_ = timeout_goal;
          has_last_goal_ = true;
          nav_start_time_ = now;

          std::cout << YELLOW << "[NavigateToPoseAction:" << name()
                    << "] patrol timeout(>15s), force next patrol goal=(" << timeout_goal.x << ", "
                    << timeout_goal.y << ")" << RESET << std::endl;
          return;
        }
      }
    }

    if (isGoalChanged(nav_goal)) {
      setActionGoal(nav_goal);
      goal_updated_ = true;
      last_goal_ = nav_goal;
      has_last_goal_ = true;
      nav_start_time_ = std::chrono::steady_clock::now();

      std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] preempt goal=(" << nav_goal.x << ", "
                << nav_goal.y << ")" << RESET << std::endl;
    }
  }

private:
  Sentry_BT::Point2D last_goal_{};
  bool has_last_goal_ = false;
  std::chrono::time_point<std::chrono::steady_clock> nav_start_time_ = std::chrono::steady_clock::now();
};

using NavigateToPoseAction = MapsToPoseAction;

}  // namespace Sentry_BT
