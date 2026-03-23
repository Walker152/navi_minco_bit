#pragma once

#include <iostream>
#include <string>
#include <cmath>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"

using namespace color_text;
namespace Sentry_BT
{
class MapsToPoseAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MapsToPoseAction(const std::string& xml_tag_name,
                   const std::string& action_name,
                   const BT::NodeConfiguration& conf)
    : nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<Sentry_BT::Point2D>("nav_goal")});
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
    if(!has_last_goal_)
    {
      return true;
    }
    const double dx = nav_goal.x - last_goal_.x;
    const double dy = nav_goal.y - last_goal_.y;
    return std::hypot(dx, dy) > 0.01;  // 1cm threshold for action-goal update
  }

  void on_tick() override
  {
    auto blackboard = config().blackboard;
    Sentry_BT::Point2D nav_goal;
    if(!readGoalFromBlackboard(nav_goal))
    {
      throw BT::RuntimeError("missing required input [nav_goal]");
    }

    setActionGoal(nav_goal);
    last_goal_ = nav_goal;
    has_last_goal_ = true;

    int current_mode = -1;
    blackboard->get<int>("current_mode", current_mode);
    std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] send initial goal=(" << nav_goal.x << ", "
          << nav_goal.y << "), mode=" << current_mode << RESET << std::endl;
  }

  void on_wait_for_result(std::shared_ptr<const typename nav2_msgs::action::NavigateToPose::Feedback>) override
  {
    Sentry_BT::Point2D nav_goal;
    if(!readGoalFromBlackboard(nav_goal))
    {
      return;
    }

    if(isGoalChanged(nav_goal))
    {
      auto blackboard = config().blackboard;
      setActionGoal(nav_goal);
      goal_updated_ = true;
      last_goal_ = nav_goal;
      has_last_goal_ = true;

      int current_mode = -1;
      blackboard->get<int>("current_mode", current_mode);
      std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] preempt goal=(" << nav_goal.x << ", "
                << nav_goal.y << "), mode=" << current_mode << RESET << std::endl;
    }
  }

private:
  Sentry_BT::Point2D last_goal_{};
  bool has_last_goal_ = false;
};

using NavigateToPoseAction = MapsToPoseAction;

}  // namespace Sentry_BT
