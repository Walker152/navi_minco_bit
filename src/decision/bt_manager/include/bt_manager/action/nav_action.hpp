#pragma once

#include <iostream>
#include <string>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "bt_manager/utils/log.hpp"

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

  void on_tick() override
  {
    auto blackboard = config().blackboard;
    Sentry_BT::Point2D nav_goal;
    if(!blackboard->get<Sentry_BT::Point2D>("nav_goal", nav_goal) && !getInput("nav_goal", nav_goal))
    {
      throw BT::RuntimeError("missing required input [nav_goal]");
    }

    goal_.pose.header.frame_id = "map";
    goal_.pose.pose.position.x = nav_goal.x;
    goal_.pose.pose.position.y = nav_goal.y;
    goal_.pose.pose.orientation.w = 1.0;

    static int tick_count = 0;
    ++tick_count;
    if(tick_count == 1 || tick_count % 20 == 0)
    {
      std::cout << GREEN << "[NavigateToPoseAction] tick=" << tick_count << ", goal=(" << nav_goal.x << ", " << nav_goal.y
                << ")" << RESET << std::endl;
    }
  }
};

using NavigateToPoseAction = MapsToPoseAction;

}  // namespace Sentry_BT
