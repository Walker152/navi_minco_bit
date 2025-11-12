#include "bt_manager/auto_actions.hpp"
#include "bt_manager/blackboard.hpp"
#include "nav_zone.hpp"
#include <string>

namespace Sentry_BT
{

  // ------------------- PublishNavigationGoal -------------------
  PublishNavigationGoal::PublishNavigationGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList PublishNavigationGoal::providedPorts()
  {
    return {};
  }

  BT::NodeStatus PublishNavigationGoal::tick()
  {
    auto blackboard = config().blackboard;
    auto nav_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal");
    // if (!nav_goal)
    // {
    //   RCLCPP_ERROR(rclcpp::get_logger("PublishNavigationGoal"), "missing nav_goal on blackboard");
    //   return BT::NodeStatus::FAILURE;
    // }

    // 发布目标点
    auto ros_interface_ptr = blackboard->get<std::shared_ptr<ros_interface>>("ros_interface");
    if(!ros_interface_ptr)
    {
      throw BT::RuntimeError("missing ros_interface on blackboard");
    }
    bool success = ros_interface_ptr->publishNavigationGoal(nav_goal);
    if(success)
    {
      std::cout << "Published navigation goal: (" << nav_goal.x << ", " << nav_goal.y << ")" << std::endl;
      std::cout << "-----------------------------------" << std::endl;
    }

    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // ------------------- SetCoordinate -------------------
  SetCoordinate::SetCoordinate(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList SetCoordinate::providedPorts()
  {
    return {
        BT::InputPort<int>("goal")  // 全局导航点索引
    };
  }

  BT::NodeStatus SetCoordinate::tick()
  {
    std::cout << "---------- SetCoordinate ----------" << std::endl;
    auto goal_index = getInput<int>("goal");

    if(goal_index.value() < 0 || goal_index.value() >= static_cast<int>(nav_points.size()))
    {
      // throw BT::RuntimeError("invalid goal index: ", (char)goal_index.value());
    }

    std::vector<std::string> goal_names = {"HOME", "BONUS", "OUTPOST"};
    Sentry_BT::Point2D point = nav_points[goal_index.value()];

    auto blackboard = config().blackboard;
    blackboard->set("nav_goal", point);
    std::cout << "Set navigation goal to " << goal_names[goal_index.value()] << ": (" << point.x << ", " << point.y
              << ")" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ------------------- SetTargetCoordinate -------------------
  SetTargetCoordinate::SetTargetCoordinate(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList SetTargetCoordinate::providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::Pose>("target_coordinate")};
  }

  BT::NodeStatus SetTargetCoordinate::tick()
  {
    std::cout << "---------- SetTargetCoordinate ----------" << std::endl;
    auto blackboard = config().blackboard;
    auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");

    Sentry_BT::Point2D point;
    point.x = target_pose.position.x;
    point.y = target_pose.position.y;

    // 将目标点设置到黑板
    blackboard->set("nav_goal", point);
    std::cout << "Set target pose to: (" << point.x << ", " << point.y << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ------------------- SelectPatrolPoint -------------------
  SelectPatrolPoint::SelectPatrolPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList SelectPatrolPoint::providedPorts()
  {
    return {};
  }

  BT::NodeStatus SelectPatrolPoint::tick()
  {
    auto blackboard = config().blackboard;

    std::cout << "---------- SelectPatrolPoint ----------" << std::endl;
    // 获取当前巡逻索引
    int current_index = 0;
    if(auto index = blackboard->get<int>("patrol_index"))
    {
      current_index = index;
    }
    else
    {
      blackboard->set("patrol_index", current_index);
    }

    // 获取前哨站状态决定使用哪种巡逻路线
    auto own_outpost_destroyed = blackboard->get<bool>("own_outpost_destroyed");
    std::vector<Sentry_BT::Point2D> patrol_points =
        own_outpost_destroyed ? Sentry_BT::patrol_points_attack : Sentry_BT::patrol_points_normal;

    // 检查索引有效性
    if(current_index >= static_cast<int>(patrol_points.size()))
    {
      current_index = 0;
      blackboard->set("patrol_index", current_index);
    }

    Sentry_BT::Point2D selected_point = patrol_points[current_index];

    int next_index = (current_index + 1) % patrol_points.size();
    blackboard->set("patrol_index", next_index);
    blackboard->set("nav_goal", selected_point);
    blackboard->set("patrol_wait_time", patrol_points_milliseconds[current_index]);

    std::cout << "Selected patrol point " << current_index << ": (" << selected_point.x << ", " << selected_point.y
              << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ------------------- WaitUntilStopped -------------------
  WaitUntilStopped::WaitUntilStopped(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
  }

  BT::PortsList WaitUntilStopped::providedPorts()
  {
    return {};
  }

  BT::NodeStatus WaitUntilStopped::onStart()
  {
    auto blackboard = config().blackboard;
    // 检查导航状态
    auto nav_status = blackboard->get<int>("nav_status");

    // 如果导航已经停止，直接返回成功
    if(nav_status == Sentry_BT::NavStatus::IDLE || nav_status == Sentry_BT::NavStatus::FAILURE)
    {
      return BT::NodeStatus::SUCCESS;
    }

    // 否则继续等待
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus WaitUntilStopped::onRunning()
  {
    auto blackboard = config().blackboard;
    // 检查导航状态
    auto nav_status  = blackboard->get<int>("nav_status");

    std::cout << "Current navigation status: " << current_nav_status[nav_status] << std::endl;
    // 如果导航已经停止，返回成功
    if(nav_status == Sentry_BT::NavStatus::IDLE || nav_status == Sentry_BT::NavStatus::FAILURE)
    {
      return BT::NodeStatus::SUCCESS;
    }

    // 否则继续等待
    return BT::NodeStatus::RUNNING;
  }

  void WaitUntilStopped::onHalted()
  {
    // 无需特殊处理
  }

  // ------------------- Wait -------------------
  Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList Wait::providedPorts()
  {
    return {BT::InputPort<int>("milliseconds")};
  }

  BT::NodeStatus Wait::tick()
  {
    auto blackboard = config().blackboard;
    std::cout << "---------- Wait ----------" << std::endl;
    auto wait_time = blackboard->get<int>("patrol_wait_time");

    if(!wait_time)
    {
      auto time = getInput<int>("milliseconds");
      wait_time = time.value();
    }

    std::cout << "Waiting for " << wait_time << " milliseconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
    return BT::NodeStatus::SUCCESS;
  }
}  // namespace Sentry_BT
