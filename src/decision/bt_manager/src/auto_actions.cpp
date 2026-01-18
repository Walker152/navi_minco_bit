#include "bt_manager/auto_actions.hpp"
#include "bt_manager/blackboard.hpp"
#include "nav_zone.hpp"
#include <string>
#include <cmath>
#include <chrono>

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

    // ------------------- ChangePosition -------------------
  ChangePosition::ChangePosition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList ChangePosition::providedPorts()
  {
    return {};
  }
  BT::NodeStatus ChangePosition::tick()
  {   
    auto blackboard = config().blackboard;
    return BT::NodeStatus::SUCCESS;
  }

  //    justprotect
  JustProtect::JustProtect(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList JustProtect::providedPorts()
  {
    return {};
  }
  BT::NodeStatus JustProtect::tick()
  {   
    return BT::NodeStatus::SUCCESS;
  }

  // -------------------- DirectVelocityControl ---------------------------
DirectVelocityControl::DirectVelocityControl(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{ 
  // 创建简单的节点
  node_ = std::make_shared<rclcpp::Node>("direct_control_" + name);
  
  // 创建速度发布器 - 直接发布到/cmd_vel
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  linear_x_ = 0.0;
  angular_z_ = 0.0;
  duration_ = 0.0;
  start_time_ = rclcpp::Time(0, 0);
  last_pub_time_ = rclcpp::Time(0, 0);
}

BT::PortsList DirectVelocityControl::providedPorts()
{
  return {
    BT::InputPort<double>("linear_x", 0.5, "前进速度 m/s"),
    BT::InputPort<double>("angular_z", 0.0, "转向速度 rad/s"), 
    BT::InputPort<double>("duration", 2.0, "持续时间秒")
  };
}

BT::NodeStatus DirectVelocityControl::onStart()
{
  // 1. 获取参数
  auto linear_x = getInput<double>("linear_x");
  auto angular_z = getInput<double>("angular_z"); 
  auto duration = getInput<double>("duration");
  
  if (!linear_x || !duration) {
    RCLCPP_ERROR(node_->get_logger(), "参数缺失: linear_x 或 duration");
    return BT::NodeStatus::FAILURE; // 参数缺失
  }
  
  // 2. 存储参数
  linear_x_ = linear_x.value();
  angular_z_ = angular_z.value_or(0.0);
  duration_ = duration.value();
  
  // 3. 记录开始时间
  start_time_ = node_->now();
  last_pub_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  
  // 4. 发布停止指令，确保从静止开始
  geometry_msgs::msg::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_msg);
  
  // 给一点时间让机器人停止
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DirectVelocityControl::onRunning()
{
  // 计算经过的时间
  auto current_time = node_->now();
  auto elapsed = (current_time - start_time_).seconds();
  
  // 检查是否超时
  if (elapsed >= duration_) {
    // 时间到，发布停止指令
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
    return BT::NodeStatus::SUCCESS;
  }
  
  // 发布速度指令
  geometry_msgs::msg::Twist cmd_vel;
  if ((current_time - last_pub_time_).seconds() >= 0.05) { // 20Hz发布频率
    cmd_vel.linear.x = linear_x_;
    cmd_vel.angular.z = angular_z_;
    cmd_vel_pub_->publish(cmd_vel);
    last_pub_time_ = current_time;
    }

  return BT::NodeStatus::RUNNING;
}

void DirectVelocityControl::onHalted()
{
  // 被中断时立即停止
  geometry_msgs::msg::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(stop_msg);
}

// ------------------- SetStairsPosition -------------------
SetStairsPosition::SetStairsPosition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList SetStairsPosition::providedPorts()
{
  return {}; 
}

BT::NodeStatus SetStairsPosition::tick()
{
 
  auto blackboard = config().blackboard;

  // 创建一个固定目标点（台阶前准备位置）（硬编码）
  geometry_msgs::msg::Point goal_point;
  goal_point.x = 2.5;  
  goal_point.y = 1.2;  
  goal_point.z = 0.0;  

  blackboard->set("nav_goal", goal_point);
  
  return BT::NodeStatus::SUCCESS; // 总是成功
}

}  // namespace Sentry_BT
