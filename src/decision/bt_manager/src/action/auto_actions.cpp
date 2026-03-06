#include "bt_manager/action/auto_actions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <array>
#include <string>
#include <cmath>
#include <chrono>
using namespace color_text;
namespace Sentry_BT
{

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
    std::cout << BLUE << "---------- SetCoordinate ----------" << RESET << std::endl;
    auto goal_index = getInput<int>("goal");

    if(goal_index.value() < 0 || goal_index.value() >= static_cast<int>(nav_points.size()))
    {
      return BT::NodeStatus::FAILURE;
    }

    static const std::array<std::string, 3> goal_names = {"HOME", "BONUS", "OUTPOST"};
    Sentry_BT::Point2D point = nav_points[goal_index.value()];

    auto blackboard = config().blackboard;
    blackboard->set("nav_goal", point);
    std::cout << GREEN << "Set navigation goal to " << goal_names[goal_index.value()] << ": (" << point.x << ", " << point.y
              << ")" << RESET << std::endl;
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
    std::cout << BLUE << "---------- SetTargetCoordinate ----------" << RESET << std::endl;
    auto blackboard = config().blackboard;
    auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    Sentry_BT::Point2D point;
    point.x = target_pose.position.x;
    point.y = target_pose.position.y;

    // 将目标点设置到黑板
    blackboard->set("nav_goal", point);
    std::cout << WHITE << "Set target pose to: (" << point.x << ", " << point.y << ")" << RESET << std::endl;
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

    std::cout << RED << "---------- SelectPatrolPoint ----------" << RESET << std::endl;
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
    auto own_outpost_health = blackboard->get<int>("own_outpost_health");
    std::vector<Sentry_BT::Point2D> patrol_points =
      (own_outpost_health <= 0) ? Sentry_BT::patrol_points_attack : Sentry_BT::patrol_points_normal;

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
    blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);

    std::cout << GREEN << "Selected patrol point " << current_index << ": (" << selected_point.x << ", " << selected_point.y
              << ")" << RESET << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ------------------- Wait -------------------
  Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    , wait_time_(0)
    , start_time_(std::chrono::system_clock::now())
  {
  }

  BT::PortsList Wait::providedPorts()
  {
    return {BT::InputPort<int>("milliseconds")};
  }

  BT::NodeStatus Wait::onStart()
  {
    auto blackboard = config().blackboard;
    std::cout << BLUE << "---------- Wait ----------" << RESET << std::endl;
    auto wait_time = blackboard->get<int>("patrol_wait_time");

    if(!wait_time)
    {
      auto time = getInput<int>("milliseconds");
      wait_time = time.value();
    }

    std::cout << WHITE << "Waiting for " << wait_time << " milliseconds" << RESET << std::endl;
    wait_time_ = wait_time;
    start_time_ = std::chrono::system_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Wait::onRunning()
  {
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
    if(elapsed < wait_time_)
    {
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  void Wait::onHalted()
  {
  }

  // -------------------- DirectVelocityControl ---------------------------
DirectVelocityControl::DirectVelocityControl(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{ 
  linear_y_ = 0.0;
  angular_z_ = 0.0;
  duration_ = 0.0;
  start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

BT::PortsList DirectVelocityControl::providedPorts()
{
  return {
    BT::InputPort<double>("linear_y", 0.5, "前进速度 m/s"),
    BT::InputPort<double>("angular_z", 0.0, "转向速度 rad/s"), 
    BT::InputPort<double>("duration", 2.0, "持续时间秒")
  };
}

BT::NodeStatus DirectVelocityControl::onStart()
{
  auto ros_iface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");

  // 1. 获取参数
  auto linear_y = getInput<double>("linear_y");
  auto angular_z = getInput<double>("angular_z"); 
  auto duration = getInput<double>("duration");
  
  std::cout << MAGENTA << "---------- DirectVelocityControl ----------" << RESET << std::endl;

  if (!linear_y || !duration) {
    std::cerr << "参数缺失: linear_y 或 duration" << std::endl;
    return BT::NodeStatus::FAILURE; // 参数缺失
  }
  
  // 2. 存储参数
  linear_y_ = linear_y.value();
  angular_z_ = angular_z.value_or(0.0);
  duration_ = duration.value();
  
  // 3. 记录开始时间
  start_time_ = ros_iface->now();
  last_pub_time_ = rclcpp::Time(0, 0, ros_iface->get_clock()->get_clock_type());
  
  // 4. 发布停止指令，确保从静止开始
  ros_iface->publishCmdVel(0.0, 0.0);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DirectVelocityControl::onRunning()
{
  auto ros_iface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  // 计算经过的时间
  auto current_time = ros_iface->now();
  auto elapsed = (current_time - start_time_).seconds();

  if(elapsed < 0.1)
  {
    ros_iface->publishCmdVel(0.0, 0.0);
    return BT::NodeStatus::RUNNING;
  }
  
  // 检查是否超时
  if (elapsed >= duration_) {
    // 时间到，发布停止指令
    ros_iface->publishCmdVel(0.0, 0.0);
    return BT::NodeStatus::SUCCESS;
  }
  
  // 发布速度指令
  if ((current_time - last_pub_time_).seconds() >= 0.05) { // 20Hz发布频率
    ros_iface->publishCmdVel(linear_y_, angular_z_);
    last_pub_time_ = current_time;
    }

  return BT::NodeStatus::RUNNING;
}

void DirectVelocityControl::onHalted()
{
  auto ros_iface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  // 被中断时立即停止
  ros_iface->publishCmdVel(0.0, 0.0);
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
  std::cout << MAGENTA << "---------- SetStairsPosition ----------" << RESET << std::endl;
  
  // 创建 Sentry_BT::Point2D 类型的固定目标点
  Sentry_BT::Point2D goal_point;
  goal_point.x = 9.5;  
  goal_point.y = 1.5;  

  blackboard->set("nav_goal", goal_point);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
