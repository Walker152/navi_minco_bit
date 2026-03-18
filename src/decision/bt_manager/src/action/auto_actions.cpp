#include "bt_manager/action/auto_actions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <array>
#include <string>
#include <cmath>
#include <chrono>
#include <limits>
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
    auto goal_index = getInput<int>("goal");

    if(goal_index.value() < 0 || goal_index.value() >= static_cast<int>(nav_points.size()))
    {
      return BT::NodeStatus::FAILURE;
    }

    static const std::array<std::string, 3> goal_names = {"HOME", "BONUS", "OUTPOST"};
    Sentry_BT::Point2D point = nav_points[goal_index.value()];

    auto blackboard = config().blackboard;
    blackboard->set("nav_goal", point);
    static int last_goal_index = -1;
    if(goal_index.value() != last_goal_index)
    {
      std::cout << GREEN << "Set navigation goal to " << goal_names[goal_index.value()] << ": (" << point.x << ", "
                << point.y << ")" << RESET << std::endl;
      last_goal_index = goal_index.value();
    }
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
    auto blackboard = config().blackboard;
    auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    Sentry_BT::Point2D point;//最终目标点
    //获取当前位置
    std::shared_ptr<ros_interface> ros_iface;
    geometry_msgs::msg::Pose current_pose;
    ros_iface = blackboard->get<std::shared_ptr<ros_interface>>("ros_interface");
    if(!ros_iface){
      return BT::NodeStatus::FAILURE;
    }
    current_pose = ros_iface->getCurrentPose();

    // 提取坐标
    double current_x = current_pose.position.x;
    double current_y = current_pose.position.y;
    double target_x = target_pose.position.x;
    double target_y = target_pose.position.y;
    //适当漂移，留出攻击距离
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = std::sqrt(dx*dx + dy*dy);

    const double ATTACK_DISTANCE = 0.3; // 攻击距离

    int guidance_case = -1; // 0: approach, 1: backoff, 2: overlap-backoff

    if (distance > ATTACK_DISTANCE) {
    // 距离大于30cm，在线段上取离目标点ATTACK_DISTANCE的点
    double scale = 1.0 - ATTACK_DISTANCE / distance;
    point.x = current_x + dx * scale;
    point.y = current_y + dy * scale;
    guidance_case = 0;
  } else {
    // 距离小于等于30cm，沿着两点连线方向后退30cm
    if (distance > 0.001) { // 避免除零
      double ux = dx / distance;
      double uy = dy / distance;
      point.x = current_x - ux * ATTACK_DISTANCE;
      point.y = current_y - uy * ATTACK_DISTANCE;
      guidance_case = 1;
    } else {
      // 如果距离几乎为0（当前位置与目标点重合）
      point.x = current_x;
      point.y = current_y - ATTACK_DISTANCE;
      guidance_case = 2;
    }
  }

    static int last_guidance_case = -1;
    if(guidance_case != last_guidance_case)
    {
      if(guidance_case == 0)
      {
        std::cout << YELLOW << "距离(" << distance << "m)大于30cm,沿连线方向前进到距离目标点30cm位置"
                  << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET
                  << std::endl;
      }
      else if(guidance_case == 1)
      {
        std::cout << YELLOW << "距离(" << distance << "m)小于等于30cm,沿连线方向后退30cm"
                  << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET << std::endl;
      }
      else if(guidance_case == 2)
      {
        std::cout << YELLOW << "当前位置与目标点重合,向y轴负方向后退30cm"
                  << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET << std::endl;
      }
      last_guidance_case = guidance_case;
    }

    Sentry_BT::Point2D old_goal;
    bool has_old_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal", old_goal);
    static bool last_rate_limited = false;
    
    if (has_old_goal) {
      double diff_x = point.x - old_goal.x;
      double diff_y = point.y - old_goal.y;
      double diff_distance = std::sqrt(diff_x*diff_x + diff_y*diff_y);
      
      // 如果新目标点跟老目标点的差距小于 0.5 米，就不更新
      if (diff_distance < 0.5) {
        if(!last_rate_limited)
        {
          std::cout << WHITE << "Target update skipped by 0.5m limiter" << RESET << std::endl;
        }
        last_rate_limited = true;
        return BT::NodeStatus::SUCCESS; // 直接返回成功，放过底层
      }
    }
    last_rate_limited = false;

    // 将目标点设置到黑板
    blackboard->set("nav_goal", point);
    std::cout << GREEN << "Set target pose to: (" << point.x << ", " << point.y << ")"
              << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET << std::endl;
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

    static int last_logged_index = -1;
    if(current_index != last_logged_index)
    {
      std::cout << GREEN << "Selected patrol point " << current_index << ": (" << selected_point.x << ", "
                << selected_point.y << ")" << RESET << std::endl;
      last_logged_index = current_index;
    }

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
    auto wait_time = blackboard->get<int>("patrol_wait_time");

    if(!wait_time)
    {
      auto time = getInput<int>("milliseconds");
      wait_time = time.value();
    }

    static int last_wait_time = -1;
    if(wait_time != last_wait_time)
    {
      std::cout << GREEN << "Waiting for " << wait_time << " milliseconds" << RESET << std::endl;
      last_wait_time = wait_time;
    }

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
  
  if (!linear_y || !duration) {
    std::cerr << "参数缺失: linear_y 或 duration" << std::endl;
    return BT::NodeStatus::FAILURE; // 参数缺失
  }
  
  // 2. 存储参数
  linear_y_ = linear_y.value();
  angular_z_ = angular_z.value_or(0.0);
  duration_ = duration.value();

  static double last_linear_y = std::numeric_limits<double>::quiet_NaN();
  static double last_angular_z = std::numeric_limits<double>::quiet_NaN();
  static double last_duration = std::numeric_limits<double>::quiet_NaN();
  if(linear_y_ != last_linear_y || angular_z_ != last_angular_z || duration_ != last_duration)
  {
    std::cout << MAGENTA << "DirectVelocityControl start: linear_y=" << linear_y_ << ", angular_z=" << angular_z_
              << ", duration=" << duration_ << "s" << RESET << std::endl;
    last_linear_y = linear_y_;
    last_angular_z = angular_z_;
    last_duration = duration_;
  }
  
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
  
  // 创建 Sentry_BT::Point2D 类型的固定目标点
  Sentry_BT::Point2D goal_point;
  goal_point.x = 9.5;  
  goal_point.y = 1.5;  

  blackboard->set("nav_goal", goal_point);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT
