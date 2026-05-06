#include "bt_manager/action/auto_actions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
using namespace color_text;
namespace Sentry_BT {

// ------------------- SetCoordinate -------------------
SetCoordinate::SetCoordinate(const std::string & name, const BT::NodeConfiguration & config)
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

  if (goal_index.value() < 0 || goal_index.value() >= static_cast<int>(nav_points.size())) {
    return BT::NodeStatus::FAILURE;
  }

  static const std::array<std::string, 5> goal_names = {
    "HOME", "BONUS", "OUTPOST", "OWN_FORT", "ENEMY_FORT"};
  Sentry_BT::Point2D point = nav_points[goal_index.value()];

  auto blackboard = config().blackboard;
  blackboard->set("nav_goal", point);
  static int last_goal_index = -1;
  if (goal_index.value() != last_goal_index) {
    std::cout << CYAN << "[NAV_TREE]" << GREEN << "Set navigation goal to "
              << goal_names[goal_index.value()] << ": (" << point.x << ", " << point.y << ")" << RESET
              << std::endl;
    last_goal_index = goal_index.value();
  }
  return BT::NodeStatus::SUCCESS;
}

// ------------------- SetTargetCoordinate -------------------
SetTargetCoordinate::SetTargetCoordinate(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetTargetCoordinate::providedPorts()
{
  return {BT::InputPort<double>("tracing_dist")};
}

BT::NodeStatus SetTargetCoordinate::tick()
{
  auto blackboard = config().blackboard;
  auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
  auto tracing_dist = getInput<double>("tracing_dist").value_or(0.3);
  Sentry_BT::Point2D point;  //最终目标点
  //获取当前位置
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  // 提取坐标
  double current_x = current_pose.position.x;
  double current_y = current_pose.position.y;
  double target_x = target_pose.position.x;
  double target_y = target_pose.position.y;
  double dx = target_x - current_x;
  double dy = target_y - current_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  int guidance_case = -1;
  if (distance > 0.001) {
    double ux = dx / distance;
    double uy = dy / distance;
    point.x = target_x - ux * tracing_dist;
    point.y = target_y - uy * tracing_dist;
    guidance_case = 0;
  }
  static int last_guidance_case = -1;
  if (guidance_case != last_guidance_case) {
    std::cout << CYAN << "[NAV_TREE]" << GREEN << "Target_dist(" << distance
              << "m)" << "threshold(" << tracing_dist << "m), approach target along the line"
              << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET << std::endl;
    last_guidance_case = guidance_case;
  }
  Sentry_BT::Point2D old_goal;
  bool has_old_goal = blackboard->get<Sentry_BT::Point2D>("nav_goal", old_goal);
  static bool last_rate_limited = false;

  if (has_old_goal) {
    double diff_x = point.x - old_goal.x;
    double diff_y = point.y - old_goal.y;
    double diff_distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);

    // 如果新目标点跟老目标点的差距小于 0.5 米，就不更新
    if (diff_distance < 0.5) {
      if (!last_rate_limited) {
        std::cout << CYAN << "[NAV_TREE]" << WHITE << "Target update skipped by 0.5m limiter" << RESET
                  << std::endl;
      }
      last_rate_limited = true;
      return BT::NodeStatus::SUCCESS;  // 直接返回成功，放过底层
    }
  }
  last_rate_limited = false;

  // 将目标点设置到黑板
  blackboard->set("nav_goal", point);
  std::cout << CYAN << "[NAV_TREE]" << GREEN << "Set target pose to: (" << point.x << ", " << point.y << ")"
            << " | current_pose=(" << current_x << ", " << current_y << ")" << RESET << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// ------------------- SetManualOverrideGoal -------------------
SetManualOverrideGoal::SetManualOverrideGoal(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetManualOverrideGoal::providedPorts()
{
  return {};
}

BT::NodeStatus SetManualOverrideGoal::tick()
{
  auto blackboard = config().blackboard;

  const auto manual_goal = blackboard->get<Sentry_BT::Point2D>("manual_override_goal");
  blackboard->set("nav_goal", manual_goal);
  blackboard->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::MANUAL));

  static Sentry_BT::Point2D last_goal;
  static bool has_last_goal = false;
  const bool goal_changed =
    !has_last_goal || std::hypot(manual_goal.x - last_goal.x, manual_goal.y - last_goal.y) > 0.01;
  if (goal_changed) {
    std::cout << CYAN << "Set manual override goal to (" << manual_goal.x << ", " << manual_goal.y << ")"
              << RESET << std::endl;
    last_goal = manual_goal;
    has_last_goal = true;
  }

  return BT::NodeStatus::SUCCESS;
}

// ------------------- SelectPatrolPoint -------------------
SelectPatrolPoint::SelectPatrolPoint(const std::string & name, const BT::NodeConfiguration & config)
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
  blackboard->set<Sentry_BT::ControlMode>("control_mode", Sentry_BT::ControlMode::AUTO);

  // 获取当前巡逻索引
  int current_index = 0;
  if (auto index = blackboard->get<int>("patrol_index")) {
    current_index = index;
  } else {
    blackboard->set("patrol_index", current_index);
  }

  TacticalMode tactical_mode = TacticalMode::BALANCED;
  blackboard->get<TacticalMode>("tactical_mode", tactical_mode);
  std::vector<Sentry_BT::PatrolPoint> patrol_points = Sentry_BT::patrol_points_normal;
  const auto patrol_it = tactical_patrol_map.find(tactical_mode);
  if (patrol_it != tactical_patrol_map.end() && !patrol_it->second.empty()) {
    patrol_points = patrol_it->second;
  }

  // 检查索引有效性
  if (current_index >= static_cast<int>(patrol_points.size())) {
    current_index = 0;
    blackboard->set("patrol_index", current_index);
  }

  Sentry_BT::Point2D selected_point = patrol_points[current_index].position;

  int next_index = (current_index + 1) % patrol_points.size();
  blackboard->set("patrol_index", next_index);
  blackboard->set("nav_goal", selected_point);
  blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);

  static int last_logged_index = -1;
  if (current_index != last_logged_index) {
    std::cout << CYAN << "[NAV_TREE]" << GREEN << "Selected patrol point " << current_index << ": ("
              << selected_point.x << ", " << selected_point.y << ")" << RESET << std::endl;
    last_logged_index = current_index;
  }

  return BT::NodeStatus::SUCCESS;
}

// ------------------- Wait -------------------
Wait::Wait(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), wait_time_(0), start_time_(std::chrono::system_clock::now())
{
}

BT::PortsList Wait::providedPorts()
{
  return {BT::InputPort<int>("milliseconds")};
}

BT::NodeStatus Wait::onStart()
{
  auto blackboard = config().blackboard;
  auto patrol_index = blackboard->get<int>("patrol_index");
  TacticalMode tactical_mode = TacticalMode::BALANCED;
  blackboard->get<TacticalMode>("tactical_mode", tactical_mode);
  auto patrol_list_it = tactical_patrol_map.find(tactical_mode);
  const auto & patrol_list =
    (patrol_list_it != tactical_patrol_map.end() && !patrol_list_it->second.empty())
      ? patrol_list_it->second
      : patrol_points_normal;

  if (patrol_index >= static_cast<int>(patrol_list.size())) {
    patrol_index = 0;
  }
  auto wait_time = patrol_list[patrol_index].duration_ms;
  if (!wait_time) {
    auto time = getInput<int>("milliseconds");
    wait_time = time.value();
  }

  static int last_wait_time = -1;
  if (wait_time != last_wait_time) {
    std::cout << CYAN << "[NAV_TREE]" << GREEN << "Waiting for " << wait_time << " milliseconds" << RESET
              << std::endl;
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
  if (elapsed < wait_time_) {
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::SUCCESS;
}

void Wait::onHalted()
{
}
// ------------------- SetStairsPosition -------------------
SetStairsPosition::SetStairsPosition(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

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
  goal_point.y = 1.0;

  blackboard->set("nav_goal", goal_point);

  return BT::NodeStatus::SUCCESS;
}

// ------------------- DescendStairsAction -------------------
DescendStairsAction::DescendStairsAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  timeout_ = 0.0;
  start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

BT::PortsList DescendStairsAction::providedPorts()
{
  return {BT::InputPort<double>("linear_y", 2.0, "Linear velocity y"),
    BT::InputPort<double>("angular_z", 0.0, "Angular velocity z"),
    BT::InputPort<double>("timeout", 5.0, "Timeout in seconds")};
}

BT::NodeStatus DescendStairsAction::onStart()
{
  auto ros_interface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  start_time_ = ros_interface->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DescendStairsAction::onRunning()
{
  auto blackboard = config().blackboard;
  //超时退出
  timeout_ = getInput<double>("timeout").value_or(5.0);
  auto ros_interface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  auto current_time = ros_interface->now();
  if ((current_time - start_time_).seconds() > timeout_) {
    std::cout << CYAN << "[NAV_TREE]" << RED << "DescendStairsAction timed out after " << timeout_
              << " seconds" << RESET << std::endl;
    blackboard->set("cmd_vel", geometry_msgs::msg::Twist());
    return BT::NodeStatus::SUCCESS;
  }
  const auto pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
  bool in_stairs = false;
  for (auto zone : stairs_zone) {
    if (zone.contains({pose.position.x, pose.position.y, 0.0})) {
      in_stairs = true;
      break;
    }
  }
  if (!in_stairs) {
    blackboard->set("cmd_vel", geometry_msgs::msg::Twist());
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.y = getInput<double>("linear_y").value_or(2.0);
  cmd_vel.angular.z = getInput<double>("angular_z").value_or(0.0);
  blackboard->set("cmd_vel", cmd_vel);
  return BT::NodeStatus::RUNNING;
}

void DescendStairsAction::onHalted()
{
  config().blackboard->set("cmd_vel", geometry_msgs::msg::Twist());
}

// ------------------- AccumulateAmmoPurchase -------------------
AccumulateAmmoPurchase::AccumulateAmmoPurchase(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList AccumulateAmmoPurchase::providedPorts()
{
  return {BT::InputPort<int>("step", 30, "Minimum ammo purchase step")};
}

BT::NodeStatus AccumulateAmmoPurchase::tick()
{
  auto blackboard = config().blackboard;
  const int step = getInput<int>("step").value_or(100);
  const int ammo = blackboard->get<int>("bullets_remaining");
  int total = blackboard->get<int>("ammo_purchase_total");

  const int shortage = std::max(0, 100 - ammo);
  const int request = std::max(step, shortage);
  total += request;
  blackboard->set("ammo_purchase_total", total);

  static int last_total = -1;
  if (total != last_total) {
    std::cout << CYAN << "[NAV_TREE]" << GREEN << "AccumulateAmmoPurchase => request=" << request
              << ", total=" << total << RESET << std::endl;
    last_total = total;
  }
  return BT::NodeStatus::SUCCESS;
}

// ------------------- ChangeMapAction -------------------
ChangeMapAction::ChangeMapAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ChangeMapAction::providedPorts()
{
  return {BT::InputPort<std::string>("map_dir", "Directory containing map files"),
    BT::InputPort<std::string>("pcd_dir", "Directory containing ESDF PCD files"),
    BT::InputPort<std::string>("map_name", "Base name of the map files (without extensions)")};
}

BT::NodeStatus ChangeMapAction::tick()
{
  static std::string last_map_name;

  auto map_dir = getInput<std::string>("map_dir");
  if (!map_dir) {
    throw BT::RuntimeError("missing required input [map_dir]: ", map_dir.error());
  }

  auto map_name = getInput<std::string>("map_name");
  if (!map_name) {
    throw BT::RuntimeError("missing required input [map_name]: ", map_name.error());
  }

  if (map_name.value() == last_map_name) {
    return BT::NodeStatus::SUCCESS;
  }

  auto pcd_dir = getInput<std::string>("pcd_dir");
  const std::string pcd_dir_value = pcd_dir ? pcd_dir.value() : map_dir.value();

  std::string yaml_path = map_dir.value() + "/" + map_name.value() + ".yaml";
  std::string pcd_path = pcd_dir_value + "/" + map_name.value() + "_esdf.pcd";

  auto ros_iface = config().blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  if (!ros_iface || !ros_iface->getParamManager()) {
    throw BT::RuntimeError("ros_interface or ParamManager is not available from blackboard");
  }

  std::cout << CYAN << "Changing map to: " << map_name.value() << " (Paths: " << yaml_path << ", "
            << pcd_path << ")" << RESET << std::endl;

  ros_iface->getParamManager()->changeMapAndPcd(yaml_path, pcd_path);
  last_map_name = map_name.value();
  return BT::NodeStatus::SUCCESS;
}
// ------------------- ControlThroughTunnel -------------------
ControlThroughTunnel::ControlThroughTunnel(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ControlThroughTunnel::providedPorts()
{
  return {};
}

BT::NodeStatus ControlThroughTunnel::onStart()
{
  auto blackboard = config().blackboard;
  auto ros_iface = blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  is_through_tunnel_ = blackboard->get<bool>("through_tunnel");
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(ros_iface, "/controller_server");
  if (!wait_param_service()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!wait_param_available()) {
    return BT::NodeStatus::FAILURE;
  }
  auto get_params_future = parameters_client_->get_parameters(required_params_);
  if (get_params_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    std::cerr << "Failed to get parameters within timeout." << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  auto params = get_params_future.get();
  for (const auto & param : params) {
    if (param.get_name() == "FollowPath.use_small_gyro_mode") {
      initial_small_gyoro_ = param.as_bool();
    } else if (param.get_name() == "FollowPath.enable_yaw_opt") {
      initial_yaw_opt_ = param.as_bool();
    }
  }
  auto set_param_future =
    parameters_client_->set_parameters({rclcpp::Parameter("FollowPath.use_small_gyro_mode", false),
      rclcpp::Parameter("FollowPath.enable_yaw_opt", true)});
  if (set_param_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    std::cerr << "Failed to set parameters within timeout." << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  // last_pub_time_ = ros_iface->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ControlThroughTunnel::onRunning()
{
  auto blackboard = config().blackboard;
  auto ros_iface = blackboard->get<std::shared_ptr<Sentry_BT::ros_interface>>("ros_interface");
  if (!ros_iface) {
    return BT::NodeStatus::FAILURE;
  }
  is_through_tunnel_ = blackboard->get<bool>("through_tunnel");
  RCLCPP_INFO(ros_iface->get_logger(), "is_through_tunnel: %s", is_through_tunnel_ ? "true" : "false");
  if (!is_through_tunnel_) {
    auto set_param_future = parameters_client_->set_parameters(
      {rclcpp::Parameter("FollowPath.use_small_gyro_mode", initial_small_gyoro_),
        rclcpp::Parameter("FollowPath.enable_yaw_opt", initial_yaw_opt_)});
    if (set_param_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      std::cerr << "Failed to reset parameters within timeout." << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
  auto current_orientation = blackboard->get<geometry_msgs::msg::Pose>("current_pose").orientation;
  
  // 将四元数转换为欧拉角
  tf2::Quaternion quat(
    current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
  double yaw_diff = current_yaw_ - target_yaw_;
  if (yaw_diff > M_PI) {
    yaw_diff -= 2 * M_PI;
  } else if (yaw_diff < -M_PI) {
    yaw_diff += 2 * M_PI;
  }
  if (std::abs(yaw_diff) < 0.1) {
    if (!yaw_ready_) {
      yaw_ready_ = true;
      std::cout << GREEN << "Yaw is ready." << RESET << std::endl;
    }
  }
  // else {
  //   if (yaw_ready_) {
  //     geometry_msgs::msg::Twist vel;
  //     geometry_msgs::msg::Twist current_cmd_vel = blackboard->get<geometry_msgs::msg::Twist>("cmd_vel");
  //     vel.linear = current_cmd_vel.linear;
  //     double wz = yaw_diff * 4.0; // 简单的比例控制
  //     wz = std::clamp(wz, -fixed_wz_, fixed_wz_);
  //     vel.angular.z = wz;
  //     rclcpp::Time now = ros_iface->now();
  //     if ((now - last_pub_time_).seconds() >= 0.05) { // 20Hz发布频率
  //       last_pub_time_ = now;
  //     }
  //   }
  // }
  LifterPos lifter_pose = blackboard->get<LifterPos>("lifter_current_pos");
  if (lifter_pose == LifterPos::BOTTOM && !lifter_ready_) {
    std::cout << GREEN << "Lifter is ready." << RESET << std::endl;
    lifter_ready_ = true;
  }
  return BT::NodeStatus::RUNNING;
}

bool ControlThroughTunnel::wait_param_service()
{
  int time_count = 0;
  while (!parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      std::cerr << "Interrupted while waiting for the service. Exiting." << std::endl;
      return false;
    }
    std::cout << "Service not available, waiting again..." << std::endl;
    time_count++;
    if (time_count > 2) {
      std::cerr << "Service not available after waiting for 5 seconds. Exiting." << std::endl;
      return false;
    }
  }
  return true;
}

bool ControlThroughTunnel::wait_param_available()
{
  auto start_time = std::chrono::steady_clock::now();
  while (!has_params(required_params_)) {
    if (!rclcpp::ok()) {
      std::cerr << "Interrupted while waiting for parameters. Exiting." << std::endl;
      return false;
    }
    std::cout << "Parameters not available, waiting again..." << std::endl;
    auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time);
    if (elapsed > std::chrono::milliseconds(200)) {
      std::cerr << "Parameters not available after waiting. Exiting." << std::endl;
      return false;
    }
  }
  return true;
}

bool ControlThroughTunnel::has_params(std::vector<std::string> param_names)
{
  auto list_future = parameters_client_->get_parameters(param_names);
  if (list_future.wait_for(std::chrono::milliseconds(30)) != std::future_status::ready) {
    std::cerr << "Failed to get parameters within timeout." << std::endl;
    return false;
  }
  auto parameters = list_future.get();
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      std::cerr << "Parameter " << param.get_name() << " is not set." << std::endl;
      return false;
    }
  }
  return true;  // 所有参数都可用
}

void ControlThroughTunnel::onHalted()
{
  return;
}
}  // namespace Sentry_BT
