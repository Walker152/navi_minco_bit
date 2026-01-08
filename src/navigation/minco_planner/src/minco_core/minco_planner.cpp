#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>
#include <stdexcept>
#include <cmath>
#include <cstring>

#include "sensor_msgs/msg/point_field.hpp"

namespace minco_planner
{

void MincoPlanner::publishEsdfCloud(const std_msgs::msg::Header & header)
{
  if (!esdf_cloud_pub_ || !esdf_map_) {
    return;
  }

  const int width = esdf_map_->getWidth();
  const int height = esdf_map_->getHeight();
  if (width <= 0 || height <= 0) {
    return;
  }

  const auto & data = esdf_map_->getData();
  if (data.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(static_cast<size_t>(width) * static_cast<size_t>(height));
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  cloud.fields.resize(4);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;

  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height);

  const double res = esdf_map_->getResolution();
  const auto origin = esdf_map_->getOrigin();

  size_t point_index = 0;
  for (int iy = 0; iy < height; ++iy)
  {
    for (int ix = 0; ix < width; ++ix, ++point_index)
    {
      const size_t idx = static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix);
      const float x = static_cast<float>(origin.x() + ix * res);
      const float y = static_cast<float>(origin.y() + iy * res);
      const float z = 0.0f;
      const float intensity = static_cast<float>((idx < data.size()) ? data[idx] : 0.0);

      const size_t base = point_index * static_cast<size_t>(cloud.point_step);
      std::memcpy(&cloud.data[base + 0], &x, sizeof(float));
      std::memcpy(&cloud.data[base + 4], &y, sizeof(float));
      std::memcpy(&cloud.data[base + 8], &z, sizeof(float));
      std::memcpy(&cloud.data[base + 12], &intensity, sizeof(float));
    }
  }

  esdf_cloud_pub_->publish(cloud);
}

void MincoPlanner::publishOptimizedTrajectory(
  const traj_opt::Trajectory & opt_traj,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step)
{
  if (!opt_path_pub_ || steps <= 0) {
    return;
  }

  ros_interfaces::msg::MpcPositionCommand traj_msg;
  traj_msg.header = header;
  traj_msg.command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
  traj_msg.cmds.resize(steps);

  const uint32_t traj_id = ++opt_trajectory_id_;
  for (int i = 0; i < steps; ++i)
  {
    double t = i * t_step;
    if (t > opt_traj.getTotalDuration()) {
      t = opt_traj.getTotalDuration();
    }
    const Eigen::Vector3d pos = opt_traj.getPos(t);
    const Eigen::Vector3d vel = opt_traj.getVel(t);
    const Eigen::Vector3d acc = opt_traj.getAcc(t);
    const Eigen::Vector3d jer = opt_traj.getJer(t);

    double yaw = 0.0;
    if (vel.norm() > 1e-4) {
      yaw = std::atan2(vel(1), vel(0));
    } else if (i > 0) {
      yaw = traj_msg.cmds[i - 1].yaw;
    }

    auto & cmd = traj_msg.cmds[i];
    cmd.header = traj_msg.header;
    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = 0.0;
    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);
    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);
    cmd.jerk.x = jer(0);
    cmd.jerk.y = jer(1);
    cmd.jerk.z = jer(2);
    cmd.angular_velocity.x = 0.0;
    cmd.angular_velocity.y = 0.0;
    cmd.angular_velocity.z = 0.0;
    cmd.yaw = yaw;
    cmd.yaw_dot = 0.0;
    cmd.vel_norm = vel.norm();
    cmd.acc_norm = acc.norm();
    cmd.kx = {0.0, 0.0, 0.0};
    cmd.kv = {0.0, 0.0, 0.0};
    cmd.trajectory_id = traj_id;
  }

  traj_msg.mpc_horizon = static_cast<uint32_t>(traj_msg.cmds.size());
  opt_path_pub_->publish(traj_msg);
}

MincoPlanner::MincoPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

MincoPlanner::~MincoPlanner()
{
}

void MincoPlanner::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = parent.lock();
  logger_ = node->get_logger();
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_astar", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_astar", use_astar_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.safe_dist", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".minco_optimizer.safe_dist", minco_config.safe_dist);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.max_velocity", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".minco_optimizer.max_velocity", minco_config.max_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.max_acceleration", rclcpp::ParameterValue(4.0));
  node->get_parameter(name + ".minco_optimizer.max_acceleration", minco_config.max_acc);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.time_allocation_iters", rclcpp::ParameterValue(15));
  node->get_parameter(name + ".minco_optimizer.time_allocation_iters", minco_config.time_allocation_iters);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.penalty_weight_time", rclcpp::ParameterValue(0.01));
  node->get_parameter(name + ".minco_optimizer.penalty_weight_time", minco_config.rho);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.smooth_eps", rclcpp::ParameterValue(0.01));
  node->get_parameter(name + ".minco_optimizer.smooth_eps", minco_config.smooth_eps);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.integral_res", rclcpp::ParameterValue(16));
  node->get_parameter(name + ".minco_optimizer.integral_res", minco_config.integral_res);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.opt_accuracy", rclcpp::ParameterValue(1.0e-4));
  node->get_parameter(name + ".minco_optimizer.opt_accuracy", minco_config.opt_accuracy);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.print_optimizer_log", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".minco_optimizer.print_optimizer_log", minco_config.print_optimizer_log);

  double penalty_weight_pos, penalty_weight_vel, penalty_weight_acc, penalty_weight_att;
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.penalty_weight_pos", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".minco_optimizer.penalty_weight_pos", penalty_weight_pos);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.penalty_weight_vel", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".minco_optimizer.penalty_weight_vel", penalty_weight_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.penalty_weight_acc", rclcpp::ParameterValue(10000.0));
  node->get_parameter(name + ".minco_optimizer.penalty_weight_acc", penalty_weight_acc);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.penalty_weight_att", rclcpp::ParameterValue(1000.0));
  node->get_parameter(name + ".minco_optimizer.penalty_weight_att", penalty_weight_att);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".static_esdf.esdf_pcd_path", rclcpp::ParameterValue("src/utils/pcd2esdf/maps/2026_esdf.pcd"));
  node->get_parameter(name + ".static_esdf.esdf_pcd_path", esdf_pcd_path_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".static_esdf.esdf_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".static_esdf.esdf_resolution", esdf_resolution_);

  minco_config.penaltyWeights.resize(4);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;

  minco_config.magnitudeBounds.resize(3);
  minco_config.magnitudeBounds(0) = minco_config.safe_dist;
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;

  astar_planner_ = std::make_unique<Astar>(
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  opt_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  esdf_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/esdf_cloud", 10);

  // Load Static ESDF Map
  esdf_map_ = std::make_shared<StaticESDFMap>();
  if (!esdf_map_->loadMap(esdf_pcd_path_, esdf_resolution_)) {
    RCLCPP_ERROR(logger_, "MincoPlanner: Failed to load static ESDF map from PCD: %s", esdf_pcd_path_.c_str());
  } else {
    RCLCPP_INFO(logger_, "MincoPlanner: Static ESDF map loaded from PCD: %s", esdf_pcd_path_.c_str());

    // Create 1Hz timer to publish ESDF cloud if there are subscribers
    esdf_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {
        if (esdf_cloud_pub_ && esdf_cloud_pub_->get_subscription_count() > 0) {
          auto node_ptr = node_.lock();
          if (node_ptr) {
            std_msgs::msg::Header header;
            header.stamp = node_ptr->now();
            header.frame_id = global_frame_;
            publishEsdfCloud(header);
          }
        }
      });
  }

  // Initialize Minco Optimizer
  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setESDFMap(esdf_map_); 
}

void MincoPlanner::cleanup()
{
  astar_planner_.reset();
  minco_optimizer_.reset();
  opt_path_pub_.reset();
  esdf_cloud_pub_.reset();
  esdf_timer_.reset();
}

void MincoPlanner::activate()
{
}

void MincoPlanner::deactivate()
{
}

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  // We use rclcpp::ok() as a basic check.
  auto cancel_checker = []() {
    return !rclcpp::ok();
  };

  if (!astar_planner_ || !costmap_) {
    RCLCPP_ERROR(logger_, "MincoPlanner: planner is not properly configured");
    throw std::runtime_error("MincoPlanner: planner is not properly configured");
  }

  if (!minco_optimizer_) {
    RCLCPP_ERROR(logger_, "MincoPlanner: minco_optimizer is not initialized");
    throw std::runtime_error("MincoPlanner: minco_optimizer is not initialized");
  }

  // 将起点和终点转换到地图坐标系
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw std::runtime_error(
      "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
      std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw std::runtime_error(
      "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (tolerance_ == 0.0 &&
    costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw std::runtime_error(
      "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  // 特殊情况：起点和终点重合，直接返回单点路径
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = start.pose;
    path.poses.push_back(pose);
    return path;
  }

  if (!makePlan(start.pose, goal.pose, tolerance_, cancel_checker, path)) {
    throw std::runtime_error(
            "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
  }
  return path;
}

bool MincoPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  (void)tolerance;
  // 1. Prepare the plan
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  double plan_start_time = rclcpp::Clock().now().seconds() + 0.03;
  Eigen::Matrix3d start_state, end_state;
  start_state.setZero();
  end_state.setZero();

  start_state.col(0) = Eigen::Vector3d(start.position.x, start.position.y, 0.0);

  bool hot_start = false;
  if (has_last_traj_)
  {
    double t_dur = plan_start_time - last_traj_.start_WT;
    if (t_dur > 0.0 && t_dur < last_traj_.getTotalDuration())
    {
      Eigen::Vector3d pos = last_traj_.getPos(t_dur);
      double dist = (pos - start_state.col(0)).norm();
      if (dist < 0.3)
      {
        // start_state.col(0) = last_traj_.getPos(t_dur);
        start_state.col(1) = last_traj_.getVel(t_dur);
        start_state.col(2) = last_traj_.getAcc(t_dur);
        hot_start = true;
      }
    }
  }
  
  if (!hot_start)
  {
    start_state.col(0) = Eigen::Vector3d(start.position.x, start.position.y, 0.0);
  }

  // 2. 使用 A* 算法找到路径
  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start, my_start;
  worldToMap(wx, wy, mx_start, my_start);
  clearRobotCell(mx_start, my_start);

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal, my_goal;
  worldToMap(wx, wy, mx_goal, my_goal);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int nx = costmap_->getSizeInCellsX();
  unsigned int ny = costmap_->getSizeInCellsY();
  astar_planner_->setSize(nx, ny);
  astar_planner_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
  astar_planner_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
  astar_planner_->setupNavFn(true);
  astar_planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  // 传播波前
  // 循环调用 propNavFnAstar 直到找到 Start 或队列为空
  // 给定一个足够大的总循环次数上限，防止死循环
  int max_total_cycles = nx * ny*9999; 
  int cycles_per_step = std::max(nx * ny / 20, nx + ny);
  auto time = rclcpp::Clock().now().seconds();
  while (max_total_cycles > 0) {
    if (cancel_checker && cancel_checker()) {
      return false;
    }
    if (!astar_planner_->propNavFnAstar(cycles_per_step, cancel_checker)) {
      break;
    }
    max_total_cycles -= cycles_per_step;
  }
  auto time_end = rclcpp::Clock().now().seconds();
  RCLCPP_INFO(logger_, "A* planning time: %f seconds", time_end - time);
  // 提取路径
  if (!astar_planner_->calcPath(nx * ny / 2) || astar_planner_->getPathLen() < 2) {
    return false;
  }

  // 取出 A* 路径（地图坐标），转换到世界坐标并构造导航路径
  float * path_x = astar_planner_->getPathX();
  float * path_y = astar_planner_->getPathY();
  const int len = astar_planner_->getPathLen();

  std::vector<Eigen::Vector3d> guide_path;
  guide_path.reserve(len);

  for (int i = 0; i < len; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;

    double wx, wy;
    costmap_->mapToWorld(path_x[i], path_y[i], wx, wy);

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    guide_path.emplace_back(wx, wy, 0.0);
    plan.poses.push_back(pose);
  }

  std::vector<Eigen::Vector3d> sparse_path = getSparseWaypoints(guide_path);
  lock.unlock();
  // 3. 使用 Minco 优化器优化路径
  auto opt_time = rclcpp::Clock().now().seconds();
  traj_opt::Trajectory opt_traj;
  end_state.col(0) = sparse_path.back();
  while (sparse_path.size() > 2)
  {
    Eigen::Vector3d first_pt = sparse_path[1];
    Eigen::Vector3d dir = first_pt - start_state.col(0);
    if ((dir.norm() < 0.2 )) 
    {
      sparse_path.erase(sparse_path.begin() + 1);
    }
    else
    {
      break;
    }
  }
  
  double cost = minco_optimizer_->optimize(sparse_path, start_state, end_state, opt_traj);
  if (std::isinf(cost))
  {
    RCLCPP_WARN(node_.lock()->get_logger(), "Minco optimization failed");
    return false;
  } 
  auto opt_time_end = rclcpp::Clock().now().seconds();
  RCLCPP_INFO(logger_, "Minco optimization time: %f seconds, cost: %f", opt_time_end - opt_time, cost);
  RCLCPP_INFO(logger_, "Start goal: (%.2f, %.2f) -> (%.2f, %.2f), original path length: %d, sparse path length: %d, traj duration: %.2f s",
    start.position.x, start.position.y, goal.position.x, goal.position.y,
    len, static_cast<int>(sparse_path.size()), opt_traj.getTotalDuration());
  // 4. 将优化后的轨迹转换为导航路径
  double t_start = plan_start_time;
  // Re-sample based on fixed time step
  double t_step = 0.05;
  int steps = std::ceil(opt_traj.getTotalDuration() / t_step) + 1;
  steps = std::max(2, steps);
  
  // Resize plan
  plan.poses.resize(steps);
  for (int i = 0; i < steps; ++i) {
      plan.poses[i].header = plan.header;
  }

  for (int i = 0; i < steps; ++i) {
    double t = i * t_step;
    if (t > opt_traj.getTotalDuration()) {
      t = opt_traj.getTotalDuration();
    }
    Eigen::Vector3d pos = opt_traj.getPos(t);
    Eigen::Vector3d vel = opt_traj.getVel(t);

    double yaw = 0.0;
    if (vel.norm() > 1e-4) {
      yaw = std::atan2(vel(1), vel(0));
    } else if (i > 0) {
      yaw = 2.0 * std::atan2(plan.poses[i-1].pose.orientation.z, plan.poses[i-1].pose.orientation.w);
    } else {
      Eigen::Vector3d vel_next = opt_traj.getVel(t + 1e-3);
      if (vel_next.norm() > 1e-4) {
        yaw = std::atan2(vel_next(1), vel_next(0));
      }
    }

    plan.poses[i].pose.position.x = pos(0);
    plan.poses[i].pose.position.y = pos(1);
    plan.poses[i].pose.position.z = 0.0;
    plan.poses[i].pose.orientation.x = 0.0;
    plan.poses[i].pose.orientation.y = 0.0;
    plan.poses[i].pose.orientation.z = sin(yaw / 2.0);
    plan.poses[i].pose.orientation.w = cos(yaw / 2.0);
  }

  RCLCPP_INFO(logger_, "MincoPlanner: Successfully created plan with %d poses, duration %f", steps, opt_traj.getTotalDuration());

  // Publish optimized trajectory using custom message on opt_path
  publishOptimizedTrajectory(opt_traj, plan.header, steps, t_step);

  // 5. 保存优化后的轨迹
  last_traj_ = opt_traj;
  last_traj_.start_WT = t_start;
  has_last_traj_ = true;

  return !plan.poses.empty();
}

std::vector<Eigen::Vector3d> MincoPlanner::getSparseWaypoints(const std::vector<Eigen::Vector3d>& path) {
    std::vector<Eigen::Vector3d> sparse;
    if (path.empty()) return sparse;
    
    sparse.push_back(path.front());
    if (path.size() < 3) {
        sparse.push_back(path.back());
        return sparse;
    }
    
    const int lookahead_step = 2.0 / costmap_->getResolution();
    size_t current_idx = 0;

    while (current_idx < path.size() - 1) {
      size_t next_idx = current_idx + 1;
      Eigen::Vector3d curr_pt = path[current_idx];
      size_t max_lookahead = std::min(static_cast<size_t>(lookahead_step), path.size() - 1 - current_idx);
      for (size_t i = 1; i < max_lookahead; i++) {
        if (!isLineFree(curr_pt, path[current_idx + i])) {
          break;
        }
        next_idx = current_idx + i;
      }
      sparse.push_back(path[next_idx]);
      current_idx = next_idx;
    }
    
    if (sparse.back() != path.back()) {
      sparse.push_back(path.back());
    }

    std::vector<Eigen::Vector3d> final_sparse;
    final_sparse.push_back(sparse.front());
    
    for (size_t i = 1; i < sparse.size() - 1; ++i) {
        const auto& last_pt = final_sparse.back();
        const auto& curr_pt = sparse[i];
        const auto& next_pt = sparse[i+1];
        
        double dist = (curr_pt - last_pt).norm();
        
        // 如果距离太近 (例如 < 1.0m)，且直接连线 (last -> next) 是无碰撞的，则跳过当前点
        if (dist < 1.0 && isLineFree(last_pt, next_pt)) {
             continue; // 跳过这个点，直接尝试连下一个
        }
        
        final_sparse.push_back(curr_pt);
    }
    
    // 确保终点被加入
    final_sparse.push_back(sparse.back());
    return final_sparse;
}

bool MincoPlanner::isLineFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    if (!costmap_) return true; 
    unsigned int mx, my;
    double dist = (p2 - p1).norm();
    int steps = std::ceil(dist / costmap_->getResolution()); 
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d p = p1 + (p2 - p1) * t;
        if (costmap_->worldToMap(p.x(), p.y(), mx, my)) {
            if (costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { 
                return false; 
            }
        }
    }
    return true;
}

bool MincoPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }
  return false;
}

void MincoPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void MincoPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}
}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
