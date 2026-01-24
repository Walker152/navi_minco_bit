#include "minco_core/minco_planner.hpp"
#include "minco_core/minco_fsm.hpp"
#include "minco_core/visualizer.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>
#include <stdexcept>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace minco_planner
{
using namespace color_text;

namespace
{
template<typename T>
inline T clampValue(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

double getDistFromTrapezoid(
  double t,
  double total_length,
  double a_ref,
  double v_peak,
  double t_acc,
  double t_flat)
{
  if (!(std::isfinite(t) && std::isfinite(total_length) && std::isfinite(a_ref) && std::isfinite(v_peak) &&
        std::isfinite(t_acc) && std::isfinite(t_flat))) {
    return 0.0;
  }

  if (total_length <= 0.0) {
    return 0.0;
  }
  if (a_ref <= 0.0 || v_peak <= 0.0 || t_acc <= 0.0) {
    return 0.0;
  }

  const double t_total = 2.0 * t_acc + std::max(0.0, t_flat);
  if (t <= 0.0) {
    return 0.0;
  }
  if (t >= t_total) {
    return total_length;
  }

  const double d_acc = 0.5 * v_peak * v_peak / a_ref;
  const double d_flat = v_peak * std::max(0.0, t_flat);

  double s = 0.0;
  if (t < t_acc) {
    // s = 1/2 a t^2
    s = 0.5 * a_ref * t * t;
  } else if (t < t_acc + t_flat) {
    // s = d_acc + v * (t - t_acc)
    s = d_acc + v_peak * (t - t_acc);
  } else {
    // Decel: s = d_acc + d_flat + v*t_dec - 1/2 a t_dec^2
    const double t_dec = t - (t_acc + t_flat);
    s = d_acc + d_flat + v_peak * t_dec - 0.5 * a_ref * t_dec * t_dec;
  }

  if (!std::isfinite(s)) {
    return 0.0;
  }
  return clampValue(s, 0.0, total_length);
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((unused))
#endif
Eigen::Vector3d interpolateByArcLength(
  const std::vector<Eigen::Vector3d> & path,
  const std::vector<double> & accumulated_dist,
  double s)
{
  if (path.empty()) {
    return Eigen::Vector3d::Zero();
  }
  if (path.size() == 1 || accumulated_dist.size() != path.size()) {
    return path.front();
  }

  const double s_clamped = clampValue(s, 0.0, accumulated_dist.back());
  auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), s_clamped);
  if (it == accumulated_dist.begin()) {
    return path.front();
  }
  if (it == accumulated_dist.end()) {
    return path.back();
  }

  const size_t idx1 = static_cast<size_t>(std::distance(accumulated_dist.begin(), it));
  const size_t idx0 = idx1 - 1;
  const double s0 = accumulated_dist[idx0];
  const double s1 = accumulated_dist[idx1];
  const double denom = (s1 - s0);
  if (denom <= 1e-9) {
    return path[idx1];
  }
  const double ratio = clampValue((s_clamped - s0) / denom, 0.0, 1.0);
  return path[idx0] + ratio * (path[idx1] - path[idx0]);
}
}  // namespace

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

void MincoPlanner::publishBackupTrajectory(
  const traj_opt::Trajectory & backup_traj,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step)
{
  if (!backup_path_pub_ || steps <= 0) {
    return;
  }

  ros_interfaces::msg::MpcPositionCommand traj_msg;
  traj_msg.header = header;
  traj_msg.command_flag = ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND;
  traj_msg.cmds.resize(steps);

  const uint32_t traj_id = ++backup_trajectory_id_;
  for (int i = 0; i < steps; ++i)
  {
    double t = i * t_step;
    if (t > backup_traj.getTotalDuration()) {
      t = backup_traj.getTotalDuration();
    }

    Eigen::Vector3d pos = backup_traj.getPos(t);
    Eigen::Vector3d vel = backup_traj.getVel(t);
    Eigen::Vector3d acc = backup_traj.getAcc(t);
    Eigen::Vector3d jer = backup_traj.getJer(t);

    // Force 2D consistency for ground robot.
    pos.z() = 0.0;
    vel.z() = 0.0;
    acc.z() = 0.0;
    jer.z() = 0.0;

    double yaw = 0.0;
    if (vel.head<2>().norm() > 1e-4) {
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
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = 0.0;
    cmd.jerk.x = jer(0);
    cmd.jerk.y = jer(1);
    cmd.jerk.z = 0.0;
    cmd.angular_velocity.x = 0.0;
    cmd.angular_velocity.y = 0.0;
    cmd.angular_velocity.z = 0.0;
    cmd.yaw = yaw;
    cmd.yaw_dot = 0.0;
    cmd.vel_norm = vel.head<2>().norm();
    cmd.acc_norm = acc.head<2>().norm();
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

traj_opt::Trajectory MincoPlanner::generateBackupTraj(const Eigen::Matrix3d& start_state)
{
  auto make_stop_traj = [&start_state]() -> traj_opt::Trajectory {
    traj_opt::Trajectory stop_traj;
    const Eigen::Vector3d p = start_state.col(0);

    Eigen::MatrixXd cMat(3, 1);
    cMat.col(0) = p;

    // Two very short constant pieces ("2 points" semantics).
    stop_traj.emplace_back(0.02, cMat);
    stop_traj.emplace_back(0.02, cMat);
    return stop_traj;
  };

  if (!corridor_gen_ || !backup_opt_) {
    std::cout << RED << "[MincoPlanner] Backup optimizer not initialized!" << RESET << std::endl;
    auto stop_traj = make_stop_traj();
    return stop_traj;
  }

  // Step 1: Generate SFC (safe box)
  auto safe_poly = corridor_gen_->generateSafeBox(start_state.col(0), 1.0);

  // Step 2: Setup backup optimizer
  backup_opt_->setInitState(start_state);
  backup_opt_->setStopConstraints();
  backup_opt_->setPolygons({safe_poly});

  // Step 3: Optimize
  traj_opt::Trajectory backup_traj;
  bool success = backup_opt_->optimize(backup_traj);

  // Step 4: Return
  if (success) {
    return backup_traj;
  }

  std::cout << RED << "[MincoPlanner] Backup trajectory optimization failed, fallback to stop." << RESET
            << std::endl;
  auto stop_traj = make_stop_traj();
  return stop_traj;
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

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.opt_freq", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".minco_optimizer.opt_freq", opt_freq_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.lookahead_dist", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".minco_optimizer.lookahead_dist", lookahead_dist_);

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
  
  // Control publisher
  opt_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  backup_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/backup_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Load Static ESDF Map
  esdf_map_ = std::make_shared<small_rog_map::HybridESDFMap>();

  // Initialize ESDF dynamic layer ROS subscription (STVL voxel_grid).
  esdf_map_->initRos(parent, "/global_costmap/voxel_grid");

  bool esdf_loaded = esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);
  if (!esdf_loaded) {
    std::cout << RED << "[MincoPlanner] "
              << "Failed to load Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
  } else {
    std::cout << MAGENTA << "[MincoPlanner] "
              << "Successfully loaded Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
  }

  // Visualization / ESDF publishing helper
  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, global_frame_, esdf_map_, esdf_loaded);

  // Initialize Minco Optimizer
  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>(esdf_map_);
  backup_opt_ = std::make_unique<traj_opt::BackupTrajOpt>();
  minco_optimizer_->setESDFMap(esdf_map_); 

  // Planner handle for FSM (non-owning; lifetime managed by pluginlib).
  planner_handle_ = MincoPlanner::Ptr(this, [](MincoPlanner *) {});

  // High-level FSM @ 20Hz.
  fsm_ = std::make_unique<MincoFsm>(planner_handle_);
  fsm_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / 20.0),
    [this]() {
      if (fsm_) {
        fsm_->callMainFsmOnce();
      }
    });

  // Asynchronous safety monitor @ 20Hz.
  safety_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / 20.0),
    std::bind(&MincoPlanner::safetyTimerCallback, this));
}

void MincoPlanner::cleanup()
{
  fsm_timer_.reset();
  safety_timer_.reset();

  fsm_.reset();
  planner_handle_.reset();

  if (visualizer_) {
    visualizer_->cleanup();
    visualizer_.reset();
  }

  astar_planner_.reset();
  minco_optimizer_.reset();
  backup_opt_.reset();
  opt_path_pub_.reset();
  backup_path_pub_.reset();

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
  // Nav2 interface: createPlan() only sets the goal flag for MincoFSM.
  // It must NOT run A* or optimization here.
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  // Keep a minimal path for Nav2 callers (e.g., visualization/debug).
  path.poses.push_back(start);
  path.poses.push_back(goal);

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    pending_goal_ = goal;
    has_pending_goal_ = true;
  }

  return path;
}

bool MincoPlanner::consumePendingGoal(geometry_msgs::msg::PoseStamped & goal_out)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  if (!has_pending_goal_) {
    return false;
  }
  goal_out = pending_goal_;
  has_pending_goal_ = false;
  return true;
}

double MincoPlanner::nowSeconds() const
{
  return rclcpp::Clock().now().seconds();
}

bool MincoPlanner::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  if (!costmap_ros_) {
    return false;
  }
  return costmap_ros_->getRobotPose(pose);
}

bool MincoPlanner::isTrajectoryTimeExpired(double now_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_traj_) {
    return true;
  }
  const double end_s = last_traj_.start_WT + last_traj_.getTotalDuration();
  return now_s > end_s;
}

bool MincoPlanner::PlanGlobalPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!astar_planner_ || !costmap_) {
    return false;
  }

  nav_msgs::msg::Path dummy;
  dummy.header.stamp = rclcpp::Clock().now();
  dummy.header.frame_id = global_frame_;

  auto cancel_checker = []() {
    return !rclcpp::ok();
  };

  return makePlan(start.pose, goal.pose, tolerance_, cancel_checker, dummy);
}

bool MincoPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  (void)tolerance;
  // 1. Reset plan output and header
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  // 2. Search a discrete guide path using A*
  // Convert world coords to map coords
  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start, my_start;
  if (!worldToMap(wx, wy, mx_start, my_start)) {
    return false;
  }
  clearRobotCell(mx_start, my_start);

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal, my_goal;
  if (!worldToMap(wx, wy, mx_goal, my_goal)) {
    return false;
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int nx = costmap_->getSizeInCellsX();
  unsigned int ny = costmap_->getSizeInCellsY();

  // Setup A* inputs
  astar_planner_->setSize(nx, ny);
  astar_planner_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
  astar_planner_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
  astar_planner_->setupNavFn(true);
  astar_planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  // Run A* wavefront expansion
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
  std::cout << GREEN << "[MincoPlanner] A* planning time: "
            << (time_end - time) << " seconds" << RESET << std::endl;

  // Extract the A* path and convert it to world coords
  if (!astar_planner_->calcPath(nx * ny / 2) || astar_planner_->getPathLen() < 2) {
    return false;
  }

  // Convert map path to nav_msgs::Path
  float * path_x = astar_planner_->getPathX();
  float * path_y = astar_planner_->getPathY();
  const int len = astar_planner_->getPathLen();

  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_.clear();
  latest_global_path_.reserve(len);
  plan.poses.reserve(len);

  for (int i = 0; i < len; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;

    double wx, wy;
    costmap_->mapToWorld(path_x[i], path_y[i], wx, wy);

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    latest_global_path_.push_back(pose);
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

std::vector<Eigen::Vector3d> MincoPlanner::extractLocalPath(const Eigen::Vector3d& cur_pos)
{
  std::vector<Eigen::Vector3d> local_segment;
  std::lock_guard<std::mutex> lock(path_mutex_);

  if (latest_global_path_.empty()) {
    return local_segment;
  }

  // 1. Find the nearest point on the global path
  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i < latest_global_path_.size(); ++i) {
    const auto& pt = latest_global_path_[i].pose.position;
    double dist_sq = (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + 
                     (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // 2. Extract path from nearest point up to lookahead_dist_
  double accum_dist = 0.0;
  
  // Add nearest point
  local_segment.push_back(Eigen::Vector3d(
    latest_global_path_[start_idx].pose.position.x,
    latest_global_path_[start_idx].pose.position.y,
    0.0
  ));

  for (size_t i = start_idx + 1; i < latest_global_path_.size(); ++i) {
    const auto& p1 = latest_global_path_[i - 1].pose.position;
    const auto& p2 = latest_global_path_[i].pose.position;
    
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    accum_dist += dist;

    local_segment.push_back(Eigen::Vector3d(p2.x, p2.y, 0.0));

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

bool MincoPlanner::ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (!costmap_ || !minco_optimizer_) {
    return false;
  }

  // Snapshot the global goal for end-state logic.
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (latest_global_path_.empty()) {
      return false;
    }
    global_goal.x() = latest_global_path_.back().pose.position.x;
    global_goal.y() = latest_global_path_.back().pose.position.y;
    global_goal.z() = 0.0;
  }
  
  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  // 2. Extract local dense path
  std::vector<Eigen::Vector3d> dense_local_path = extractLocalPath(cur_pos);
  if (dense_local_path.size() < 2) {
    return false;
  }

  // 3. Sparsify local path
  std::vector<Eigen::Vector3d> sparse_path = getSparseWaypoints(dense_local_path);
  
  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  // 4. Determine state (HOT/COLD)
  PlanningState state = PlanningState::COLD_START;
  geometry_utils::Trajectory last_traj_snapshot;
  bool has_last_traj_snapshot = false;
  double last_traj_start_WT = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = determinePlanningState(current_pose.pose, sparse_path);
    if (has_last_traj_) {
      last_traj_snapshot = last_traj_;
      has_last_traj_snapshot = true;
      last_traj_start_WT = last_traj_.start_WT;
    }
  }
  
  // 5. Prepare Start State
  Eigen::Matrix3d start_state;
  vec_Vec3f shifted_waypoints;
  VecDf shifted_durations;
  bool has_shifted_seed = false;
  if (state == PlanningState::HOT_START) {
    const double now = rclcpp::Clock().now().seconds() + 0.005; // small buffer
    const double t_dur = now - last_traj_start_WT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      prepareHotStart(current_pose.pose, t_dur, start_state);
    }

    // Extract remaining trajectory segment as shifted warm-start seed.
    if (has_last_traj_snapshot) {
      geometry_utils::Trajectory remain;
      const double total = last_traj_snapshot.getTotalDuration();
      if (std::isfinite(t_dur) && t_dur > 0.0 && total > t_dur + 1e-3 &&
          last_traj_snapshot.getPartialTrajectoryByTime(t_dur, total, remain)) {
        shifted_waypoints = remain.getWaypoints();
        shifted_durations = remain.getDurations();
        has_shifted_seed = (!shifted_waypoints.empty() && shifted_durations.size() > 0);
      }
    }
  } else {
    prepareColdStart(current_pose.pose, start_state);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }

  // 6. Generate Backup Trajectory (Safety)
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  publishBackupTrajectory(backup_traj, header_msg, 20, 0.1);

  // 7. Prepare MINCO Optimization
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

  // End State Logic:
  const double dist_to_goal = (end_state.col(0) - global_goal).head<2>().norm();
  if (dist_to_goal > 1.0) {
    Eigen::Vector3d tangent(1.0, 0.0, 0.0);
    if (sparse_path.size() >= 2) {
      tangent = sparse_path.back() - sparse_path[sparse_path.size() - 2];
      tangent.z() = 0.0;
      const double n = tangent.head<2>().norm();
      if (n > 1e-6) {
        tangent /= n;
      } else {
        tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
    }
    const double v_cmd = 0.8 * std::max(0.0, minco_config.max_vel);
    end_state.col(1) = tangent * v_cmd;
    end_state.col(2).setZero();
  } else {
    end_state.col(1).setZero();
    end_state.col(2).setZero();
  }

  // Remove near-start redundant points from sparse_path
  while (sparse_path.size() > 2)
  {
      if ((sparse_path[1] - start_state.col(0)).norm() < 0.2)
      {
          sparse_path.erase(sparse_path.begin() + 1);
      }
      else
      {
          break;
      }
  }

  // 7.5 Shifted hot-start (reuse remaining Ps/Ts, extend tail with cold init).
  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int N = static_cast<int>(sparse_path.size()) - 1;
    if (N > 0) {
      VecDf init_ts(N);
      const double speed = std::max(1e-3, 0.8 * std::max(0.0, minco_config.max_vel));
      for (int i = 0; i < N; ++i) {
        const double dis = (sparse_path[static_cast<size_t>(i + 1)] -
                            sparse_path[static_cast<size_t>(i)]).head<2>().norm();
        init_ts(i) = std::max(0.1, dis / speed);
      }

      const int oldN = std::min(N, static_cast<int>(shifted_durations.size()));
      for (int i = 0; i < oldN; ++i) {
        const double t = shifted_durations(i);
        if (std::isfinite(t) && t > 0.02) {
          init_ts(i) = t;
        }
      }

      vec_Vec3f init_ps;
      init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
      const int oldWp = static_cast<int>(shifted_waypoints.size());
      const int oldPs = std::max(0, oldWp - 2);
      const int copyPs = std::min(std::max(0, N - 1), oldPs);
      for (int j = 0; j < (N - 1); ++j) {
        if (j < copyPs) {
          init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
        } else {
          init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
        }
      }

      minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
    }
  }

  // 8. Optimize
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double cost = minco_optimizer_->optimize(sparse_path, start_state, end_state, opt_traj);
  
  if (std::isinf(cost)) {
    std::cout << RED << "[MincoPlanner] Minco optimization failed!" << RESET << std::endl;
    return false;
  }
  auto opt_end_time = rclcpp::Clock().now().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  std::cout << GREEN << "[MincoPlanner] Minco optimization time: "
            << opt_duration << " seconds, "
            << "cost: " << cost << RESET << std::endl;

  // 8.5 Post-optimization safety check: reject slight wall-penetration.
  if (!checkCollision(opt_traj)) {
    std::cout << YELLOW
              << "[MincoPlanner] Post-check failed: optimized trajectory collides (penetration). Rejecting."
              << RESET << std::endl;
    is_traj_safe_.store(false);
    return false;
  }
  // 9. Publish and Cache
  const double t_step = 0.05;
  int steps = static_cast<int>(std::ceil(opt_traj.getTotalDuration() / t_step)) + 1;
  steps = std::max(2, steps);
  
  publishOptimizedTrajectory(opt_traj, header_msg, steps, t_step);

  if (visualizer_) {
    nav_msgs::msg::Path astar_path_msg;
    {
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      astar_path_msg.header.stamp = rclcpp::Clock().now();
      astar_path_msg.header.frame_id = global_frame_;
      astar_path_msg.poses = latest_global_path_;
    }
    visualizer_->update(sparse_path, backup_traj, opt_traj, opt_duration, astar_path_msg);
  }
  last_traj_ = opt_traj;
  last_traj_.start_WT = rclcpp::Clock().now().seconds();
  has_last_traj_ = true;

  is_traj_safe_.store(true);
  return true;
}

bool MincoPlanner::checkCollision()
{
  if (!costmap_) {
    return true;
  }

  // Snapshot trajectory under mutex to avoid data races with ReplanLocal().
  geometry_utils::Trajectory traj_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_) {
      return true;
    }
    traj_snapshot = last_traj_;
  }

  // Sample along the committed trajectory.
  const double dur = traj_snapshot.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  const double dt = 0.05;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  for (double t = 0.0; t <= dur; t += dt) {
    const Eigen::Vector3d pos = traj_snapshot.getPos(t);
    unsigned int mx, my;
    if (!costmap_->worldToMap(pos.x(), pos.y(), mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }

  return true;
}

bool MincoPlanner::checkCollision(const geometry_utils::Trajectory & traj)
{
  if (!costmap_) {
    return true;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  // Dense sampling to catch slight obstacle penetration.
  const double dt = 0.02;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  for (double t = 0.0; t <= dur; t += dt) {
    const Eigen::Vector3d pos = traj.getPos(t);
    unsigned int mx, my;
    if (!costmap_->worldToMap(pos.x(), pos.y(), mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }

  return true;
}

void MincoPlanner::safetyTimerCallback()
{
  const bool safe = checkCollision();
  if (!safe) {
    is_traj_safe_.store(false);
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000, "[MincoPlanner] Trajectory collision detected.");
    }
    return;
  }
  is_traj_safe_.store(true);
}

void MincoPlanner::publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose)
{
  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  Eigen::Matrix3d start_state;
  prepareColdStart(current_pose.pose, start_state);

  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  publishBackupTrajectory(backup_traj, header_msg, 20, 0.1);
}

std::vector<Eigen::Vector3d> MincoPlanner::getSparseWaypoints(const std::vector<Eigen::Vector3d>& path) {
  std::vector<Eigen::Vector3d> sparse;
  if (path.empty()) {
    return sparse;
  }

  sparse.push_back(path.front());
  if (path.size() < 2) {
    return sparse;
  }
  if (path.size() == 2) {
    sparse.push_back(path.back());
    return sparse;
  }

  // 1) Build arc-length mapping
  std::vector<double> accumulated_dist;
  accumulated_dist.resize(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    double seg_len = (path[i] - path[i - 1]).head<2>().norm();
    
    // Over-curvature penalty
    if (i > 1) {
        Eigen::Vector3d v1 = path[i] - path[i-1];
        Eigen::Vector3d v2 = path[i-1] - path[i-2];
        if (v1.norm() > 1e-3 && v2.norm() > 1e-3) {
            double dot = v1.normalized().dot(v2.normalized());
            if (dot < 0.7) {
                seg_len *= 2.0;
            }
        }
    }
    accumulated_dist[i] = accumulated_dist[i - 1] + seg_len;
  }
  const double total_length = accumulated_dist.back();
  if (!(std::isfinite(total_length) && total_length > 1e-3)) {
    sparse.push_back(path.back());
    return sparse;
  }

  // 2) Heuristic trapezoid / triangle velocity profile
  const double v_ref = 0.8 * std::max(0.0, minco_config.max_vel);
  const double a_ref = std::max(1e-6, minco_config.max_acc);

  if (v_ref <= 1e-6) {
    sparse.push_back(path.back());
    return sparse;
  }

  const double d_acc_ref = v_ref * v_ref / (2.0 * a_ref);
  double v_peak = v_ref;
  double t_acc = v_ref / a_ref;
  double t_flat = 0.0;
  if (total_length > 2.0 * d_acc_ref) {
    const double d_flat = total_length - 2.0 * d_acc_ref;
    t_flat = d_flat / v_ref;
  } else {
    v_peak = std::sqrt(std::max(0.0, total_length * a_ref));
    t_acc = v_peak / a_ref;
    t_flat = 0.0;
  }

  const double t_total = 2.0 * t_acc + t_flat;
  if (!(std::isfinite(t_total) && t_total > 1e-6)) {
    sparse.push_back(path.back());
    return sparse;
  }

  auto arcLengthToIndex = [&accumulated_dist](double s) -> size_t {
    const double s_clamped = clampValue(s, 0.0, accumulated_dist.back());
    auto it = std::lower_bound(accumulated_dist.begin(), accumulated_dist.end(), s_clamped);
    if (it == accumulated_dist.begin()) {
      return 0u;
    }
    if (it == accumulated_dist.end()) {
      return accumulated_dist.size() - 1u;
    }
    const size_t idx1 = static_cast<size_t>(std::distance(accumulated_dist.begin(), it));
    const size_t idx0 = idx1 - 1u;
    const double s0 = accumulated_dist[idx0];
    const double s1 = accumulated_dist[idx1];
    if (!(std::isfinite(s0) && std::isfinite(s1)) || (s1 - s0) <= 1e-12) {
      return idx1;
    }
    return ((s_clamped - s0) < (s1 - s_clamped)) ? idx0 : idx1;
  };

  // Helper: find a corner index within (start_idx, end_idx) by max perpendicular distance
  // to the straight segment (start -> end) in 2D.
  auto findCornerIndex = [&path](size_t start_idx, size_t end_idx) -> size_t {
    if (end_idx <= start_idx + 1u) {
      return start_idx;
    }

    const Eigen::Vector2d a = path[start_idx].head<2>();
    const Eigen::Vector2d b = path[end_idx].head<2>();
    const Eigen::Vector2d ab = b - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 <= 1e-12) {
      return (start_idx + end_idx) / 2u;
    }

    double best_dist2 = -1.0;
    size_t best_k = (start_idx + end_idx) / 2u;
    for (size_t k = start_idx + 1u; k < end_idx; ++k) {
      const Eigen::Vector2d p = path[k].head<2>();
      const double t = clampValue((p - a).dot(ab) / ab2, 0.0, 1.0);
      const Eigen::Vector2d proj = a + t * ab;
      const double dist2 = (p - proj).squaredNorm();
      if (dist2 > best_dist2) {
        best_dist2 = dist2;
        best_k = k;
      }
    }
    return best_k;
  };

  // 3) Build Ideal Indices by time-uniform sampling in trapezoid time, then s(t)->raw index.
  int n_segments = static_cast<int>(std::ceil(t_total / 0.25));
  n_segments = std::max(4, std::min(6, n_segments));
  const double dt = t_total / static_cast<double>(n_segments);

  std::vector<size_t> target_indices;
  target_indices.reserve(static_cast<size_t>(n_segments + 1));
  target_indices.push_back(0u);

  size_t last_added = 0u;
  for (int i = 1; i < n_segments; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double s = getDistFromTrapezoid(t, total_length, a_ref, v_peak, t_acc, t_flat);
    size_t idx = arcLengthToIndex(s);
    // enforce strictly increasing indices to preserve ordering and avoid duplicates
    idx = std::max(idx, last_added);
    if (idx == last_added) {
      if (idx + 1u < path.size()) {
        idx = idx + 1u;
      }
    }
    if (idx > last_added && idx < path.size()) {
      target_indices.push_back(idx);
      last_added = idx;
    }
  }

  const size_t last_idx = path.size() - 1u;
  if (target_indices.empty() || target_indices.back() != last_idx) {
    target_indices.push_back(last_idx);
  }

  // 4) Safety Verification & Repair
  size_t current_safe_idx = 0u;
  for (size_t ti = 1; ti < target_indices.size(); ++ti) {
    const size_t target_idx = target_indices[ti];
    if (target_idx <= current_safe_idx || target_idx >= path.size()) {
      continue;
    }

    // Try to connect current_safe_idx -> target_idx; if collision, insert corner(s).
    size_t guard = 0u;
    while (guard++ < 32u && target_idx > current_safe_idx) {
      if (isLineFree(path[current_safe_idx], path[target_idx])) {
        Eigen::Vector3d p = path[target_idx];
        p.z() = 0.0;
        if ((p - sparse.back()).head<2>().norm() > 1e-6) {
          sparse.push_back(p);
        }
        current_safe_idx = target_idx;
        break;
      }

      // Collision: recover a corner point inside (current_safe_idx, target_idx)
      size_t corner_idx = findCornerIndex(current_safe_idx, target_idx);
      if (corner_idx <= current_safe_idx || corner_idx >= target_idx) {
        // Fallback: force progress by inserting the next raw point.
        corner_idx = current_safe_idx + 1u;
        if (corner_idx >= target_idx) {
          // Worst-case: cannot progress further, just stop trying this target.
          break;
        }
      }

      Eigen::Vector3d corner = path[corner_idx];
      corner.z() = 0.0;
      if ((corner - sparse.back()).head<2>().norm() > 1e-6) {
        sparse.push_back(corner);
      }
      current_safe_idx = corner_idx;
    }
  }

  // Ensure goal is included
  if ((path.back() - sparse.back()).head<2>().norm() > 1e-6) {
    Eigen::Vector3d goal = path.back();
    goal.z() = 0.0;
    sparse.push_back(goal);
  }

  return sparse;
}

bool MincoPlanner::isLineFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
  // 1. 用 costmap 栅格做直线离散采样碰撞检查（用于路径抽稀/走廊生成等）
  //    1.1 以分辨率为步长采样线段
  //    1.2 任一点落入膨胀障碍（>= INSCRIBED）则判为不可直连
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
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

MincoPlanner::PlanningState MincoPlanner::determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose,
    const std::vector<Eigen::Vector3d> & new_path)
{
    // 1. Check history availability
    if (!has_last_traj_) {
        return PlanningState::COLD_START;
    }

    // 2. Check time validity
    double now = rclcpp::Clock().now().seconds() + 0.03;
    double t_dur = now - last_traj_.start_WT;
    if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
        std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Invalid time duration (t_dur="
                  << t_dur << "s)" << RESET << std::endl;
        return PlanningState::COLD_START;
    }

    // 3. Check position consistency
    Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y, 0.0);
    Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
    double tracking_error = (current_pos - pred_pos).norm();
    
    // Error threshold
    if (tracking_error > 0.5) {
      std::cout << YELLOW << "[MincoPlanner] EMERGENCY_STOP: Large tracking error (" << tracking_error << "m)" << RESET << std::endl;
      return PlanningState::EMERGENCY_STOP;
    }

    // 4. Check direction consistency
    if (new_path.size() >= 2) {
        Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
      // Skip direction check when speed is low
        if (pred_vel.norm() > 0.1) {
            Eigen::Vector3d path_dir = (new_path[1] - new_path[0]).normalized();
            Eigen::Vector3d vel_dir = pred_vel.normalized();
            double dot = vel_dir.dot(path_dir);

        // Reject if the heading changes too much
            if (dot < 0.9) {
                std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Direction mismatch (dot=" << dot
                          << ", angle=" << std::acos(dot) * 180.0 / M_PI << " deg)" << RESET << std::endl;
                return PlanningState::COLD_START;
            }
        }
    }

    return PlanningState::HOT_START;
}

void MincoPlanner::prepareColdStart(
    const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state)
{
    start_state.setZero();
    start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
}

void MincoPlanner::prepareHotStart(
    const geometry_msgs::msg::Pose & /*start_pose*/,
    double t_dur,
    Eigen::Matrix3d & start_state)
{
    start_state.setZero();
    start_state.col(0) = last_traj_.getPos(t_dur);
    start_state.col(1) = last_traj_.getVel(t_dur);
    start_state.col(2) = last_traj_.getAcc(t_dur);
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
