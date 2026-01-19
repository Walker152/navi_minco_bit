#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>
#include <stdexcept>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace minco_planner
{
using namespace color_text;

void MincoPlanner::publishEsdfCloud(const std_msgs::msg::Header & header)
{
  if (!esdf_cloud_pub_ || !esdf_map_) {
    return;
  }

  // Visualize fused ESDF (static + dynamic). We sample on the dynamic layer grid if available,
  // otherwise fall back to the static layer grid.
  int width = 0;
  int height = 0;
  double res = 0.0;
  Eigen::Vector2d origin(0.0, 0.0);

  const auto dynamic_layer = esdf_map_->dynamicLayer();
  if (dynamic_layer && dynamic_layer->isValid()) {
    width = dynamic_layer->width();
    height = dynamic_layer->height();
    res = dynamic_layer->resolution();
    origin = dynamic_layer->origin();
  } else {
    const auto static_layer = esdf_map_->staticLayer();
    if (!static_layer || !static_layer->isValid()) {
      return;
    }
    width = static_layer->width();
    height = static_layer->height();
    res = static_layer->resolution();
    origin = static_layer->origin();
  }

  if (width <= 0 || height <= 0) {
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

  size_t point_index = 0;
  for (int iy = 0; iy < height; ++iy)
  {
    for (int ix = 0; ix < width; ++ix, ++point_index)
    {
      const float x = static_cast<float>(origin.x() + ix * res);
      const float y = static_cast<float>(origin.y() + iy * res);
      const float z = 0.0f;
      double dist = 0.0;
      Eigen::Vector3d grad;
      esdf_map_->evaluate(Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), 0.0), dist, grad);
      if (!std::isfinite(dist)) {
        dist = 0.0;
      }
      const float intensity = static_cast<float>(dist);

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
  backup_path_pub_->publish(traj_msg);
}

nav_msgs::msg::Path MincoPlanner::convertTrajectoryToPath(
  const traj_opt::Trajectory & traj,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step) const
{
  nav_msgs::msg::Path path_msg;
  path_msg.header = header;

  if (steps <= 0 || t_step <= 0.0) {
    return path_msg;
  }

  path_msg.poses.resize(static_cast<size_t>(steps));

  const double total_duration = traj.getTotalDuration();
  for (int i = 0; i < steps; ++i)
  {
    double t = i * t_step;
    if (t > total_duration) {
      t = total_duration;
    }

    Eigen::Vector3d pos = traj.getPos(t);
    Eigen::Vector3d vel = traj.getVel(t);

    pos.z() = 0.0;
    vel.z() = 0.0;

    double yaw = 0.0;
    if (vel.head<2>().norm() > 1e-4) {
      yaw = std::atan2(vel(1), vel(0));
    } else if (i > 0) {
      const auto & last_q = path_msg.poses[static_cast<size_t>(i - 1)].pose.orientation;
      yaw = 2.0 * std::atan2(last_q.z, last_q.w);
    }

    auto & pose = path_msg.poses[static_cast<size_t>(i)];
    pose.header = header;
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
  }

  return path_msg;
}

void MincoPlanner::updateVisCache(
  const std::vector<Eigen::Vector3d> & control_points,
  const traj_opt::Trajectory & backup_traj,
  const traj_opt::Trajectory & opt_traj,
  double opt_time)
{
  nav_msgs::msg::Path astar_path_msg;
  {
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    astar_path_msg.header.stamp = rclcpp::Clock().now();
    astar_path_msg.header.frame_id = global_frame_;
    astar_path_msg.poses = latest_global_path_;
  }

  std::lock_guard<std::mutex> vis_lock(vis_mutex_);
  vis_control_points_ = control_points;
  vis_backup_traj_ = backup_traj;
  has_vis_backup_traj_ = (backup_traj.getTotalDuration() > 1e-3);

  vis_opt_traj_ = opt_traj;
  vis_opt_time_ = opt_time;
  has_vis_opt_traj_ = (opt_traj.getTotalDuration() > 1e-3);

  vis_astar_path_ = astar_path_msg;
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

  // Vis publishers
  opt_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/opt_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  backup_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/backup_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  astar_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/astar_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  control_points_vis_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "/minco_control_points_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  esdf_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/esdf_cloud", 10);

  // Load Static ESDF Map
  esdf_map_ = std::make_shared<small_rog_map::HybridESDFMap>();

  // Initialize ESDF dynamic layer ROS subscription (STVL voxel_grid).
  esdf_map_->initRos(parent, "/global_costmap/voxel_grid");

  if (!esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_)) {
    std::cout << RED << "[MincoPlanner] "
              << "Failed to load Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
  } else {
    std::cout << MAGENTA << "[MincoPlanner] "
              << "Successfully loaded Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
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
  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>(esdf_map_);
  backup_opt_ = std::make_unique<traj_opt::BackupTrajOpt>();
  minco_optimizer_->setESDFMap(esdf_map_); 
  opt_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / opt_freq_),
    std::bind(&MincoPlanner::optimizationTimerCallback, this));
  visual_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(66), // 15Hz
      std::bind(&MincoPlanner::visualTimerCallback, this));
}

void MincoPlanner::cleanup()
{
  visual_timer_.reset();
  opt_timer_.reset();
  esdf_timer_.reset();

  astar_planner_.reset();
  minco_optimizer_.reset();
  backup_opt_.reset();
  opt_path_pub_.reset();
  backup_path_pub_.reset();

  opt_path_vis_pub_.reset();
  backup_path_vis_pub_.reset();
  astar_path_vis_pub_.reset();
  control_points_vis_pub_.reset();
  esdf_cloud_pub_.reset();
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
  {
    static auto start = rclcpp::Clock().now();
    static int count = 0;
    ++count;
    if (count % 100 == 0) {
      auto now = rclcpp::Clock().now();
      double duration = (now - start).seconds();
      std::cout << BLUE << "[MincoPlanner] createPlan called " << count
                << " times over " << duration << " seconds. Avg rate: "
                << static_cast<double>(count) / duration << " Hz." << RESET << std::endl;
      start = now;
      count = 0;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // 1. Initialize the plan message
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  // We use rclcpp::ok() as a basic check.
  auto cancel_checker = []() {
    return !rclcpp::ok();
  };

  if (!astar_planner_ || !costmap_) {
    std::cout << RED << "MincoPlanner: planner is not properly configured" << RESET << std::endl;
    throw std::runtime_error("MincoPlanner: planner is not properly configured");
  }

  if (!minco_optimizer_) {
    std::cout << RED << "MincoPlanner: minco_optimizer is not initialized" << RESET << std::endl;
    throw std::runtime_error("MincoPlanner: minco_optimizer is not initialized");
  }

  // 2. convert start and goal to global frame
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

  // 3. Handle the trivial case (start == goal)
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = start.pose;
    path.poses.push_back(pose);
    return path;
  }

  // 4. Run the main planning pipeline
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
  // 1. Reset plan output and header
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  // 2. Search a discrete guide path using A*
  // Convert world coords to map coords
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


  
  std::cout << GREEN << "[MincoPlanner] Successfully created global plan (A* only) with " << plan.poses.size()
            << " waypoints." << RESET << std::endl;

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

void MincoPlanner::optimizationTimerCallback()
{
  if (!costmap_ros_ || latest_global_path_.empty()) {
    return;
  }

  // 1. Get current robot pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return;
  }
  
  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  // 2. Extract local dense path
  std::vector<Eigen::Vector3d> dense_local_path = extractLocalPath(cur_pos);
  if (dense_local_path.size() < 2) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // 3. Sparsify local path
  std::vector<Eigen::Vector3d> sparse_path = getSparseWaypoints(dense_local_path);
  
  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  // 4. Determine state (HOT/COLD)
  PlanningState state = determinePlanningState(current_pose.pose, sparse_path);
  
  // 5. Prepare Start State
  Eigen::Matrix3d start_state;
  if (state == PlanningState::HOT_START) {
    double now = rclcpp::Clock().now().seconds() + 0.005; // small buffer
    double t_dur = now - last_traj_.start_WT;
    prepareHotStart(current_pose.pose, t_dur, start_state);
  } else {
    prepareColdStart(current_pose.pose, start_state);
  }

  // 6. Generate Backup Trajectory (Safety)
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  publishBackupTrajectory(backup_traj, header_msg, 20, 0.1);

  // 7. Prepare MINCO Optimization
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

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

  // 8. Optimize
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double cost = minco_optimizer_->optimize(sparse_path, start_state, end_state, opt_traj);
  
  if (std::isinf(cost)) {
    std::cout << RED << "[MincoPlanner] Minco optimization failed!" << RESET << std::endl;
    return;
  }
  auto opt_end_time = rclcpp::Clock().now().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  std::cout << GREEN << "[MincoPlanner] Minco optimization time: "
            << opt_duration << " seconds, "
            << "cost: " << cost << RESET << std::endl;
  // 9. Publish and Cache
  const double t_step = 0.05;
  int steps = static_cast<int>(std::ceil(opt_traj.getTotalDuration() / t_step)) + 1;
  steps = std::max(2, steps);
  
  publishOptimizedTrajectory(opt_traj, header_msg, steps, t_step);

  updateVisCache(sparse_path, backup_traj, opt_traj, opt_duration);
  last_traj_ = opt_traj;
  last_traj_.start_WT = rclcpp::Clock().now().seconds();
  has_last_traj_ = true;
}

void MincoPlanner::visualTimerCallback()
{
  std_msgs::msg::Header header;
  auto node = node_.lock();
  if (node) {
    header.stamp = node->now();
    header.frame_id = global_frame_;
  } else {
    return;
  }

  std::lock_guard<std::mutex> lock(vis_mutex_);

  // 1. A* Path
  if (astar_path_vis_pub_ && !vis_astar_path_.poses.empty()) {
    vis_astar_path_.header = header;
    for (auto & p : vis_astar_path_.poses) {
      p.header = header;
    }
    astar_path_vis_pub_->publish(vis_astar_path_);
  }

  // 2. Control Points
  if (control_points_vis_pub_ && !vis_control_points_.empty()) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "minco_control_points";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.25; mk.scale.y = 0.25; mk.scale.z = 0.25;
    mk.color.r = 1.0f; mk.color.g = 0.55f; mk.color.b = 0.0f; mk.color.a = 1.0f;

    mk.points.reserve(vis_control_points_.size());
    for (const auto & p : vis_control_points_) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x(); pt.y = p.y(); pt.z = 0.05;
      mk.points.push_back(pt);
    }
    control_points_vis_pub_->publish(mk);
  }

  // 3. Backup Path
  if (backup_path_vis_pub_ && has_vis_backup_traj_ && vis_backup_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_backup_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_backup_traj_, header, steps, t_step);
    backup_path_vis_pub_->publish(path_msg);
  }
  
  // 4. Optimized Path & Time
  if (opt_path_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_opt_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_opt_traj_, header, steps, t_step);
    opt_path_vis_pub_->publish(path_msg);
  }

  if (control_points_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3 && vis_opt_time_ > 0.0) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "opt_time";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mk.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d start_pos = vis_opt_traj_.getPos(0.0);
    mk.pose.position.x = start_pos(0);
    mk.pose.position.y = start_pos(1);
    mk.pose.position.z = 1.0;
    mk.pose.orientation.w = 1.0;
    mk.scale.z = 0.3;
    mk.color.r = 1.0f; mk.color.g = 1.0f; mk.color.b = 0.0f; mk.color.a = 1.0f;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << (vis_opt_time_ * 1000.0) << " ms";
    mk.text = ss.str();
    control_points_vis_pub_->publish(mk);
  }
}

std::vector<Eigen::Vector3d> MincoPlanner::getSparseWaypoints(const std::vector<Eigen::Vector3d>& path) {
  // 1. Handle degenerate inputs
    std::vector<Eigen::Vector3d> sparse;
    if (path.empty()) return sparse;
    
    sparse.push_back(path.front());
    if (path.size() < 3) {
        sparse.push_back(path.back());
        return sparse;
    }
    
    // 2. Greedy sparsify using line-of-sight checks
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

    // 3. Merge very close points and keep the final goal
    std::vector<Eigen::Vector3d> final_sparse;
    final_sparse.push_back(sparse.front());
    
    for (size_t i = 1; i < sparse.size() - 1; ++i) {
        const auto& last_pt = final_sparse.back();
        const auto& curr_pt = sparse[i];
        const auto& next_pt = sparse[i+1];
        
        double dist = (curr_pt - last_pt).norm();
        
           // Skip close points if the direct segment is collision-free
        if (dist < 1.0 && isLineFree(last_pt, next_pt)) {
             continue;
        }
        
        final_sparse.push_back(curr_pt);
    }
    
    // Always keep the goal
    final_sparse.push_back(sparse.back());
    return final_sparse;
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
    // Vel and Acc are implicitly zero
}

void MincoPlanner::prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose,
    double t_dur,
    Eigen::Matrix3d & start_state)
{
    start_state.setZero();
    start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
    
    start_state.col(1) = last_traj_.getVel(t_dur);
    start_state.col(2) = last_traj_.getAcc(t_dur);
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
