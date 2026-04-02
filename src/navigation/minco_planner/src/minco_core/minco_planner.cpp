// Corresponding header
#include "minco_core/minco_planner.hpp"

// C++ standard library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

// Third-party
#include <Eigen/Core>

// ROS 2
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"

// Project
#include "minco_core/minco_fsm.hpp"
#include "minco_core/minco_utils.hpp"
#include "minco_core/visualizer.hpp"

namespace minco_planner {

using namespace color_text;


MincoPlanner::MincoPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

MincoPlanner::~MincoPlanner() = default;

// -----------------------------------------------------------------------------
// 2) Lifecycle management
// -----------------------------------------------------------------------------

void MincoPlanner::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID(); // map

  auto node = parent.lock();
  logger_ = node->get_logger();

  const std::string prefix = name_ + ".";

  // --- General config --------------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "tolerance", tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "use_smac", rclcpp::ParameterValue(false));
  node->get_parameter(prefix + "use_smac", use_smac_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "allow_unknown", allow_unknown_);

  // Odometry topic
  std::string odom_topic = "/odom";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "odom_topic", rclcpp::ParameterValue(odom_topic));
  node->get_parameter(prefix + "odom_topic", odom_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.opt_freq", rclcpp::ParameterValue(20.0));
  node->get_parameter(prefix + "minco_optimizer.opt_freq", opt_freq_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.lookahead_dist", rclcpp::ParameterValue(5.0));
  node->get_parameter(prefix + "minco_optimizer.lookahead_dist", lookahead_dist_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.traj_goal_tolerance", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.traj_goal_tolerance", traj_goal_tolerance_);

  // --- Optimizer config ------------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.safe_dist", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.safe_dist", minco_config.safe_dist);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.max_velocity", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.max_velocity", minco_config.max_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.max_acceleration", rclcpp::ParameterValue(4.0));
  node->get_parameter(prefix + "minco_optimizer.max_acceleration", minco_config.max_acc);

  double max_yaw_dot = 3.14;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.max_yaw_dot", rclcpp::ParameterValue(max_yaw_dot));
  node->get_parameter(prefix + "minco_optimizer.max_yaw_dot", max_yaw_dot);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.enable_yaw_opt", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.enable_yaw_opt", use_yaw_opt_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.time_allocation_iters", rclcpp::ParameterValue(15));
  node->get_parameter(prefix + "minco_optimizer.time_allocation_iters", minco_config.time_allocation_iters);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_time", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time", minco_config.rho);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.smooth_eps", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.smooth_eps", minco_config.smooth_eps);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.integral_res", rclcpp::ParameterValue(16));
  node->get_parameter(prefix + "minco_optimizer.integral_res", minco_config.integral_res);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.opt_accuracy", rclcpp::ParameterValue(1.0e-4));
  node->get_parameter(prefix + "minco_optimizer.opt_accuracy", minco_config.opt_accuracy);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.print_optimizer_log", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.print_optimizer_log", minco_config.print_optimizer_log);

  double penalty_weight_pos = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_pos", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_pos", penalty_weight_pos);

  double penalty_weight_vel = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_vel", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_vel", penalty_weight_vel);

  double penalty_weight_acc = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_acc", rclcpp::ParameterValue(10000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_acc", penalty_weight_acc);

  double penalty_weight_att = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_att", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_att", penalty_weight_att);

  minco_config.penaltyWeights.resize(4);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;

  minco_config.magnitudeBounds.resize(3);
  minco_config.magnitudeBounds(0) = minco_config.safe_dist;
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;

  // --- Map / ESDF config -----------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
    node,
    prefix + "static_esdf.esdf_pcd_path",
    rclcpp::ParameterValue("src/utils/pcd2esdf/maps/2026_esdf.pcd"));
  node->get_parameter(prefix + "static_esdf.esdf_pcd_path", esdf_pcd_path_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "static_esdf.esdf_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "static_esdf.esdf_resolution", esdf_resolution_);

  double dynamic_esdf_size = 10.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "static_esdf.dynamic_esdf_size", rclcpp::ParameterValue(10.0));
  node->get_parameter(prefix + "static_esdf.dynamic_esdf_size", dynamic_esdf_size);

  double dynamic_dilation_radius = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "static_esdf.dynamic_dilation_radius", rclcpp::ParameterValue(0.0));
  node->get_parameter(prefix + "static_esdf.dynamic_dilation_radius", dynamic_dilation_radius);


  // --- Corridor config -------------------------------------------------------

  double corridor_robot_radius = 0.4;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "corridor.robot_radius", rclcpp::ParameterValue(corridor_robot_radius));
  node->get_parameter(prefix + "corridor.robot_radius", corridor_robot_radius);

  double corridor_extra_margin = 0.15;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "corridor.extra_margin", rclcpp::ParameterValue(corridor_extra_margin));
  node->get_parameter(prefix + "corridor.extra_margin", corridor_extra_margin);

  // --- Recovery server config -----------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.fail_threshold", rclcpp::ParameterValue(3));
  node->get_parameter(
    prefix + "recovery_server.fail_threshold",
    recovery_server_config_.fail_threshold);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.cooldown_sec", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "recovery_server.cooldown_sec", recovery_server_config_.cooldown_sec);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.recovery_window_sec", rclcpp::ParameterValue(3.0));
  node->get_parameter(prefix + "recovery_server.recovery_window_sec", recovery_server_config_.recovery_window_sec);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.search_min_dist", rclcpp::ParameterValue(0.2));
  node->get_parameter(prefix + "recovery_server.search_min_dist", recovery_server_config_.search_min_dist);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.search_max_dist", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "recovery_server.search_max_dist", recovery_server_config_.search_max_dist);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.search_step", rclcpp::ParameterValue(0.05));
  node->get_parameter(prefix + "recovery_server.search_step", recovery_server_config_.search_step);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.escape_speed", rclcpp::ParameterValue(0.4));
  node->get_parameter(prefix + "recovery_server.escape_speed", recovery_server_config_.escape_speed);

  // --- Components / publishers / timers -------------------------------------

  astar_planner_ = std::make_unique<Astar>(
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  if (use_smac_) {
    smac_planner_ = std::make_unique<minco_planner::smac::SmacPlanner2DSimple>();
    smac_planner_->configure(node, costmap_ros_, prefix);
    smac_planner_->setParameters(allow_unknown_, 1000000, tolerance_);
  }

  opt_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  backup_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/backup_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!msg) {
        return;
      }

      {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        latest_odom_ = *msg;
        has_latest_odom_ = true;
      }

      geometry_msgs::msg::PoseStamped odom_pose;
      odom_pose.header = msg->header;
      odom_pose.pose = msg->pose.pose;

      try {
        if (tf_) {
          auto map_pose = tf_->transform(odom_pose, global_frame_);
          const double x = map_pose.pose.position.x;
          const double y = map_pose.pose.position.y;
          if (esdf_map_) {
            esdf_map_->setRobotPosition(x, y);
          }
        }
      } catch (const tf2::TransformException &) {
      }
    });

  // Load Static ESDF map.
  esdf_map_ = std::make_shared<small_rog_map::HybridESDFMap>();
  esdf_map_->initRos(
    parent,
    "/global_costmap/voxel_grid",
    esdf_resolution_,
    dynamic_esdf_size,
    dynamic_dilation_radius);

  // Initialize ESDF window center once at startup so static validation works without odom.
  geometry_msgs::msg::PoseStamped init_pose;
  if (costmap_ros_ && costmap_ros_->getRobotPose(init_pose)) {
    esdf_map_->setRobotPosition(init_pose.pose.position.x, init_pose.pose.position.y);
  } else {
    esdf_map_->setRobotPosition(0.0, 0.0);
    RCLCPP_WARN(
      logger_,
      "[MincoPlanner] Failed to get initial robot pose from costmap, fallback ESDF center to (0, 0).");
  }

  const bool esdf_loaded = esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);
  if (!esdf_loaded) {
    std::cout << RED << "[MincoPlanner] "
              << "Failed to load Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
  } else {
    std::cout << MAGENTA << "[MincoPlanner] "
              << "Successfully loaded Static ESDF map from PCD: " << esdf_pcd_path_ << RESET
              << std::endl;
  }

  if (use_smac_ && smac_planner_) {
    smac_planner_->setESDFMap(esdf_map_);
  }

  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, global_frame_, esdf_map_, esdf_loaded);

  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setESDFMap(esdf_map_);

  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>(esdf_map_);
  corridor_gen_->setSafetyMargins(corridor_robot_radius, corridor_extra_margin);

  backup_opt_ = std::make_unique<traj_opt::BackupTrajOpt>();
  yaw_opt_ = std::make_unique<traj_opt::YawTrajOpt>(max_yaw_dot);

  recovery_server_ = std::make_shared<RecoverServer>();
  recovery_server_->configure(recovery_server_config_);

  // Planner handle for FSM (non-owning; lifetime managed by pluginlib).
  planner_handle_ = MincoPlanner::Ptr(this, [](MincoPlanner *) {});

  // High-level FSM @ 20Hz.
  fsm_ = std::make_unique<MincoFsm>(planner_handle_, recovery_server_);
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

  on_set_parameters_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&MincoPlanner::onSetParameters, this, std::placeholders::_1));
}

void MincoPlanner::activate()
{
}

void MincoPlanner::deactivate()
{
}

void MincoPlanner::cleanup()
{
  on_set_parameters_callback_handle_.reset();

  fsm_timer_.reset();
  safety_timer_.reset();

  fsm_.reset();
  recovery_server_.reset();
  planner_handle_.reset();

  if (visualizer_) {
    visualizer_->cleanup();
    visualizer_.reset();
  }

  astar_planner_.reset();
  minco_optimizer_.reset();
  backup_opt_.reset();
  yaw_opt_.reset();
  opt_path_pub_.reset();
  backup_path_pub_.reset();
}

rcl_interfaces::msg::SetParametersResult MincoPlanner::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::string target_param = name_ + ".static_esdf.esdf_pcd_path";
  for (const auto & param : parameters) {
    if (param.get_name() != target_param) {
      continue;
    }

    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      result.successful = false;
      result.reason = "Parameter must be a string: " + target_param;
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    esdf_pcd_path_ = param.as_string();
    if (!esdf_map_) {
      result.successful = false;
      result.reason = "ESDF map is not initialized";
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    const bool reloaded = esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);
    if (reloaded) {
      RCLCPP_INFO(
        logger_,
        "[MincoPlanner] Reloaded static ESDF map from PCD: %s",
        esdf_pcd_path_.c_str());
    } else {
      result.successful = false;
      result.reason = "Failed to reload static ESDF map from PCD: " + esdf_pcd_path_;
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
    }

    return result;
  }

  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

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

bool MincoPlanner::ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  if (!costmap_ || !minco_optimizer_) {
    return false;
  }

  // Snapshot the global goal for end-state logic.
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  double goal_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (latest_global_path_.empty()) {
      return false;
    }
    global_goal.x() = latest_global_path_.back().pose.position.x;
    global_goal.y() = latest_global_path_.back().pose.position.y;
    global_goal.z() = 0.0;
    const auto & q = latest_global_path_.back().pose.orientation;
    goal_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(goal_yaw)) {
      goal_yaw = 0.0;
    }
  }

  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  // 2. Extract local dense path.
  std::vector<Eigen::Vector3d> dense_local_path = extractLocalPath(cur_pos);
  if (dense_local_path.size() < 2) {
    return false;
  }

  // 3. Sparsify local path.
  std::vector<Eigen::Vector3d> sparse_path = utils::getSparseWaypoints(
    dense_local_path,
    minco_config.max_vel,
    minco_config.max_acc,
    [this](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return utils::isLineFree(this->costmap_, a, b);
    });

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  // 4. Determine state (HOT/COLD).
  PlanningState state = PlanningState::COLD_START;
  traj_opt::Trajectory last_traj_snapshot;
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

  if (state == PlanningState::EMERGENCY_STOP) {
    return false;
  }

  // 5. Prepare start state.
  Eigen::Matrix3d start_state;
  vec_Vec3f shifted_waypoints;
  VecDf shifted_durations;
  bool has_shifted_seed = false;
  if (state == PlanningState::HOT_START) {
    const double now = rclcpp::Clock().now().seconds() + 0.005;  // small buffer
    const double t_dur = now - last_traj_start_WT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      prepareHotStart(current_pose.pose, t_dur, start_state);
    }

    // Extract remaining trajectory segment as shifted warm-start seed.
    if (has_last_traj_snapshot) {
      traj_opt::Trajectory remain;
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

  // 6. Generate backup trajectory (safety).
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);

  // 7. Prepare MINCO optimization.
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

  // End state logic.
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

  // Remove near-start redundant points from sparse_path.
  while (sparse_path.size() > 2) {
    if ((sparse_path[1] - start_state.col(0)).norm() < 0.2) {
      sparse_path.erase(sparse_path.begin() + 1);
    } else {
      break;
    }
  }

  // 7.5 Initial guess Ps/Ts for optimizer (all cases).
  {
    const int N = static_cast<int>(sparse_path.size()) - 1;
    if (N > 0) {
      const double vmax = std::max(0.0, minco_config.max_vel);
      const double amax = std::max(1e-3, minco_config.max_acc);
      const double kMinSegTime = 0.1;
      const double kBrakeSafety = 1.2;

      // Build init control points (Ps). Use shifted seed when available.
      vec_Vec3f init_ps;
      init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
      int copyPs = 0;
      if (state == PlanningState::HOT_START && has_shifted_seed) {
        const int oldWp = static_cast<int>(shifted_waypoints.size());
        const int oldPs = std::max(0, oldWp - 2);
        copyPs = std::min(std::max(0, N - 1), oldPs);
      }
      for (int j = 0; j < (N - 1); ++j) {
        if (j < copyPs) {
          init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
        } else {
          init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
        }
      }

      // Kinematics-aware time allocation.
      double v_curr = start_state.col(1).norm();
      if (!std::isfinite(v_curr) || v_curr < 0.0) {
        v_curr = 0.0;
      }

      std::vector<double> seg_len;
      seg_len.resize(static_cast<size_t>(N), 0.0);
      for (int i = 0; i < N; ++i) {
        const double dis = (sparse_path[static_cast<size_t>(i + 1)] -
                            sparse_path[static_cast<size_t>(i)])
                             .head<2>()
                             .norm();
        seg_len[static_cast<size_t>(i)] = (std::isfinite(dis) && dis > 0.0) ? dis : 0.0;
      }

      VecDf init_ts(N);
      for (int i = 0; i < N; ++i) {
        const bool is_last = (i == N - 1);
        const double L = seg_len[static_cast<size_t>(i)];

        // Remaining distance after this segment (for stop feasibility capping).
        double remain_after = 0.0;
        for (int k = i + 1; k < N; ++k) {
          remain_after += seg_len[static_cast<size_t>(k)];
        }

        if (L <= 1e-6) {
          init_ts(i) = kMinSegTime;
          continue;
        }

        if (is_last) {
          const double t_stop = v_curr / amax;
          const double t_dist = L / std::max(v_curr, 0.1);
          init_ts(i) = std::max({kMinSegTime, t_dist, kBrakeSafety * t_stop});
          v_curr = 0.0;
          continue;
        }

        // Predict reachable speed at end of this segment under accel limits.
        double v_next = std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * amax * L));
        if (std::isfinite(vmax) && vmax > 0.0) {
          v_next = std::min(v_next, vmax);
        }

        // Cap speed to ensure it can stop within remaining path length.
        if (remain_after > 1e-6) {
          const double v_cap_stop = std::sqrt(std::max(0.0, 2.0 * amax * remain_after));
          v_next = std::min(v_next, v_cap_stop);
        } else {
          v_next = 0.0;
        }

        if (!std::isfinite(v_next) || v_next < 0.0) {
          v_next = 0.0;
        }

        double t = 0.0;
        const double v_sum = v_curr + v_next;
        if (v_sum > 1e-3) {
          t = 2.0 * L / v_sum;
        } else {
          t = L / 0.1;
        }

        // If saturating at vmax and still long, include cruise time.
        if (vmax > 1e-6 && std::abs(v_next - vmax) < 1e-6 && v_curr < vmax - 1e-6) {
          const double d_acc = (vmax * vmax - v_curr * v_curr) / (2.0 * amax);
          if (std::isfinite(d_acc) && d_acc > 0.0 && L > d_acc) {
            const double t_acc = (vmax - v_curr) / amax;
            const double t_cruise = (L - d_acc) / vmax;
            const double t_alt = t_acc + t_cruise;
            if (std::isfinite(t_alt) && t_alt > 0.0) {
              t = t_alt;
            }
          }
        }

        if (!std::isfinite(t) || t < kMinSegTime) {
          t = kMinSegTime;
        }

        init_ts(i) = t;
        v_curr = v_next;
      }

      // If we have shifted seed durations, keep them only as a lower bound.
      if (state == PlanningState::HOT_START && has_shifted_seed) {
        const int oldN = std::min(N, static_cast<int>(shifted_durations.size()));
        for (int i = 0; i < oldN; ++i) {
          const double t_seed = shifted_durations(i);
          if (std::isfinite(t_seed) && t_seed > 0.02) {
            init_ts(i) = std::max(init_ts(i), t_seed);
          }
        }
      }

      minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
    }
  }

  // 8. Optimize.
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double final_cost = minco_optimizer_->optimize(sparse_path, start_state, end_state, opt_traj);

  const double max_allowed_cost = 2000.0;
  if (!std::isfinite(final_cost) || final_cost > max_allowed_cost) {
    RCLCPP_WARN(
      logger_,
      "[MincoPlanner] Rejecting new trajectory! Cost (%.2f) exceeds limit (%.2f).",
      final_cost,
      max_allowed_cost);

    bool has_last_traj = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_last_traj = has_last_traj_;
    }

    if (has_last_traj && isTrajSafe()) {
        std::cout << YELLOW << "[MincoPlanner] Continuing to execute last trajectory since it's still safe. Cost of new traj: " << final_cost << RESET << std::endl;
      return true;
    }
    return false;
  }

  auto opt_end_time = rclcpp::Clock().now().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  // std::cout << GREEN << "[MincoPlanner] Minco optimization time: "
  //           << opt_duration << " seconds, "
  //           << "cost: " << final_cost << RESET << std::endl;

  // 8.5 Quality gating (hard validation) before publishing.
  if (!validateTrajectory(opt_traj, end_state.col(0))) {
    std::cout << RED << "[MincoPlanner] Trajectory validation failed! Rejecting." << RESET << std::endl;
    is_traj_safe_.store(false);
    return false;
  }

  double fallback_yaw = 0.0;
  if (start_state.col(1).head<2>().norm() > 1e-3) {
    fallback_yaw = std::atan2(start_state.col(1).y(), start_state.col(1).x());
  } else {
    const auto & q = current_pose.pose.orientation;
    fallback_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  traj_opt::Trajectory yaw_traj;
  if (use_yaw_opt_) {
    const bool yaw_success = optimizeYaw(
      start_state,
      opt_traj,
      yaw_traj,
      state,
      current_pose.pose,
      goal_yaw);
    if (!yaw_success) {
      std::cout << YELLOW
                << "[MincoPlanner] Yaw optimization failed. Falling back to constant yaw trajectory."
                << RESET << std::endl;

      Eigen::MatrixXd cMat(3, 6);
      cMat.setZero();
      cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
      const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
      yaw_traj.clear();
      yaw_traj.emplace_back(yaw_dur, cMat);
      yaw_traj.start_WT = opt_traj.start_WT;
    }
  } else {
    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
    const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
    yaw_traj.clear();
    yaw_traj.emplace_back(yaw_dur, cMat);
    yaw_traj.start_WT = opt_traj.start_WT;
  }

  // 9. Publish and cache.
  const double t_step = 0.05;
  int steps = static_cast<int>(std::ceil(opt_traj.getTotalDuration() / t_step)) + 1;
  steps = std::max(2, steps);

  utils::publishOptimizedTrajectory(
    opt_traj, yaw_traj, opt_path_pub_, opt_trajectory_id_, header_msg, steps, t_step);

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
  last_yaw_traj_ = yaw_traj;
  last_yaw_traj_.start_WT = last_traj_.start_WT;
  has_last_yaw_traj_ = true;

  is_traj_safe_.store(true);
  return true;
}

// -----------------------------------------------------------------------------
// 4) Internal implementation logic / algorithms
// -----------------------------------------------------------------------------

bool MincoPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  (void)tolerance;

  // 1. Reset plan output and header.
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  // 2. Search a discrete guide path using A*.
  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start, my_start;
  if (!utils::worldToMap(costmap_, logger_, wx, wy, mx_start, my_start)) {
    RCLCPP_ERROR(
      logger_,
      "Failed to convert start world coordinates (%.2f, %.2f) to map coordinates",
      wx,
      wy);
    return false;
  }
  utils::clearRobotCell(costmap_, mx_start, my_start);

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal, my_goal;
  if (!utils::worldToMap(costmap_, logger_, wx, wy, mx_goal, my_goal)) {
    RCLCPP_ERROR(
      logger_,
      "Failed to convert goal world coordinates (%.2f, %.2f) to map coordinates",
      wx,
      wy);
    return false;
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int nx = costmap_->getSizeInCellsX();
  unsigned int ny = costmap_->getSizeInCellsY();

  if (use_smac_ && smac_planner_) {
    // auto time = rclcpp::Clock().now().seconds();

    minco_planner::smac::SmacPlanner2DSimple::CoordinateVector smac_path;
    bool smac_success = smac_planner_->createPath(
      mx_start,
      my_start,
      mx_goal,
      my_goal,
      smac_path,
      cancel_checker);

    // auto time_end = rclcpp::Clock().now().seconds();

    if (!smac_success || smac_path.size() < 2) {
      RCLCPP_ERROR(logger_, "SMAC 2D: Failed to find path");
      return false;
    }

    // std::cout << GREEN << "[MincoPlanner] SMAC 2D planning time: " << (time_end - time)
    //           << " seconds, path length: " << smac_path.size() << RESET << std::endl;

    // Convert SMAC path to nav_msgs::Path.
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    latest_global_path_.clear();
    latest_global_path_.reserve(smac_path.size());
    plan.poses.reserve(smac_path.size());

    // SMAC path is reversed (from goal to start), so iterate backwards.
    for (auto it = smac_path.rbegin(); it != smac_path.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double wx, wy;
      utils::mapToWorld(costmap_, it->x, it->y, wx, wy);

      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path_.push_back(pose);
      plan.poses.push_back(pose);
    }
    if (!plan.poses.empty()) {
      plan.poses.back().pose.orientation = goal.orientation;
      latest_global_path_.back().pose.orientation = goal.orientation;
    }
  } else {
    // Use original A* algorithm.
    astar_planner_->setSize(nx, ny);
    astar_planner_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
    astar_planner_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
    astar_planner_->setupNavFn(true);
    astar_planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    int max_total_cycles = static_cast<int>(nx * ny) * 9999;
    int cycles_per_step = std::max(static_cast<int>(nx * ny / 20), static_cast<int>(nx + ny));
    // auto time = rclcpp::Clock().now().seconds();
    while (max_total_cycles > 0) {
      if (cancel_checker && cancel_checker()) {
        return false;
      }
      if (!astar_planner_->propNavFnAstar(cycles_per_step, cancel_checker)) {
        break;
      }
      max_total_cycles -= cycles_per_step;
    }
    // auto time_end = rclcpp::Clock().now().seconds();
    // std::cout << GREEN << "[MincoPlanner] A* planning time: " << (time_end - time) << " seconds"
    //           << RESET << std::endl;

    if (!astar_planner_->calcPath(nx * ny / 2) || astar_planner_->getPathLen() < 2) {
      return false;
    }

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
      utils::mapToWorld(costmap_, path_x[i], path_y[i], wx, wy);

      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path_.push_back(pose);
      plan.poses.push_back(pose);
    }
    if (!plan.poses.empty()) {
      plan.poses.back().pose.orientation = goal.orientation;
      latest_global_path_.back().pose.orientation = goal.orientation;
    }
  }

  return !plan.poses.empty();
}

std::vector<Eigen::Vector3d> MincoPlanner::extractLocalPath(const Eigen::Vector3d & cur_pos)
{
  std::vector<Eigen::Vector3d> local_segment;
  std::lock_guard<std::mutex> lock(path_mutex_);

  if (latest_global_path_.empty()) {
    return local_segment;
  }

  // 1. Find the nearest point on the global path.
  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i < latest_global_path_.size(); ++i) {
    const auto & pt = latest_global_path_[i].pose.position;
    double dist_sq = (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) +
                     (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // 2. Extract path from nearest point up to lookahead_dist_.
  double accum_dist = 0.0;

  local_segment.push_back(Eigen::Vector3d(
    latest_global_path_[start_idx].pose.position.x,
    latest_global_path_[start_idx].pose.position.y,
    0.0));

  for (size_t i = start_idx + 1; i < latest_global_path_.size(); ++i) {
    const auto & p1 = latest_global_path_[i - 1].pose.position;
    const auto & p2 = latest_global_path_[i].pose.position;

    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    accum_dist += dist;

    local_segment.push_back(Eigen::Vector3d(p2.x, p2.y, 0.0));

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

MincoPlanner::PlanningState MincoPlanner::determinePlanningState(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<Eigen::Vector3d> & new_path)
{
  if (!has_last_traj_) {
    return PlanningState::COLD_START;
  }

  double now = rclcpp::Clock().now().seconds() + 0.03;
  double t_dur = now - last_traj_.start_WT;
  if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
    std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Invalid time duration (t_dur="
              << t_dur << "s)" << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
  Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
  double tracking_error = (current_pos - pred_pos).norm();
  Eigen::Vector3d current_speed = getCurrentSpeed();
  double dynamic_error_threshold = 0.5 + 0.3 * current_speed.head<2>().norm();
  double vel_error = (current_speed - pred_vel).norm();
  if (tracking_error > dynamic_error_threshold) {
    std::cout << YELLOW << "[MincoPlanner] Large tracking error (" << tracking_error
              << "m). Downgrading to COLD_START." << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  if (vel_error > 0.3) {
    std::cout << YELLOW << "[MincoPlanner] Large velocity error (" << vel_error
              << "m/s). Downgrading to COLD_START." << RESET << std::endl;
    // return PlanningState::COLD_START;
  }

  if (new_path.size() >= 2) {
    Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
    if (pred_vel.norm() > 0.1) {
      Eigen::Vector3d path_dir = (new_path[1] - new_path[0]).normalized();
      Eigen::Vector3d vel_dir = pred_vel.normalized();
      double dot = vel_dir.dot(path_dir);

      if (dot < 0.5) {
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
  if (has_last_traj_) {
    double now = rclcpp::Clock().now().seconds();
    double t_dur = now - last_traj_.start_WT;
    if (t_dur > 0 && t_dur < last_traj_.getTotalDuration()) {
        start_state.col(1) = 0.5*last_traj_.getVel(t_dur);
    }
  }
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

bool MincoPlanner::optimizeYaw(
  const Eigen::Matrix3d & start_state,
  const traj_opt::Trajectory & pos_traj,
  traj_opt::Trajectory & out_yaw_traj,
  PlanningState state,
  const geometry_msgs::msg::Pose & current_pose,
  double goal_yaw)
{
  if (!yaw_opt_) {
    return false;
  }

  const double pos_dur = pos_traj.getTotalDuration();
  if (!(std::isfinite(pos_dur) && pos_dur > 1e-6)) {
    return false;
  }

  Eigen::Vector4d init_yaw_state = Eigen::Vector4d::Zero();
  Eigen::Vector4d goal_yaw_state = Eigen::Vector4d::Zero();

  bool use_hot_seed = false;
  if (state == PlanningState::HOT_START) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_yaw_traj_ && has_last_traj_) {
      const double t_dur = nowSeconds() - last_traj_.start_WT;
      const double yaw_dur = last_yaw_traj_.getTotalDuration();
      if (std::isfinite(t_dur) && std::isfinite(yaw_dur) && yaw_dur > 1e-6 && t_dur >= 0.0) {
        const double sample_t = std::min(t_dur, yaw_dur);
        init_yaw_state(0) = last_yaw_traj_.getPos(sample_t)(0);
        init_yaw_state(1) = last_yaw_traj_.getVel(sample_t)(0);
        use_hot_seed = true;
      }
    }
  }

  if (!use_hot_seed) {
    if (start_state.col(1).head<2>().norm() > 1e-3) {
      init_yaw_state(0) = std::atan2(start_state.col(1).y(), start_state.col(1).x());
    } else {
      const auto & q = current_pose.orientation;
      init_yaw_state(0) = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    init_yaw_state(1) = 0.0;
  }

  if (!std::isfinite(goal_yaw)) {
    goal_yaw = init_yaw_state(0);
  }
  const double yaw_err = std::atan2(
    std::sin(goal_yaw - init_yaw_state(0)),
    std::cos(goal_yaw - init_yaw_state(0)));
  goal_yaw_state(0) = init_yaw_state(0) + yaw_err;
  goal_yaw_state(1) = 0.0;

  return yaw_opt_->optimize(
    init_yaw_state,
    goal_yaw_state,
    pos_traj,
    out_yaw_traj,
    5,
    false,
    true);
}

// -----------------------------------------------------------------------------
// 5) Helpers / callbacks / getters
// -----------------------------------------------------------------------------

bool MincoPlanner::validateTrajectory(
  const traj_opt::Trajectory & traj,
  const Eigen::Vector3d & expected_end_pos)
{
  constexpr double kDt = 0.05;
  constexpr double kSevereScale = 1.5;

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid duration." << RESET << std::endl;
    return false;
  }

  const double vmax = minco_config.max_vel;
  const double amax = minco_config.max_acc;
  if (!(std::isfinite(vmax) && std::isfinite(amax) && vmax > 1e-6 && amax > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid vmax/amax config." << RESET
              << std::endl;
    return false;
  }

  const double vmax_severe = kSevereScale * vmax;
  const double amax_severe = kSevereScale * amax;

  // 1) Dynamic feasibility (severe violation gate).
  for (double t = 0.0; t <= dur; t += kDt) {
    const Eigen::Vector3d v = traj.getVel(t);
    const Eigen::Vector3d a = traj.getAcc(t);
    if (!(v.allFinite() && a.allFinite())) {
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite v/a." << RESET
                << std::endl;
      return false;
    }
    if (v.norm() > vmax_severe || a.norm() > amax_severe) {
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: severe dynamics violation."
                << " |v|=" << v.norm() << " (limit=" << vmax_severe << ")"
                << ", |a|=" << a.norm() << " (limit=" << amax_severe << ")" << RESET
                << std::endl;
      return false;
    }
  }

  // 2) Goal reachability.
  const Eigen::Vector3d end_pos = traj.getPos(dur);
  if (!end_pos.allFinite()) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite end position." << RESET
              << std::endl;
    return false;
  }

  const double goal_err = (end_pos - expected_end_pos).norm();
  if (!(std::isfinite(goal_err) && goal_err <= traj_goal_tolerance_)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: goal not reached. err=" << goal_err
              << " tol=" << traj_goal_tolerance_ << RESET << std::endl;
    return false;
  }

  // 3) Collision safety.
  if (!checkCollision(traj)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: collision detected." << RESET << std::endl;
    return false;
  }

  return true;
}

bool MincoPlanner::checkCollision()
{
  if (!costmap_) {
    return true;
  }

  // Snapshot trajectory under mutex to avoid data races with ReplanLocal().
  traj_opt::Trajectory traj_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_) {
      return true;
    }
    traj_snapshot = last_traj_;
  }

  const double dur = traj_snapshot.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  const double dt = 0.05;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  for (double t = 0.0; t <= dur; t += dt) {
    const Eigen::Vector3d pos = traj_snapshot.getPos(t);
    unsigned int mx, my;
    if (!utils::worldToMap(costmap_, logger_, pos.x(), pos.y(), mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }

  return true;
}

bool MincoPlanner::checkCollision(const traj_opt::Trajectory & traj)
{
  if (!costmap_) {
    return true;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  // Dense sampling to catch slight obstacle penetration.
  const double dt = 0.05;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  for (double t = 0.0; t <= dur; t += dt) {
    const Eigen::Vector3d pos = traj.getPos(t);
    unsigned int mx, my;
    if (!utils::worldToMap(costmap_, logger_, pos.x(), pos.y(), mx, my)) {
      return false;
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
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
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 2000, "[MincoPlanner] Trajectory collision detected.");
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_last_traj_) {
    const double t_dur = nowSeconds() - last_traj_.start_WT;
    const double total = last_traj_.getTotalDuration();
    if (std::isfinite(t_dur) && std::isfinite(total) && t_dur >= 0.0 && t_dur <= total) {
      start_state.col(1) = last_traj_.getVel(t_dur);
      start_state.col(2) = last_traj_.getAcc(t_dur);
    }
  }

  const auto & q = current_pose.pose.orientation;
  double current_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  utils::publishBackupTrajectory(
    backup_traj, opt_path_pub_, opt_trajectory_id_, header_msg, 20, 0.1, current_yaw);
}

traj_opt::Trajectory MincoPlanner::generateBackupTraj(const Eigen::Matrix3d & start_state)
{
  auto make_stop_traj = [&start_state]() -> traj_opt::Trajectory {
    traj_opt::Trajectory stop_traj;
    const Eigen::Vector3d p = start_state.col(0);

    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat.col(5) = p;

    // Two very short constant pieces ("2 points" semantics).
    stop_traj.emplace_back(0.2, cMat);
    stop_traj.emplace_back(0.2, cMat);
    return stop_traj;
  };

  if (!corridor_gen_ || !backup_opt_) {
    std::cout << RED << "[MincoPlanner] Backup optimizer not initialized!" << RESET << std::endl;
    return make_stop_traj();
  }

  // Step 1: Generate SFC (safe box).
  auto safe_poly = corridor_gen_->generateSafeBox(start_state.col(0), 1.0);

  // Step 2: Setup backup optimizer.
  backup_opt_->setInitState(start_state);
  backup_opt_->setStopConstraints();
  backup_opt_->setPolygons({safe_poly});

  // Step 3: Optimize.
  traj_opt::Trajectory backup_traj;
  bool success = backup_opt_->optimize(backup_traj);

  // Step 4: Return.
  if (success) {
    return backup_traj;
  }

  std::cout << RED << "[MincoPlanner] Backup trajectory optimization failed, fallback to stop." << RESET
            << std::endl;
  return make_stop_traj();
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

void MincoPlanner::cancelGoal()
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  has_pending_goal_ = false;
  if (fsm_) {
    fsm_->cancelGoal();
  }
}

double MincoPlanner::nowSeconds() const
{
  return rclcpp::Clock().now().seconds();
}

double MincoPlanner::getTrajectoryRemainTime() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_traj_) {
    return 0.0;
  }
  double passed_time = nowSeconds() - last_traj_.start_WT;
  return std::max(0.0, last_traj_.getTotalDuration() - passed_time);
}

bool MincoPlanner::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  if (!costmap_ros_) {
    return false;
  }
  return costmap_ros_->getRobotPose(pose);
}

bool MincoPlanner::checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (latest_global_path_.empty()) {
    return false;
  }

  const auto & goal = latest_global_path_.back().pose.position;
  const double dx = current_pose.pose.position.x - goal.x;
  const double dy = current_pose.pose.position.y - goal.y;
  const double dist = std::hypot(dx, dy);
  return std::isfinite(dist) && dist <= traj_goal_tolerance_;
}

Eigen::Vector3d MincoPlanner::getCurrentSpeed() const
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (has_latest_odom_) {
    return Eigen::Vector3d(latest_odom_.twist.twist.linear.x, latest_odom_.twist.twist.linear.y, latest_odom_.twist.twist.linear.z);
  }
  return Eigen::Vector3d::Zero();
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

double MincoPlanner::getEsdfDistance(const Eigen::Vector3d & pos) const
{
  if (!esdf_map_) {
    return 0.0;
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  esdf_map_->evaluate(pos, dist, grad);
  return dist;
}

void MincoPlanner::publishEscapeCommand(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const Eigen::Vector2d & escape_vel)
{
  if (visualizer_) {
    visualizer_->publishRecoveryDebug(current_pose, escape_vel, 0.5);
  }

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();
  utils::publishEscapeCommand(
    current_pose,
    escape_vel,
    opt_path_pub_,
    opt_trajectory_id_,
    header_msg);
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
