// Corresponding header
#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

// Project
#include "minco_core/minco_fsm.hpp"
#include "minco_core/minco_utils.hpp"
#include "minco_core/visualizer.hpp"
#include <iostream>

namespace minco_planner {

using namespace color_text;

MincoPlanner::MincoPlanner() : tf_(nullptr), costmap_(nullptr)
{
}

MincoPlanner::~MincoPlanner() = default;

// -----------------------------------------------------------------------------
// 2) Lifecycle management
// -----------------------------------------------------------------------------

void MincoPlanner::configure(const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();  // map

  auto node = parent.lock();
  logger_ = node->get_logger();

  const std::string prefix = name_ + ".";

  // --- General config --------------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(node, prefix + "tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "tolerance", tolerance_);

  nav2_util::declare_parameter_if_not_declared(node, prefix + "use_smac", rclcpp::ParameterValue(false));
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

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.turn_angle_deadzone", rclcpp::ParameterValue(0.174));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_deadzone", minco_config.turn_angle_deadzone);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.turn_angle_saturation", rclcpp::ParameterValue(1.57));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_saturation", minco_config.turn_angle_saturation);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.min_turn_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "minco_optimizer.min_turn_vel", minco_config.min_turn_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.decay_power", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.decay_power", minco_config.decay_power);

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

  double penalty_weight_time_barrier = 0.0;
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minco_optimizer.penalty_weight_time_barrier", rclcpp::ParameterValue(100.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time_barrier", penalty_weight_time_barrier);

  minco_config.penaltyWeights.resize(5);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;
  minco_config.penaltyWeights(4) = penalty_weight_time_barrier;

  minco_config.magnitudeBounds.resize(3);
  minco_config.magnitudeBounds(0) = minco_config.safe_dist;
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;

  // --- Map / ESDF config -----------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(node,
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
  node->get_parameter(prefix + "recovery_server.fail_threshold", recovery_server_config_.fail_threshold);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.cooldown_sec", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "recovery_server.cooldown_sec", recovery_server_config_.cooldown_sec);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.recovery_window_sec", rclcpp::ParameterValue(3.0));
  node->get_parameter(
    prefix + "recovery_server.recovery_window_sec", recovery_server_config_.recovery_window_sec);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "recovery_server.escape_speed", rclcpp::ParameterValue(0.4));
  node->get_parameter(prefix + "recovery_server.escape_speed", recovery_server_config_.escape_speed);

  // --- Components / publishers / timers -------------------------------------

  astar_planner_ = std::make_unique<Astar>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

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
    odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
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
    parent, "/global_costmap/voxel_grid", esdf_resolution_, dynamic_esdf_size, dynamic_dilation_radius);

  // Initialize ESDF window center once at startup so static validation works without odom.
  geometry_msgs::msg::PoseStamped init_pose;
  if (costmap_ros_ && costmap_ros_->getRobotPose(init_pose)) {
    esdf_map_->setRobotPosition(init_pose.pose.position.x, init_pose.pose.position.y);
  } else {
    esdf_map_->setRobotPosition(0.0, 0.0);
    RCLCPP_WARN(logger_,
      "[MincoPlanner] Failed to get initial robot pose from costmap, fallback ESDF center to (0, 0).");
  }

  const bool esdf_loaded = esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);
  if (!esdf_loaded) {
    std::cout << RED << "[MincoPlanner] "
              << "Failed to load Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
  } else {
    std::cout << MAGENTA << "[MincoPlanner] "
              << "Successfully loaded Static ESDF map from PCD: " << esdf_pcd_path_ << RESET << std::endl;
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
  planner_handle_ = MincoPlanner::Ptr(this, [](MincoPlanner *) {
  });

  // High-level FSM @ 20Hz.
  fsm_ = std::make_unique<MincoFsm>(planner_handle_, recovery_server_);
  fsm_timer_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), [this]() {
    if (fsm_) {
      fsm_->callMainFsmOnce();
    }
  });

  // Asynchronous safety monitor @ 20Hz.
  safety_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / 20.0), std::bind(&MincoPlanner::safetyTimerCallback, this));

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

  const std::string esdf_path_param = name_ + ".static_esdf.esdf_pcd_path";
  const std::string esdf_resolution_param = name_ + ".static_esdf.esdf_resolution";
  const std::string dynamic_size_param = name_ + ".static_esdf.dynamic_esdf_size";
  const std::string dynamic_dilation_param = name_ + ".static_esdf.dynamic_dilation_radius";
  const std::string max_vel_param = name_ + ".minco_optimizer.max_velocity";
  const std::string max_acc_param = name_ + ".minco_optimizer.max_acceleration";
  const std::string penalty_pos_param = name_ + ".minco_optimizer.penalty_weight_pos";
  const std::string penalty_vel_param = name_ + ".minco_optimizer.penalty_weight_vel";
  const std::string penalty_acc_param = name_ + ".minco_optimizer.penalty_weight_acc";
  const std::string penalty_att_param = name_ + ".minco_optimizer.penalty_weight_att";
  const std::string penalty_time_barrier_param = name_ + ".minco_optimizer.penalty_weight_time_barrier";

  std::string next_esdf_pcd_path = esdf_pcd_path_;
  double next_esdf_resolution = esdf_resolution_;
  bool need_reload_static_esdf = false;

  double next_max_vel = minco_config.max_vel;
  double next_max_acc = minco_config.max_acc;
  double next_penalty_pos =
    (minco_config.penaltyWeights.size() > 0) ? minco_config.penaltyWeights(0) : 1000.0;
  double next_penalty_vel =
    (minco_config.penaltyWeights.size() > 1) ? minco_config.penaltyWeights(1) : 1000.0;
  double next_penalty_acc =
    (minco_config.penaltyWeights.size() > 2) ? minco_config.penaltyWeights(2) : 10000.0;
  double next_penalty_att =
    (minco_config.penaltyWeights.size() > 3) ? minco_config.penaltyWeights(3) : 1000.0;
  double next_penalty_time_barrier =
    (minco_config.penaltyWeights.size() > 4) ? minco_config.penaltyWeights(4) : 100.0;
  bool optimizer_config_changed = false;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();

    auto parse_numeric = [&](double & out) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        out = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        out = static_cast<double>(param.as_int());
        return true;
      }
      return false;
    };

    if (param_name == esdf_path_param) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason = "Parameter must be a string: " + esdf_path_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      next_esdf_pcd_path = param.as_string();
      need_reload_static_esdf = true;
      continue;
    }

    if (param_name == esdf_resolution_param) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
          param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "Parameter must be a number: " + esdf_resolution_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      const double candidate_resolution = (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                                            ? param.as_double()
                                            : static_cast<double>(param.as_int());
      if (!std::isfinite(candidate_resolution) || candidate_resolution <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be > 0: " + esdf_resolution_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      next_esdf_resolution = candidate_resolution;
      need_reload_static_esdf = true;
      continue;
    }

    if (param_name == dynamic_size_param || param_name == dynamic_dilation_param) {
      result.successful = false;
      result.reason =
        "Dynamic ESDF layer params are not hot-reloadable, please restart node: " + param_name;
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    if (param_name == max_vel_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_vel_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_vel = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == max_acc_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_acc_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_acc = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == penalty_pos_param || param_name == penalty_vel_param ||
        param_name == penalty_acc_param || param_name == penalty_att_param ||
        param_name == penalty_time_barrier_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate < 0.0) {
        result.successful = false;
        result.reason = "Penalty weight must be a non-negative number: " + param_name;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      if (param_name == penalty_pos_param) {
        next_penalty_pos = candidate;
      } else if (param_name == penalty_vel_param) {
        next_penalty_vel = candidate;
      } else if (param_name == penalty_acc_param) {
        next_penalty_acc = candidate;
      } else if (param_name == penalty_att_param) {
        next_penalty_att = candidate;
      } else {
        next_penalty_time_barrier = candidate;
      }

      optimizer_config_changed = true;
      continue;
    }
  }

  if (optimizer_config_changed) {
    minco_config.max_vel = next_max_vel;
    minco_config.max_acc = next_max_acc;

    minco_config.penaltyWeights.resize(5);
    minco_config.penaltyWeights(0) = next_penalty_pos;
    minco_config.penaltyWeights(1) = next_penalty_vel;
    minco_config.penaltyWeights(2) = next_penalty_acc;
    minco_config.penaltyWeights(3) = next_penalty_att;
    minco_config.penaltyWeights(4) = next_penalty_time_barrier;

    minco_config.magnitudeBounds.resize(3);
    minco_config.magnitudeBounds(0) = minco_config.safe_dist;
    minco_config.magnitudeBounds(1) = minco_config.max_vel;
    minco_config.magnitudeBounds(2) = minco_config.max_acc;

    if (minco_optimizer_) {
      minco_optimizer_->setConfig(minco_config);
    }

    RCLCPP_INFO(logger_,
      "[MincoPlanner] Updated optimizer params: vmax=%.3f, amax=%.3f, w=[%.3f %.3f %.3f %.3f %.3f]",
      minco_config.max_vel,
      minco_config.max_acc,
      minco_config.penaltyWeights(0),
      minco_config.penaltyWeights(1),
      minco_config.penaltyWeights(2),
      minco_config.penaltyWeights(3),
      minco_config.penaltyWeights(4));
  }

  if (!need_reload_static_esdf) {
    return result;
  }

  if (!esdf_map_) {
    result.successful = false;
    result.reason = "ESDF map is not initialized";
    RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
    return result;
  }

  const std::string prev_esdf_pcd_path = esdf_pcd_path_;
  const double prev_esdf_resolution = esdf_resolution_;

  esdf_pcd_path_ = next_esdf_pcd_path;
  esdf_resolution_ = next_esdf_resolution;

  const bool reloaded = esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);
  if (reloaded) {
    RCLCPP_INFO(logger_,
      "[MincoPlanner] Reloaded static ESDF map from PCD: %s (resolution=%.3f)",
      esdf_pcd_path_.c_str(),
      esdf_resolution_);
    return result;
  }

  esdf_pcd_path_ = prev_esdf_pcd_path;
  esdf_resolution_ = prev_esdf_resolution;
  esdf_map_->loadStaticMap(esdf_pcd_path_, esdf_resolution_);

  result.successful = false;
  result.reason = "Failed to reload static ESDF map from PCD: " + next_esdf_pcd_path;
  RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
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
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
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
    goal_yaw = utils::quaternionToYaw(latest_global_path_.back().pose.orientation);
  }

  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  // 2. Extract local dense path.
  std::vector<Eigen::Vector3d> dense_local_path = extractLocalPath(cur_pos);
  if (dense_local_path.size() < 2) {
    return false;
  }
  const bool local_end_is_goal =
    (global_goal - dense_local_path.back()).head<2>().norm() <= traj_goal_tolerance_;
  // 3. Sparsify local path.
  std::vector<Eigen::Vector3d> sparse_path = utils::getSparseWaypoints(dense_local_path,
    minco_config.max_vel,
    minco_config.max_acc,
    local_end_is_goal,
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
  // std::cout << CYAN << "[MincoPlanner] Planning state: " << ((state == PlanningState::HOT_START) ? "HOT_START" : "COLD_START")
  //           << RESET << std::endl;
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
    prepareColdStart(current_pose.pose, start_state, sparse_path);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }
  // std::cout << CYAN << "[MincoPlanner] Start state: pos=(" << start_state.col(0).x() << ", " << start_state.col(0).y()
  //           << "), vel=(" << start_state.col(1).x() << ", " << start_state.col(1).y()
  //           << "), acc=(" << start_state.col(2).x() << ", " << start_state.col(2).y() << ")"
  //           << RESET << std::endl;
  // P1b: If the robot is inside an obstacle (ESDF dist < 0), project the
  // start position to the nearest free space along the ESDF gradient.
  if (esdf_map_) {
    double start_esdf_dist = 0.0;
    Eigen::Vector3d start_esdf_grad = Eigen::Vector3d::Zero();
    esdf_map_->evaluate(start_state.col(0), start_esdf_dist, start_esdf_grad);
    if (start_esdf_dist < 0.0 && start_esdf_grad.norm() > 1e-6) {
      constexpr double kMargin = 0.05;
      start_state.col(0) += (kMargin - start_esdf_dist) * start_esdf_grad.normalized();
    }
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
      if (n > 0.1) {
        tangent /= n;
      } else {
        tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
    }
    const double v_curr = std::max(0.0, start_state.col(1).head<2>().norm());
    const double amax = std::max(0.0, minco_config.max_acc);
    const double v_max_kinematic = std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * amax * dist_to_goal));
    double local_end_vmax = minco_config.max_vel;
    if (sparse_path.size() >= 3) {
      local_end_vmax = utils::LimitLocalVel(sparse_path, sparse_path.size() - 3, minco_config.max_vel, 
                                            minco_config.turn_angle_deadzone, minco_config.turn_angle_saturation, 
                                            minco_config.min_turn_vel, minco_config.decay_power);
    }
    const double v_cmd = std::min({minco_config.max_vel, v_max_kinematic, dist_to_goal, local_end_vmax});
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
  const int N = static_cast<int>(sparse_path.size()) - 1;
  VecDf local_vmaxs(N);
  if (N > 0) {
    vec_Vec3f init_ps;
    VecDf init_ts(N);
    PTAllocation(sparse_path,
      start_state,
      local_end_is_goal,
      state,
      has_shifted_seed,
      shifted_waypoints,
      shifted_durations,
      init_ps,
      init_ts,
      local_vmaxs);

    minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
  }

  // 8. Optimize.
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double final_cost =
    minco_optimizer_->optimize(sparse_path, start_state, end_state, local_vmaxs, opt_traj);

  const double max_allowed_cost = 3000.0;
  if (!std::isfinite(final_cost) || final_cost > max_allowed_cost) {
    RCLCPP_WARN(logger_,
      "[MincoPlanner] Rejecting new trajectory! Cost (%.2f) exceeds limit (%.2f).",
      final_cost,
      max_allowed_cost);

    bool has_last_traj = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_last_traj = has_last_traj_;
    }

    if (has_last_traj && isTrajSafe()) {
      if(!isTrajectoryTimeExpired(rclcpp::Clock().now().seconds())) {
        std::cout << YELLOW
                  << "[MincoPlanner] Last trajectory is still valid and safe. Continuing to execute it."
                  << RESET << std::endl;
        return true;
      }
      return false;
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
    fallback_yaw = getCurrentYawFromOdom();
  }

  // Slope-aware short-horizon yaw lock based on real odometry attitude.
  double pitch = 0.0;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (has_latest_odom_) {
      const auto & odom_q = latest_odom_.pose.pose.orientation;
      const tf2::Quaternion q(odom_q.x, odom_q.y, odom_q.z, odom_q.w);
      double roll = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }
  }

  constexpr double slope_threshold = 0.1;
  if (std::abs(pitch) > slope_threshold && sparse_path.size() >= 2) {
    goal_yaw = std::atan2(end_state.col(1).y(), end_state.col(1).x());
  } else if (sparse_path.size() >= 2) {
    const Eigen::Vector2d tail_dir = (sparse_path.back() - sparse_path[sparse_path.size() - 2]).head<2>();
    if (tail_dir.norm() > 0.1) {
      goal_yaw = std::atan2(tail_dir.y(), tail_dir.x());
    } else {
      goal_yaw = getCurrentYawFromOdom();
    }
  } else {
    goal_yaw = getCurrentYawFromOdom();
  }

  traj_opt::Trajectory yaw_traj;
  if (use_yaw_opt_) {
    const bool yaw_success =
      optimizeYaw(start_state, opt_traj, yaw_traj, state, current_pose.pose, goal_yaw);
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

void MincoPlanner::PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
  const Eigen::Matrix3d & start_state,
  bool goal_reached,
  PlanningState state,
  bool has_shifted_seed,
  const vec_Vec3f & shifted_waypoints,
  const VecDf & shifted_durations,
  vec_Vec3f & init_ps,
  VecDf & init_ts,
  VecDf & local_vmaxs) const
{
  const int N = static_cast<int>(sparse_path.size()) - 1;
  if (N <= 0) {
    init_ps.clear();
    init_ts.resize(0);
    local_vmaxs.resize(0);
    return;
  }

  const double global_vmax = std::max(0.0, minco_config.max_vel);
  const double amax = std::max(1e-3, minco_config.max_acc);
  const double kMinSegTime = 0.1;
  const double kBrakeSafety = 1.2;
  const double max_brake_dist = (global_vmax * global_vmax) / (2.0 * amax);

  local_vmaxs.resize(N);
  local_vmaxs.setConstant(global_vmax);
  init_ts.resize(N);

  init_ps.clear();
  init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
  int copyPs = 0;
  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldWp = static_cast<int>(shifted_waypoints.size());
    const int oldPs = std::max(0, oldWp - 2);
    copyPs = std::min(std::max(0, N - 1), oldPs);
  }
  for (int j = 0; j < copyPs; ++j) {
    init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
  }
  for (int j = copyPs; j < (N - 1); ++j) {
    init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
  }

  std::vector<double> seg_len(static_cast<size_t>(N), 0.0);
  for (int i = 0; i < N; ++i) {
    const double dis =
      (sparse_path[static_cast<size_t>(i + 1)] - sparse_path[static_cast<size_t>(i)]).head<2>().norm();
    seg_len[static_cast<size_t>(i)] = (std::isfinite(dis) && dis > 0.0) ? dis : 0.0;
  }

  std::vector<double> remain_after(static_cast<size_t>(N), 0.0);
  for (int i = N - 2; i >= 0; --i) {
    remain_after[static_cast<size_t>(i)] =
      remain_after[static_cast<size_t>(i + 1)] + seg_len[static_cast<size_t>(i + 1)];
  }

  std::vector<double> local_vmax_vec(static_cast<size_t>(N), global_vmax);
  for (int i = 0; i < N - 1; ++i) {
    local_vmax_vec[static_cast<size_t>(i)] = utils::LimitLocalVel(sparse_path,
      i,
      global_vmax,
      minco_config.turn_angle_deadzone,
      minco_config.turn_angle_saturation,
      minco_config.min_turn_vel,
      minco_config.decay_power);
  }
  utils::VelPropogation(seg_len, amax, local_vmax_vec);
  for (int i = 0; i < N; ++i) {
    local_vmaxs(i) = local_vmax_vec[static_cast<size_t>(i)];
  }

  double v_curr = start_state.col(1).head<2>().norm();
  if (!std::isfinite(v_curr) || v_curr < 0.0) {
    v_curr = 0.0;
  }

  const double local_goal_remain = goal_reached ? 0.0 : max_brake_dist;
  for (int i = 0; i < N; ++i) {
    const bool is_last = (i == N - 1);
    const double L = seg_len[static_cast<size_t>(i)];
    const double remain = remain_after[static_cast<size_t>(i)] + local_goal_remain;

    if (L <= 1e-6) {
      init_ts(i) = kMinSegTime;
      continue;
    }

    if (is_last && goal_reached) {
      const double t_stop = v_curr / amax;
      const double t_dist = L / std::max(v_curr, 0.1);
      init_ts(i) = std::max({kMinSegTime, t_dist, kBrakeSafety * t_stop});
      v_curr = 0.0;
      continue;
    }

    const double local_vmax = local_vmax_vec[static_cast<size_t>(i)];
    const double v_next = utils::ComputeNextSpeed(v_curr, L, remain, amax, local_vmax);
    init_ts(i) = utils::ComputeSegmentTime(L, v_curr, v_next, local_vmax, amax, kMinSegTime);
    v_curr = v_next;
  }

  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldN = std::min(N, static_cast<int>(shifted_durations.size()));
    for (int i = 0; i < oldN; ++i) {
      const double t_seed = shifted_durations(i);
      if (std::isfinite(t_seed) && t_seed > 0.02) {
        init_ts(i) = std::max(init_ts(i), t_seed);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// 4) Internal implementation logic / algorithms
// -----------------------------------------------------------------------------

static bool projectStartToFreeCell(nav2_costmap_2d::Costmap2D * costmap,
                                   unsigned int & mx, unsigned int & my)
{
  const unsigned int nx = costmap->getSizeInCellsX();
  const unsigned int ny = costmap->getSizeInCellsY();
  auto isFree = [costmap](unsigned int x, unsigned int y) {
    return costmap->getCost(x, y) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };
  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    int sx = static_cast<int>(mx) + dx4[k];
    int sy = static_cast<int>(my) + dy4[k];
    if (sx < 0 || sy < 0 || sx >= static_cast<int>(nx) || sy >= static_cast<int>(ny)) continue;
    if (isFree(static_cast<unsigned int>(sx), static_cast<unsigned int>(sy))) return false;
  }
  constexpr int kMaxRadius = 50;
  for (int r = 1; r <= kMaxRadius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        int cx = static_cast<int>(mx) + dx;
        int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(nx) || cy >= static_cast<int>(ny)) continue;
        if (isFree(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy))) {
          mx = static_cast<unsigned int>(cx); my = static_cast<unsigned int>(cy); return true;
        }
      }
    }
  }
  return false;
}

bool MincoPlanner::makePlan(const geometry_msgs::msg::Pose & start,
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
      logger_, "Failed to convert start world coordinates (%.2f, %.2f) to map coordinates", wx, wy);
    return false;
  }
  utils::clearRobotCell(costmap_, mx_start, my_start);

  // P1a: If start cell is surrounded by inflated obstacles, spiral-search
  // outward to find the nearest free cell so A* can expand from the start.
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    projectStartToFreeCell(costmap_, mx_start, my_start);
  }

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal, my_goal;
  if (!utils::worldToMap(costmap_, logger_, wx, wy, mx_goal, my_goal)) {
    RCLCPP_ERROR(
      logger_, "Failed to convert goal world coordinates (%.2f, %.2f) to map coordinates", wx, wy);
    return false;
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int nx = costmap_->getSizeInCellsX();
  unsigned int ny = costmap_->getSizeInCellsY();

  if (use_smac_ && smac_planner_) {
    // auto time = rclcpp::Clock().now().seconds();

    minco_planner::smac::SmacPlanner2DSimple::CoordinateVector smac_path;
    bool smac_success =
      smac_planner_->createPath(mx_start, my_start, mx_goal, my_goal, smac_path, cancel_checker);

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
      plan.poses.back().pose.position.x = goal.position.x;
      plan.poses.back().pose.position.y = goal.position.y;
      plan.poses.back().pose.position.z = goal.position.z;
      plan.poses.back().pose.orientation = goal.orientation;
      latest_global_path_.back().pose.position.x = goal.position.x;
      latest_global_path_.back().pose.position.y = goal.position.y;
      latest_global_path_.back().pose.position.z = goal.position.z;
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
      plan.poses.back().pose.position.x = goal.position.x;
      plan.poses.back().pose.position.y = goal.position.y;
      plan.poses.back().pose.position.z = goal.position.z;
      plan.poses.back().pose.orientation = goal.orientation;
      latest_global_path_.back().pose.position.x = goal.position.x;
      latest_global_path_.back().pose.position.y = goal.position.y;
      latest_global_path_.back().pose.position.z = goal.position.z;
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
    double dist_sq =
      (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // 2. Extract path from nearest point up to lookahead_dist_.
  double accum_dist = 0.0;

  local_segment.push_back(Eigen::Vector3d(
    latest_global_path_[start_idx].pose.position.x, latest_global_path_[start_idx].pose.position.y, 0.0));

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
  const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path)
{
  if (!has_last_traj_) {
    return PlanningState::COLD_START;
  }

  double now = rclcpp::Clock().now().seconds() + 0.005;
  double t_dur = now - last_traj_.start_WT;
  if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
    std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Invalid time duration (t_dur=" << t_dur
              << "s)" << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
  Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
  double tracking_error = (current_pos - pred_pos).norm();
  Eigen::Vector3d current_speed = getCurrentSpeed();
  double dynamic_error_threshold = 1.0 + 0.5 * current_speed.head<2>().norm();
  double vel_error = (current_speed - pred_vel).norm();
  if (tracking_error > dynamic_error_threshold) {
    std::cout << YELLOW << "[MincoPlanner] Large tracking error (" << tracking_error
              << "m). Downgrading to COLD_START." << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  if (vel_error > 1.0) {
    std::cout << YELLOW << "[MincoPlanner] Large velocity error (" << vel_error
              << "m/s). Downgrading to COLD_START." << RESET << std::endl;
    return PlanningState::HOT_START;
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

void MincoPlanner::prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
  Eigen::Matrix3d & start_state,
  const std::vector<Eigen::Vector3d> & sparse_path)
{
  start_state.setZero();
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  (void)sparse_path;
  Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(1) = real_speed;
  // bool has_valid_odom = false;
  // geometry_msgs::msg::Quaternion odom_q;
  // {
  //   std::lock_guard<std::mutex> lk(odom_mutex_);
  //   has_valid_odom = has_latest_odom_;
  //   if (has_valid_odom) {
  //     odom_q = latest_odom_.pose.pose.orientation;
  //   }
  // }

  // constexpr double slope_threshold = 0.05;
  // if (has_valid_odom && sparse_path.size() >= 2) {
  //   const tf2::Quaternion q(odom_q.x, odom_q.y, odom_q.z, odom_q.w);
  //   double roll = 0.0;
  //   double pitch = 0.0;
  //   double yaw = 0.0;
  //   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  //   if (std::abs(pitch) > slope_threshold) {
  //     Eigen::Vector2d path_dir = (sparse_path[1] - sparse_path[0]).head<2>();
  //     const double dir_norm = path_dir.norm();
  //     if (dir_norm > 0.1) {
  //       path_dir /= dir_norm;
  //       const double min_climb_speed = std::max(1.0, minco_config.max_vel * 0.8);
  //       const double cur_spd = std::hypot(real_speed.x(), real_speed.y());
  //       if (cur_spd < min_climb_speed) {
  //         real_speed.x() = path_dir.x() * min_climb_speed;
  //         real_speed.y() = path_dir.y() * min_climb_speed;
  //         start_state.col(2) = Eigen::Vector3d(
  //             path_dir.x() * minco_config.max_acc, path_dir.y() * minco_config.max_acc, 0.0);
  //       }
  //     }
  //   }
  // }

  start_state.col(1) = real_speed;
}

void MincoPlanner::prepareHotStart(
  const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state)
{
  start_state.setZero();
  // start_state.col(0) = last_traj_.getPos(t_dur);
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  start_state.col(1) = last_traj_.getVel(t_dur);
  // Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(2) = last_traj_.getAcc(t_dur);

  // // Slope-aware minimum speed: enforce min climb speed to prevent stalling on inclines.
  // const tf2::Quaternion q(
  //     start_pose.orientation.x, start_pose.orientation.y,
  //     start_pose.orientation.z, start_pose.orientation.w);
  // double roll = 0.0, pitch = 0.0, yaw = 0.0;
  // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // constexpr double slope_threshold = 0.05;
  // if (std::abs(pitch) > slope_threshold) {
  //   const Eigen::Vector2d path_dir(std::cos(yaw), std::sin(yaw));
  //   const double min_climb_speed = std::max(1.0, minco_config.max_vel * 0.8);
  //   const double cur_spd = std::hypot(real_speed.x(), real_speed.y());
  //   if (cur_spd < min_climb_speed) {
  //     real_speed.x() = path_dir.x() * min_climb_speed;
  //     real_speed.y() = path_dir.y() * min_climb_speed;
  //     start_state.col(2) = Eigen::Vector3d(
  //         path_dir.x() * minco_config.max_acc, path_dir.y() * minco_config.max_acc, 0.0);
  //   }
  //   start_state.col(1) = real_speed;
  // }
}

bool MincoPlanner::optimizeYaw(const Eigen::Matrix3d & start_state,
  const traj_opt::Trajectory & pos_traj,
  traj_opt::Trajectory & out_yaw_traj,
  PlanningState state,
  const geometry_msgs::msg::Pose & current_pose,
  double goal_yaw)
{
  (void)current_pose;

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
    if (start_state.col(1).head<2>().norm() > 0.1) {
      init_yaw_state(0) = std::atan2(start_state.col(1).y(), start_state.col(1).x());
    } else {
      init_yaw_state(0) = getCurrentYawFromOdom();
    }
    init_yaw_state(1) = 0.0;
  }

  if (!std::isfinite(goal_yaw)) {
    goal_yaw = init_yaw_state(0);
  }
  const double yaw_err =
    std::atan2(std::sin(goal_yaw - init_yaw_state(0)), std::cos(goal_yaw - init_yaw_state(0)));
  goal_yaw_state(0) = init_yaw_state(0) + yaw_err;
  goal_yaw_state(1) = 0.0;

  return yaw_opt_->optimize(init_yaw_state, goal_yaw_state, pos_traj, out_yaw_traj, 5, false, true);
}

// -----------------------------------------------------------------------------
// 5) Helpers / callbacks / getters
// -----------------------------------------------------------------------------

bool MincoPlanner::validateTrajectory(
  const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos)
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
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite v/a." << RESET << std::endl;
      return false;
    }
    if (v.norm() > vmax_severe || a.norm() > amax_severe) {
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: severe dynamics violation."
                << " |v|=" << v.norm() << " (limit=" << vmax_severe << ")"
                << ", |a|=" << a.norm() << " (limit=" << amax_severe << ")" << RESET << std::endl;
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
    if (esdf_map_) {
      double esdf_dist = 0.0;
      Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
      esdf_map_->evaluate(pos, esdf_dist, esdf_grad);
      if (esdf_dist <= 0.0) return false;
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
    if (esdf_map_) {
      double esdf_dist = 0.0;
      Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
      esdf_map_->evaluate(pos, esdf_dist, esdf_grad);
      if (esdf_dist <= 0.0) return false;
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
  prepareColdStart(current_pose.pose, start_state, std::vector<Eigen::Vector3d>{});
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_last_traj_) {
    const double t_dur = nowSeconds() - last_traj_.start_WT;
    const double total = last_traj_.getTotalDuration();
    if (std::isfinite(t_dur) && std::isfinite(total) && t_dur >= 0.0 && t_dur <= total) {
      start_state.col(1) = last_traj_.getVel(t_dur);
      start_state.col(2) = last_traj_.getAcc(t_dur);
    }
  }

  const double current_yaw = getCurrentYawFromOdom();
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
    const auto & twist = latest_odom_.twist.twist;
    const double yaw = utils::quaternionToYaw(latest_odom_.pose.pose.orientation);

    double vx_global = 0.0;
    double vy_global = 0.0;
    double omega_global = 0.0;
    utils::compensateLeverArm(
      twist.linear.x,
      twist.linear.y,
      twist.angular.z,
      yaw,
      vx_global,
      vy_global,
      omega_global);

    // std::cout << "[MincoPlanner] Lever-arm compensation: raw_v=(" << twist.linear.x << ", "
    //           << twist.linear.y << ") wz=" << twist.angular.z << " yaw=" << yaw << " -> v=("
    //           << vx_global << ", " << vy_global << ")" << std::endl;

    return Eigen::Vector3d(vx_global, vy_global, twist.linear.z);
  }
  return Eigen::Vector3d::Zero();
}

double MincoPlanner::getCurrentYawFromOdom() const
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (!has_latest_odom_) {
    return 0.0;
  }
  return utils::quaternionToYaw(latest_odom_.pose.pose.orientation);
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
  const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel)
{
  if (visualizer_) {
    visualizer_->publishRecoveryDebug(current_pose, escape_vel, 0.5);
  }

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = global_frame_;
  header_msg.stamp = rclcpp::Clock().now();
  const double current_yaw = getCurrentYawFromOdom();
  utils::publishEscapeCommand(
    current_pose, escape_vel, current_yaw, opt_path_pub_, opt_trajectory_id_, header_msg);
}

void MincoPlanner::clearRecoveryDebugVisualization()
{
  if (visualizer_) {
    visualizer_->clearRecoveryDebug();
  }
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
