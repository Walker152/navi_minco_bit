// Corresponding header
#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"

// Project
#include "rog_map/map_registry.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>

namespace minco_planner {

using namespace color_text;

MincoPlanner::MincoPlanner() : tf_(nullptr)
{
}

MincoPlanner::~MincoPlanner()
{
  planner_perf_monitor_.close();
}

void MincoPlanner::configureMincoPerfLogging(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & prefix)
{
  const std::string default_minco_csv_path = "/tmp/minco_perf_detailed.csv";

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.print_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.detailed_csv_enable", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.odom_sub_debug_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.print_period_sec", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.csv_flush_every_n", rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.minco_csv_path", rclcpp::ParameterValue(default_minco_csv_path));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.scenario", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "performance.variant", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.detailed_csv_enable", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.minco_csv_path", rclcpp::ParameterValue(default_minco_csv_path));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.scenario", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rog_map.performance.variant", rclcpp::ParameterValue(""));

  bool performance_enable = true;
  bool print_enable = true;
  bool detailed_csv_enable = false;
  bool odom_sub_debug_enable = true;
  double print_period_sec = 1.0;
  int csv_flush_every_n = 30;
  std::string minco_csv_path = default_minco_csv_path;
  std::string run_id;
  std::string scenario;
  std::string variant;
  node->get_parameter(prefix + "performance.enable", performance_enable);
  node->get_parameter(prefix + "performance.print_enable", print_enable);
  node->get_parameter(prefix + "performance.detailed_csv_enable", detailed_csv_enable);
  node->get_parameter(prefix + "performance.odom_sub_debug_enable", odom_sub_debug_enable);
  node->get_parameter(prefix + "performance.print_period_sec", print_period_sec);
  node->get_parameter(prefix + "performance.csv_flush_every_n", csv_flush_every_n);
  node->get_parameter(prefix + "performance.minco_csv_path", minco_csv_path);
  node->get_parameter(prefix + "performance.run_id", run_id);
  node->get_parameter(prefix + "performance.scenario", scenario);
  node->get_parameter(prefix + "performance.variant", variant);
  if (!detailed_csv_enable) {
    node->get_parameter(prefix + "rog_map.performance.detailed_csv_enable", detailed_csv_enable);
  }
  if (performance_enable) {
    node->get_parameter(prefix + "rog_map.performance.enable", performance_enable);
  }
  if (minco_csv_path == default_minco_csv_path) {
    node->get_parameter(prefix + "rog_map.performance.minco_csv_path", minco_csv_path);
  }
  if (run_id.empty()) {
    node->get_parameter(prefix + "rog_map.performance.run_id", run_id);
  }
  if (scenario.empty()) {
    node->get_parameter(prefix + "rog_map.performance.scenario", scenario);
  }
  if (variant.empty()) {
    node->get_parameter(prefix + "rog_map.performance.variant", variant);
  }

  PlannerPerformanceConfig perf_cfg;
  perf_cfg.enable = performance_enable;
  perf_cfg.print_enable = print_enable;
  perf_cfg.detailed_csv_enable = detailed_csv_enable;
  perf_cfg.odom_sub_debug_enable = odom_sub_debug_enable;
  perf_cfg.detailed_csv_path = minco_csv_path;
  perf_cfg.run_id = run_id;
  perf_cfg.scenario = scenario;
  perf_cfg.variant = variant;
  perf_cfg.print_period_sec = print_period_sec;
  perf_cfg.csv_flush_every_n = csv_flush_every_n;

  planner_perf_monitor_.configure(perf_cfg, logger_);
}

bool MincoPlanner::configureRogMap(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin_prefix)
{
  if (!node) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Cannot configure ROGMap without planner_server LifecycleNode.");
    return false;
  }

  try {
    rog_map::Config rog_cfg;
    rog_cfg.loadFromRosNode(node, plugin_prefix + "rog_map");
    rog_map_ros_ = std::make_shared<rog_map::ROGMapROS>(node, rog_cfg);
    rog_query_raw_ = rog_map_ros_->queryInterface();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Failed to configure ROGMap: %s", e.what());
    rog_map_ros_.reset();
    rog_query_raw_.reset();
    map_.reset();
    return false;
  }

  if (!rog_query_raw_) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap queryInterface is null.");
    rog_map_ros_.reset();
    return false;
  }

  rog_map::MapRegistry::set(rog_query_raw_);
  RCLCPP_INFO(
    logger_, "[MincoPlanner] ROGMap is created inside MincoPlanner plugin and shared by pointer.");
  return true;
}

bool MincoPlanner::ensureMapAvailable()
{
  if (rog_query_raw_ || map_) {
    return true;
  }

  auto map = rog_map::MapRegistry::get();
  if (map) {
    rog_query_raw_ = map;
    if (!map_) {
      map_ = map;
    }
    return true;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_ERROR_THROTTLE(logger_,
      *node->get_clock(),
      1000,
      "[MincoPlanner] MapQueryInterface unavailable: ROGMap was not created and MapRegistry is empty.");
  } else {
    RCLCPP_ERROR(logger_, "[MincoPlanner] MapQueryInterface unavailable.");
  }
  return false;
}

void MincoPlanner::rebuildModeDependentQueries()
{
  if (!mode_context_) {
    return;
  }

  mode_context_->rebuildQueries(rog_query_raw_, costmap_ros_.get(), tf_, logger_);
  map_ = mode_context_->dynamicQuery();

  if (global_path_searcher_) {
    global_path_searcher_->setQuery(mode_context_->globalQuery());
  }
  if (astar_planner_) {
    astar_planner_->setMap(mode_context_->globalQuery());
  }
  if (smac_planner_) {
    smac_planner_->setMap(mode_context_->globalQuery());
    smac_planner_->setESDFQuery(mode_context_->dynamicQuery());
  }
  if (minco_optimizer_) {
    minco_optimizer_->setMap(mode_context_->dynamicQuery());
  }
  if (corridor_gen_) {
    corridor_gen_->setMap(mode_context_->dynamicQuery());
  }
  if (safety_checker_) {
    safety_checker_->setQuery(mode_context_->dynamicQuery());
  }

  RCLCPP_INFO(logger_, "[MincoPlanner] Rebuilt mode-dependent map queries after raw ROGMap update.");
}

void MincoPlanner::initPlannerMode(
  const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame)
{
  mode_params_.planner_mode = planner_mode_param;
  mode_params_.map_frame = map_frame.empty() ? "map" : map_frame;
  mode_params_.rog_frame = rog_frame.empty() ? "camera_init" : rog_frame;
  mode_params_.priormap_use_nav2_global_search = priormap_use_nav2_global_search_;
  mode_params_.priormap_clip_seed_by_rog_boundary = priormap_clip_seed_by_rog_boundary_;
  mode_params_.priormap_rog_boundary_margin = priormap_rog_boundary_margin_;
  mode_params_.priormap_rog_boundary_sample_step = priormap_rog_boundary_sample_step_;
  mode_params_.exploration_boundary_margin = exploration_boundary_margin_;
  mode_params_.exploration_boundary_sample_step = exploration_boundary_sample_step_;
  mode_params_.exploration_unknown_as_occupied = exploration_unknown_as_occupied_;
  mode_params_.exploration_prefer_goal_direction = exploration_prefer_goal_direction_;

  mode_context_ = std::make_unique<PlannerModeContext>();
  mode_context_->configure(mode_params_, rog_query_raw_, costmap_ros_.get(), tf_, logger_);

  planning_frame_ = mode_context_->planningFrame();
  output_frame_ = mode_context_->outputFrame();
  map_frame_ = mode_context_->mapFrame();
  rog_frame_ = mode_context_->rogFrame();
  global_frame_ = output_frame_;
  map_ = mode_context_->dynamicQuery();

  RCLCPP_INFO(logger_,
    "[MincoPlanner] planner_mode=%s",
    mode_context_->mode() == PlannerMode::PRIORMAP ? "PRIORMAP" : "EXPLORATION");
  RCLCPP_INFO(logger_,
    "[MincoPlanner] planning_frame=%s output_frame=%s rog_frame=%s",
    planning_frame_.c_str(),
    output_frame_.c_str(),
    rog_frame_.c_str());
  RCLCPP_INFO(logger_,
    "[MincoPlanner] global_search=%s dynamic_query=%s",
    mode_context_->mode() == PlannerMode::PRIORMAP ? "Nav2Costmap" : "ROGMapBoundaryAstar",
    mode_context_->mode() == PlannerMode::PRIORMAP ? "FrameAwareRogQuery" : "DirectRogQuery");
}

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

  auto node = parent.lock();
  logger_ = node->get_logger();

  const std::string prefix = name_ + ".";
  configureMincoPerfLogging(node, prefix);

  // --- General config --------------------------------------------------------

  std::string planner_mode_param = "PRIORMAP";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "planner_mode", rclcpp::ParameterValue(planner_mode_param));
  node->get_parameter(prefix + "planner_mode", planner_mode_param);

  std::string configured_map_frame = "map";
  std::string configured_rog_frame = "camera_init";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "frames.map_frame", rclcpp::ParameterValue(configured_map_frame));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "frames.rog_frame", rclcpp::ParameterValue(configured_rog_frame));
  node->get_parameter(prefix + "frames.map_frame", configured_map_frame);
  node->get_parameter(prefix + "frames.rog_frame", configured_rog_frame);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.use_nav2_global_search", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.use_nav2_global_search", priormap_use_nav2_global_search_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.clip_seed_by_rog_boundary", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.clip_seed_by_rog_boundary", priormap_clip_seed_by_rog_boundary_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.rog_boundary_margin", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "priormap.rog_boundary_margin", priormap_rog_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "priormap.rog_boundary_sample_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "priormap.rog_boundary_sample_step", priormap_rog_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.boundary_margin", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "exploration.boundary_margin", exploration_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.boundary_sample_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "exploration.boundary_sample_step", exploration_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.unknown_as_occupied", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.unknown_as_occupied", exploration_unknown_as_occupied_);

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "exploration.prefer_goal_direction", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.prefer_goal_direction", exploration_prefer_goal_direction_);

  std::string configured_global_frame = "map";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "global_frame", rclcpp::ParameterValue(configured_global_frame));
  node->get_parameter(prefix + "global_frame", configured_global_frame);
  global_frame_ = costmap_ros_ ? costmap_ros_->getGlobalFrameID() : configured_global_frame;
  if (!configureRogMap(node, prefix)) {
    ensureMapAvailable();
  }
  initPlannerMode(planner_mode_param, configured_map_frame, configured_rog_frame);

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

  const auto global_query = mode_context_ ? mode_context_->globalQuery() : nullptr;
  const auto dynamic_query = mode_context_ ? mode_context_->dynamicQuery() : nullptr;
  const unsigned int init_size_x = global_query ? global_query->sizeX() : 1U;
  const unsigned int init_size_y = global_query ? global_query->sizeY() : 1U;
  astar_planner_ = std::make_unique<Astar>(init_size_x, init_size_y);
  astar_planner_->setMap(global_query);

  if (use_smac_) {
    smac_planner_ = std::make_unique<minco_planner::smac::SmacPlanner2DSimple>();
    smac_planner_->configure(node, costmap_ros_, prefix);
    smac_planner_->setParameters(allow_unknown_, 1000000, tolerance_);
    smac_planner_->setMap(global_query);
    smac_planner_->setESDFQuery(dynamic_query);
  }

  global_path_searcher_ = std::make_unique<GlobalPathSearcher>();
  global_path_searcher_->configure(
    tf_, astar_planner_.get(), smac_planner_.get(), use_smac_, allow_unknown_, tolerance_, logger_);
  global_path_searcher_->setQuery(global_query);

  local_path_processor_ = std::make_unique<LocalPathProcessor>();
  local_path_processor_->configure(
    lookahead_dist_, minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_, logger_);

  safety_checker_ = std::make_unique<TrajectorySafetyChecker>();
  safety_checker_->configure(minco_config.safe_dist, 0.05, logger_);
  safety_checker_->setQuery(dynamic_query);

  opt_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  backup_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/backup_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!msg) {
        return;
      }

      auto node = node_.lock();
      if (node) {
        planner_perf_monitor_.recordOdomCallback(node->now(), msg->header.stamp);
      }

      {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        latest_odom_ = *msg;
        has_latest_odom_ = true;
      }

    });

  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, output_frame_);

  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);

  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>();
  corridor_gen_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);
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

void MincoPlanner::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  rog_query_raw_ = map;
  rebuildModeDependentQueries();
}

void MincoPlanner::activate()
{
}

void MincoPlanner::deactivate()
{
}

void MincoPlanner::cleanup()
{
  planner_perf_monitor_.close();

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
  smac_planner_.reset();
  global_path_searcher_.reset();
  local_path_processor_.reset();
  safety_checker_.reset();
  mode_context_.reset();
  minco_optimizer_.reset();
  corridor_gen_.reset();
  backup_opt_.reset();
  yaw_opt_.reset();
  opt_path_pub_.reset();
  backup_path_pub_.reset();
  odom_sub_.reset();
  costmap_ros_.reset();
  map_.reset();
  rog_query_raw_.reset();
  rog_map_ros_.reset();
}

rcl_interfaces::msg::SetParametersResult MincoPlanner::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::string planner_mode_param = name_ + ".planner_mode";
  const auto is_configure_time_mode_param = [this, &planner_mode_param](const std::string & param_name) {
    return param_name == planner_mode_param || param_name == name_ + ".frames.map_frame" ||
           param_name == name_ + ".frames.rog_frame" ||
           param_name == name_ + ".priormap.use_nav2_global_search" ||
           param_name == name_ + ".priormap.clip_seed_by_rog_boundary" ||
           param_name == name_ + ".priormap.rog_boundary_margin" ||
           param_name == name_ + ".priormap.rog_boundary_sample_step" ||
           param_name == name_ + ".exploration.boundary_margin" ||
           param_name == name_ + ".exploration.boundary_sample_step" ||
           param_name == name_ + ".exploration.unknown_as_occupied" ||
           param_name == name_ + ".exploration.prefer_goal_direction";
  };
  const std::string max_vel_param = name_ + ".minco_optimizer.max_velocity";
  const std::string max_acc_param = name_ + ".minco_optimizer.max_acceleration";
  const std::string penalty_pos_param = name_ + ".minco_optimizer.penalty_weight_pos";
  const std::string penalty_vel_param = name_ + ".minco_optimizer.penalty_weight_vel";
  const std::string penalty_acc_param = name_ + ".minco_optimizer.penalty_weight_acc";
  const std::string penalty_att_param = name_ + ".minco_optimizer.penalty_weight_att";
  const std::string penalty_time_barrier_param = name_ + ".minco_optimizer.penalty_weight_time_barrier";

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

    if (is_configure_time_mode_param(param_name)) {
      result.successful = false;
      result.reason =
        "Planner mode/frame parameters are configure-time only; restart planner_server to apply.";
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
    if (local_path_processor_) {
      local_path_processor_->updateLimits(minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_);
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

  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

bool MincoPlanner::normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
  const std::string & fallback_frame,
  const std::string & target_frame,
  const std::string & context,
  geometry_msgs::msg::PoseStamped & out) const
{
  out = in;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = fallback_frame;
    RCLCPP_WARN(logger_,
      "[MincoPlanner] %s pose frame is empty, treating it as %s.",
      context.c_str(),
      fallback_frame.c_str());
  }

  if (out.header.frame_id == target_frame) {
    out.header.frame_id = target_frame;
    return true;
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Cannot transform %s pose from %s to %s: TF buffer is null.",
      context.c_str(),
      out.header.frame_id.c_str(),
      target_frame.c_str());
    return false;
  }

  try {
    out = tf_->transform(out, target_frame);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to transform %s pose from %s to %s: %s",
      context.c_str(),
      in.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }
}

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  // Nav2 interface: createPlan() only sets the goal flag for MincoFSM.
  // It must NOT run A* or optimization here.
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = output_frame_;

  geometry_msgs::msg::PoseStamped normalized_start;
  geometry_msgs::msg::PoseStamped normalized_goal;
  const bool start_ok =
    normalizePoseToFrame(start, planning_frame_, planning_frame_, "createPlan start", normalized_start);
  const bool goal_ok =
    normalizePoseToFrame(goal, planning_frame_, planning_frame_, "createPlan goal", normalized_goal);
  if (!start_ok || !goal_ok) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to normalize createPlan pose(s) to planning frame %s; reject pending goal.",
      planning_frame_.c_str());
    std::lock_guard<std::mutex> lk(goal_mutex_);
    has_pending_goal_ = false;
    return path;
  }

  // Keep a minimal path for Nav2 callers (e.g., visualization/debug).
  normalized_start.header = path.header;
  normalized_goal.header = path.header;
  path.poses.push_back(normalized_start);
  path.poses.push_back(normalized_goal);

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    pending_goal_ = normalized_goal;
    has_pending_goal_ = true;
  }

  return path;
}

bool MincoPlanner::PlanGlobalPath(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto search_start = record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  auto record_search_time = [&]() {
    if (!record_perf) {
      return;
    }
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - search_start).count();
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    last_global_search_time_ms_ = elapsed_ms;
    has_fresh_global_search_time_ = true;
  };

  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  std::vector<geometry_msgs::msg::PoseStamped> planned_path;
  if (!global_path_searcher_->plan(start, goal, *mode_context_, planned_path)) {
    record_search_time();
    return false;
  }
  record_search_time();

  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_ = std::move(planned_path);
  return latest_global_path_.size() >= 2U;
}

bool MincoPlanner::ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto replan_start = record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  std::optional<MincoPerfSample> perf;
  if (record_perf) {
    perf.emplace();
    perf->stamp_ros = rclcpp::Clock().now().seconds();
    perf->stamp_steady_ns = PlannerPerformanceMonitor::steadyNowNs();
    perf->planner_mode = mode_params_.planner_mode;
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    if (has_fresh_global_search_time_) {
      perf->global_search_time_ms = last_global_search_time_ms_;
      has_fresh_global_search_time_ = false;
    }
  }
  auto finish = [&](bool success, const std::string & reason) {
    if (perf) {
      perf->success = success;
      perf->failure_reason = success ? "NONE" : reason;
      perf->total_replan_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - replan_start).count();
      {
        std::lock_guard<std::mutex> perf_lock(perf_mutex_);
        if (last_minco_perf_stamp_ns_ > 0 && perf->stamp_steady_ns > last_minco_perf_stamp_ns_) {
          const double dt_sec =
            static_cast<double>(perf->stamp_steady_ns - last_minco_perf_stamp_ns_) * 1.0e-9;
          if (dt_sec > 1.0e-9) {
            perf->planner_hz = 1.0 / dt_sec;
          }
        }
        last_minco_perf_stamp_ns_ = perf->stamp_steady_ns;
      }
      planner_perf_monitor_.recordPlannerSample(*perf);
    }
    return success;
  };

  if (!minco_optimizer_ || !mode_context_ || !local_path_processor_) {
    return finish(false, "OPTIMIZER_FAILED");
  }

  // Snapshot the global goal for end-state logic.
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  double goal_yaw = 0.0;
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (latest_global_path_.empty()) {
      return finish(false, "OPTIMIZER_FAILED");
    }
    global_path_snapshot = latest_global_path_;
    global_goal.x() = latest_global_path_.back().pose.position.x;
    global_goal.y() = latest_global_path_.back().pose.position.y;
    global_goal.z() = 0.0;
    goal_yaw = utils::quaternionToYaw(latest_global_path_.back().pose.orientation);
  }

  const LocalPathSeed seed =
    local_path_processor_->buildSeed(global_path_snapshot, current_pose, *mode_context_);
  if (visualizer_) {
    if (!seed.dense_path.empty()) {
      visualizer_->updateLocalEndPoint(seed.dense_path.back(), seed.local_end_is_goal);
    } else {
      visualizer_->clearLocalEndPoint();
    }
  }
  if (!seed.valid) {
    return finish(false, "COLLISION");
  }
  std::vector<Eigen::Vector3d> sparse_path = seed.sparse_waypoints;
  const bool local_end_is_goal = seed.local_end_is_goal;

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
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
    return finish(false, "RECOVERY_TRIGGERED");
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
    prepareColdStart(current_pose.pose, start_state, sparse_path);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }
  // P1b: If the robot is inside an obstacle (ESDF dist < 0), project the
  // start position to the nearest free space along the ESDF gradient.
  if (safety_checker_) {
    constexpr double kMargin = 0.05;
    Eigen::Vector3d start_pos = start_state.col(0);
    if (safety_checker_->projectOutOfObstacle(start_pos, kMargin)) {
      start_state.col(0) = start_pos;
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
      local_end_vmax = utils::LimitLocalVel(sparse_path,
        sparse_path.size() - 3,
        minco_config.max_vel,
        minco_config.turn_angle_deadzone,
        minco_config.turn_angle_saturation,
        minco_config.min_turn_vel,
        minco_config.decay_power);
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
  const auto opt_start_steady =
    record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  if (perf) {
    perf->local_search_time_ms =
      std::chrono::duration<double, std::milli>(opt_start_steady - replan_start).count();
  }
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double final_cost =
    minco_optimizer_->optimize(sparse_path, start_state, end_state, local_vmaxs, opt_traj);
  if (perf) {
    perf->optimizer_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - opt_start_steady).count();
  }

  // const double max_allowed_cost = 6000.0;
  // if (!std::isfinite(final_cost) || final_cost > max_allowed_cost) {
  if (!std::isfinite(final_cost)) {
    // RCLCPP_WARN(logger_,
    //   "[MincoPlanner] Rejecting new trajectory! Cost (%.2f) exceeds limit (%.2f).",
    //   final_cost,
    //   max_allowed_cost);

    if (visualizer_) {
      visualizer_->clearCandidateTrajectory("OPTIMIZER_FAILED");
    }

    bool has_last_traj = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_last_traj = has_last_traj_;
    }

    if (has_last_traj && isTrajSafe()) {
      if (!isTrajectoryTimeExpired(rclcpp::Clock().now().seconds())) {
        std::cout << YELLOW
                  << "[MincoPlanner] Last trajectory is still valid and safe. Continuing to execute it."
                  << RESET << std::endl;
        return finish(true, "NONE");
      }
      return finish(false, "OPTIMIZER_FAILED");
    }
    return finish(false, "OPTIMIZER_FAILED");
  }

  auto opt_end_time = rclcpp::Clock().now().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  std::cout << GREEN << "[MincoPlanner] Minco optimization time: "
            << opt_duration << " seconds, "
            << "cost: " << final_cost << RESET << std::endl;

  // 8.5 Quality gating (hard validation) before publishing.
  const bool validation_ok = validateTrajectory(opt_traj, end_state.col(0));
  if (visualizer_) {
    visualizer_->updateCandidateTrajectory(opt_traj,
      opt_duration,
      validation_ok,
      validation_ok ? "NONE" : last_validation_failure_reason_);
  }

  if (!validation_ok) {
    std::cout << RED << "[MincoPlanner] Trajectory validation failed! Rejecting." << RESET << std::endl;
    is_traj_safe_.store(false);
    return finish(false, last_validation_failure_reason_);
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
      astar_path_msg.header.frame_id = output_frame_;
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
  return finish(true, "NONE");
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

bool MincoPlanner::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  if (!global_path_searcher_->makePlan(start, goal, *mode_context_, tolerance, cancel_checker, plan)) {
    return false;
  }
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_ = plan.poses;
  return true;
}

std::vector<Eigen::Vector3d> MincoPlanner::extractLocalPath(const Eigen::Vector3d & cur_pos)
{
  if (!local_path_processor_ || !mode_context_) {
    return {};
  }
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.position.x = cur_pos.x();
  current_pose.pose.position.y = cur_pos.y();
  current_pose.pose.position.z = cur_pos.z();
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_snapshot = latest_global_path_;
  }
  return local_path_processor_->buildSeed(global_path_snapshot, current_pose, *mode_context_).dense_path;
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
  last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
  constexpr double kDt = 0.05;
  constexpr double kSevereScale = 1.5;

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid duration." << RESET << std::endl;
    last_validation_failure_reason_ = "OPTIMIZER_FAILED";
    return false;
  }

  const double vmax = minco_config.max_vel;
  const double amax = minco_config.max_acc;
  if (!(std::isfinite(vmax) && std::isfinite(amax) && vmax > 1e-6 && amax > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid vmax/amax config." << RESET
              << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
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
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
    if (v.norm() > vmax_severe || a.norm() > amax_severe) {
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: severe dynamics violation."
                << " |v|=" << v.norm() << " (limit=" << vmax_severe << ")"
                << ", |a|=" << a.norm() << " (limit=" << amax_severe << ")" << RESET << std::endl;
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
  }

  // 2) Goal reachability.
  const Eigen::Vector3d end_pos = traj.getPos(dur);
  if (!end_pos.allFinite()) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite end position." << RESET
              << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double goal_err = (end_pos - expected_end_pos).norm();
  if (!(std::isfinite(goal_err) && goal_err <= traj_goal_tolerance_)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: goal not reached. err=" << goal_err
              << " tol=" << traj_goal_tolerance_ << RESET << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  // 3) Collision safety.
  if (!checkCollision(traj)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: collision detected." << RESET << std::endl;
    last_validation_failure_reason_ = "COLLISION";
    return false;
  }

  return true;
}

bool MincoPlanner::checkCollision()
{
  if (!safety_checker_) {
    return false;
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

  return safety_checker_->checkTrajectory(traj_snapshot);
}

bool MincoPlanner::checkCollision(const traj_opt::Trajectory & traj)
{
  if (!safety_checker_) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  return safety_checker_->checkTrajectory(traj);
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
  header_msg.frame_id = output_frame_;
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
  const bool direct_odom_pose = mode_context_ && mode_context_->directOdomPose();
  if (!direct_odom_pose && costmap_ros_) {
    if (costmap_ros_->getRobotPose(pose)) {
      pose.header.frame_id = planning_frame_;
      return true;
    }
  }

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_latest_odom_) {
      return false;
    }
    odom = latest_odom_;
  }

  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header = odom.header;
  odom_pose.pose = odom.pose.pose;

  if (direct_odom_pose) {
    if (odom_pose.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(logger_,
        *rclcpp::Clock::make_shared(),
        2000,
        "[MincoPlanner] EXPLORATION odom frame is empty, treating it as %s.",
        planning_frame_.c_str());
    }
    odom_pose.header.frame_id = planning_frame_;
    pose = odom_pose;
    return true;
  }

  if (odom_pose.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] PRIORMAP odom frame is empty, treating it as %s before transforming to %s.",
      rog_frame_.c_str(),
      planning_frame_.c_str());
    odom_pose.header.frame_id = rog_frame_;
  }

  if (odom_pose.header.frame_id == planning_frame_ || odom_pose.header.frame_id == map_frame_) {
    odom_pose.header.frame_id = planning_frame_;
    pose = odom_pose;
    return true;
  }

  if (!tf_) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] Cannot transform PRIORMAP odom pose from %s to %s: TF buffer is null.",
      odom_pose.header.frame_id.c_str(),
      planning_frame_.c_str());
    return false;
  }

  try {
    pose = tf_->transform(odom_pose, planning_frame_);
    pose.header.frame_id = planning_frame_;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] Failed to transform odom pose from %s to %s: %s",
      odom_pose.header.frame_id.c_str(),
      planning_frame_.c_str(),
      ex.what());
    return false;
  }
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
      twist.linear.x, twist.linear.y, twist.angular.z, yaw, vx_global, vy_global, omega_global);

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
  return safety_checker_ ? safety_checker_->getDistance(pos) : 0.0;
}

void MincoPlanner::publishEscapeCommand(
  const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel)
{
  if (visualizer_) {
    visualizer_->publishRecoveryDebug(current_pose, escape_vel, 0.5);
  }

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
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
