#include "minco_controller/minco_mpc_controller.hpp"
#include "color_text.hpp"
#include "log.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#ifdef MINCO_DEBUG
#include <chrono>
#include <iomanip>
#endif

#include "nav2_util/node_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace custom_log {
void log_info_line(std::string_view text)
{
  std::cout << text;
}
}  // namespace custom_log

namespace minco_controller {

// === Static helpers ===

double MincoMpcController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double MincoMpcController::interpolateYaw(double yaw1, double yaw2, double alpha)
{
  double diff = std::atan2(std::sin(yaw2 - yaw1), std::cos(yaw2 - yaw1));
  return yaw1 + diff * alpha;
}

double MincoMpcController::interpolate(double v1, double v2, double alpha)
{
  return v1 + (v2 - v1) * alpha;
}

Eigen::Vector2d MincoMpcController::interpolate(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double alpha)
{
  return v1 + (v2 - v1) * alpha;
}

// === Lifecycle ===

void MincoMpcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
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

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  // --- Declare parameters ---

  // Time
  nav2_util::declare_parameter_if_not_declared(node, name + ".dt", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".planner_freq", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_time", rclcpp::ParameterValue(0.5));

  // Physical parameters
  nav2_util::declare_parameter_if_not_declared(node, name + ".mass", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".inertia_z", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".g", rclcpp::ParameterValue(9.81));

  // Friction
  nav2_util::declare_parameter_if_not_declared(node, name + ".mu_c", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, name + ".C_v", rclcpp::ParameterValue(0.5));

  // Force/torque limits
  nav2_util::declare_parameter_if_not_declared(node, name + ".f_max", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".chassis_radius", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, name + ".M_max", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".P_limit", rclcpp::ParameterValue(100.0));

  // Force governor
  nav2_util::declare_parameter_if_not_declared(node, name + ".V_max", rclcpp::ParameterValue(5.0));

  // Cost weights: Q (6D state), R (3D control)
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".Q", rclcpp::ParameterValue(std::vector<double>{10.0, 10.0, 2.0, 1.0, 1.0, 1.0}));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".R", rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));

  // qpOASES solver params
  nav2_util::declare_parameter_if_not_declared(node, name + ".max_wsr", rclcpp::ParameterValue(200));
  nav2_util::declare_parameter_if_not_declared(node, name + ".eps_reg", rclcpp::ParameterValue(1e-9));

  // Controller params
  nav2_util::declare_parameter_if_not_declared(node, name + ".fixed_wz", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".deadzone_speed_threshold", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".control_delay_compensation", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".use_small_gyro_mode", rclcpp::ParameterValue(true));

  // Frames and offsets
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".odom_frame", rclcpp::ParameterValue("camera_init"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".map_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".lidar_roll_offset", rclcpp::ParameterValue(0.0));

  // --- Read parameters ---
  node->get_parameter(name + ".dt", model_config_.dt);
  node->get_parameter(name + ".lookahead_time", lookahead_time_);
  node->get_parameter(name + ".planner_freq", model_config_.planner_freq);
  model_config_.horizon = static_cast<int>(std::ceil(lookahead_time_ / model_config_.dt));

  node->get_parameter(name + ".mass", model_config_.mass);
  node->get_parameter(name + ".inertia_z", model_config_.inertia_z);
  node->get_parameter(name + ".g", model_config_.g);
  node->get_parameter(name + ".mu_c", model_config_.mu_c);
  node->get_parameter(name + ".C_v", model_config_.C_v);
  node->get_parameter(name + ".f_max", model_config_.f_max);
  node->get_parameter(name + ".chassis_radius", model_config_.chassis_radius);
  node->get_parameter(name + ".M_max", model_config_.M_max);
  node->get_parameter(name + ".P_limit", model_config_.P_limit);
  node->get_parameter(name + ".V_max", V_max_);

  std::vector<double> Qv, Rv;
  node->get_parameter(name + ".Q", Qv);
  node->get_parameter(name + ".R", Rv);
  if (Qv.size() == 6) {
    model_config_.Q << Qv[0], Qv[1], Qv[2], Qv[3], Qv[4], Qv[5];
  }
  if (Rv.size() == 3) {
    model_config_.R << Rv[0], Rv[1], Rv[2];
  }

  node->get_parameter(name + ".max_wsr", model_config_.max_wsr);
  node->get_parameter(name + ".eps_reg", model_config_.eps_reg);

  node->get_parameter(name + ".fixed_wz", fixed_wz_);
  node->get_parameter(name + ".deadzone_speed_threshold", deadzone_speed_threshold_);
  node->get_parameter(name + ".control_delay_compensation", control_delay_compensation_);
  node->get_parameter(name + ".use_small_gyro_mode", use_small_gyro_mode_);

  node->get_parameter(name + ".odom_frame", odom_frame_);
  node->get_parameter(name + ".map_frame", map_frame_);
  node->get_parameter(name + ".lidar_offset_x", lidar_offset_x_);
  node->get_parameter(name + ".lidar_offset_y", lidar_offset_y_);
  node->get_parameter(name + ".lidar_roll_offset", lidar_roll_offset_);

  // --- Create modules ---
  syncModelConfig();

  solver_ = std::make_unique<MpcSolver>();
  solver_->setSolverParams(model_config_.max_wsr, model_config_.eps_reg);

  // --- Subscriptions ---
  opt_path_sub_ = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
      "/opt_path", rclcpp::SystemDefaultsQoS(),
      std::bind(&MincoMpcController::onOptPath, this, std::placeholders::_1));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", rclcpp::SystemDefaultsQoS(),
      std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  // --- Publishers ---
  mpc_predict_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_predict_path", 1);
  mpc_real_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_real_path", 1);
  cmd_vel_mpc_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mpc", 1);
  cmd_force_pub_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/cmd_force_mpc", 1);

  real_path_history_.clear();
  last_real_path_pub_time_ = node->now();

  // --- Parameter callback (hot-reload) ---
  param_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&MincoMpcController::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
      "%s: configured (dt=%.3f, horizon=%d, mass=%.1f, f_max=%.1f, M_max=%.1f, P_limit=%.1f, V_max=%.1f, "
      "offset_x=%.3f, offset_y=%.3f)",
      name_.c_str(), model_config_.dt, model_config_.horizon,
      model_config_.mass, model_config_.f_max,
      model_config_.M_max, model_config_.P_limit, V_max_,
      lidar_offset_x_, lidar_offset_y_);
}

void MincoMpcController::syncModelConfig()
{
  model_builder_ = std::make_unique<ModelBuilder>(model_config_);
  model_builder_->initLTIMatrices();
}

void MincoMpcController::cleanup()
{
  opt_path_sub_.reset();
  odom_sub_.reset();
  mpc_predict_path_pub_.reset();
  mpc_real_path_pub_.reset();
  cmd_vel_mpc_pub_.reset();
  cmd_force_pub_.reset();
  param_callback_handle_.reset();
  solver_.reset();
  model_builder_.reset();

  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_opt_path_.reset();
  latest_odom_.reset();
}

void MincoMpcController::activate() {}
void MincoMpcController::deactivate() {}

void MincoMpcController::setPlan(const nav_msgs::msg::Path& path)
{
  std::lock_guard<std::mutex> lk(plan_mtx_);
  global_plan_ = path;
}

void MincoMpcController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

// === Parameter callback ===

rcl_interfaces::msg::SetParametersResult MincoMpcController::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool lti_changed = false;
  bool weights_changed = false;
  bool constraints_changed = false; // 新增：约束参数改变标志位

  for (const auto& param : parameters) {
    const std::string& pname = param.get_name();

    // --- LTI Constructor parameters ---
    if (pname == name_ + ".dt") {
      model_config_.dt = param.as_double();
      model_config_.horizon = static_cast<int>(std::ceil(lookahead_time_ / model_config_.dt));
      lti_changed = true;
    } else if (pname == name_ + ".lookahead_time") {
      lookahead_time_ = param.as_double();
      model_config_.horizon = static_cast<int>(std::ceil(lookahead_time_ / model_config_.dt));
      lti_changed = true;
    } else if (pname == name_ + ".mass") {
      model_config_.mass = param.as_double();
      lti_changed = true;
    } else if (pname == name_ + ".inertia_z") {
      model_config_.inertia_z = param.as_double();
      lti_changed = true;
    }
    // --- Weight parameters ---
    else if (pname == name_ + ".Q") {
      auto qv = param.as_double_array();
      if (qv.size() == 6) {
        model_config_.Q << qv[0], qv[1], qv[2], qv[3], qv[4], qv[5];
        weights_changed = true;
      }
    } else if (pname == name_ + ".R") {
      auto rv = param.as_double_array();
      if (rv.size() == 3) {
        model_config_.R << rv[0], rv[1], rv[2];
        weights_changed = true;
      }
    }
    // --- Constraint parameters ---
    else if (pname == name_ + ".f_max") {
      model_config_.f_max = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".chassis_radius") {
      model_config_.chassis_radius = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".M_max") {
      model_config_.M_max = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".P_limit") {
      model_config_.P_limit = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".mu_c") {
      model_config_.mu_c = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".C_v") {
      model_config_.C_v = param.as_double();
      constraints_changed = true;
    } else if (pname == name_ + ".g") {
      model_config_.g = param.as_double();
      constraints_changed = true;
    }
    // --- Solver and other parameters ---
    else if (pname == name_ + ".max_wsr") {
      model_config_.max_wsr = param.as_int();
      if (solver_) solver_->setSolverParams(model_config_.max_wsr, model_config_.eps_reg);
    } else if (pname == name_ + ".eps_reg") {
      model_config_.eps_reg = param.as_double();
    } else if (pname == name_ + ".V_max") {
      V_max_ = param.as_double();
    }
  }

  if (lti_changed) {
    syncModelConfig();
  } else {
    if (weights_changed && model_builder_) {
      model_builder_->updateCostWeights(model_config_.Q, model_config_.R);
    }
    if (constraints_changed && model_builder_) {
      model_builder_->updateForceLimits(model_config_);
    }
  }

  return result;
}

// === Callbacks ===

void MincoMpcController::onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);

  const uint32_t new_traj_id = (msg && !msg->cmds.empty()) ? msg->cmds.front().trajectory_id : 0u;
  const uint32_t old_traj_id = (latest_opt_path_ && !latest_opt_path_->cmds.empty())
                                   ? latest_opt_path_->cmds.front().trajectory_id
                                   : 0u;

  if (new_traj_id != old_traj_id) {
    has_tracked_ref_ = false;
  }

  latest_opt_path_ = msg;
}

void MincoMpcController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_odom_ = msg;
}

// === State fusion ===

void MincoMpcController::extractAttitudeFromOdom(
    const nav_msgs::msg::Odometry::SharedPtr& odom, Attitude& attitude) const
{
  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  attitude.roll = normalizeAngle(roll - lidar_roll_offset_);
  attitude.pitch = pitch;
  attitude.yaw = yaw;
}

// === Reference trajectory building ===

bool MincoMpcController::transformPathToOdom(
    const ros_interfaces::msg::MpcPositionCommand::SharedPtr& opt,
    std::vector<ros_interfaces::msg::PositionCommand>& out_cmds) const
{
  std::string source_frame = opt->header.frame_id;
  if (source_frame.empty()) {
    source_frame = map_frame_;
    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_WARN_THROTTLE(logger_, *node_ptr->get_clock(), 10000,
          "Received opt_path with empty frame_id, defaulting to '%s'", source_frame.c_str());
    }
  }

  std::string target_frame = global_frame_;
  if (target_frame.empty()) {
    target_frame = odom_frame_;
  }

  if (source_frame == target_frame) {
    out_cmds = opt->cmds;
    return true;
  }

  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    out_cmds = opt->cmds;
    for (auto& cmd : out_cmds) {
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.frame_id = source_frame;
      p_in.point = cmd.position;
      tf2::doTransform(p_in, p_out, transform);
      cmd.position = p_out.point;

      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = source_frame;
      v_in.vector = cmd.velocity;
      tf2::doTransform(v_in, v_out, transform);
      cmd.velocity = v_out.vector;

      geometry_msgs::msg::Vector3Stamped a_in, a_out;
      a_in.header.frame_id = source_frame;
      a_in.vector = cmd.acceleration;
      tf2::doTransform(a_in, a_out, transform);
      cmd.acceleration = a_out.vector;

      geometry_msgs::msg::Vector3Stamped j_in, j_out;
      j_in.header.frame_id = source_frame;
      j_in.vector = cmd.jerk;
      tf2::doTransform(j_in, j_out, transform);
      cmd.jerk = j_out.vector;

      double yaw_diff = tf2::getYaw(transform.transform.rotation);
      cmd.yaw = normalizeAngle(cmd.yaw + yaw_diff);
    }
  } catch (tf2::TransformException& ex) {
    return false;
  }
  return true;
}

bool MincoMpcController::buildReferenceFromOptPath(
    const State& curr, std::vector<ReferencePoint>& out_ref) const
{
  auto node = node_.lock();
  if (!node) return false;
  const rclcpp::Time now = node->now();

  ros_interfaces::msg::MpcPositionCommand::SharedPtr opt;
  double tracked_ref_idx = 0.0;
  rclcpp::Time tracked_ref_time;
  uint32_t tracked_opt_traj_id = 0u;
  bool has_tracked_ref = false;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    opt = latest_opt_path_;
    tracked_ref_idx = tracked_ref_idx_;
    tracked_ref_time = tracked_ref_time_;
    tracked_opt_traj_id = tracked_opt_traj_id_;
    has_tracked_ref = has_tracked_ref_;
  }

  if (!opt || opt->cmds.empty()) return false;

  std::vector<ros_interfaces::msg::PositionCommand> cmds;
  if (!transformPathToOdom(opt, cmds)) return false;

  const size_t n_cmds = cmds.size();
  const double planner_dt = 1.0 / model_config_.planner_freq;
  if (planner_dt <= 1.0e-6) return false;

  // Nearest point search
  size_t best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n_cmds; ++i) {
    const double dx = cmds[i].position.x - curr.px;
    const double dy = cmds[i].position.y - curr.py;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  double nearest_idx_float = static_cast<double>(best_idx);

  if (best_idx < n_cmds - 1) {
    const auto& p_curr = cmds[best_idx];
    const auto& p_next = cmds[best_idx + 1];
    Eigen::Vector2d a_vec(p_next.position.x - p_curr.position.x,
                          p_next.position.y - p_curr.position.y);
    Eigen::Vector2d b_vec(curr.px - p_curr.position.x, curr.py - p_curr.position.y);
    double len_sq = a_vec.squaredNorm();
    if (len_sq > 1e-6) {
      double projection = a_vec.dot(b_vec) / len_sq;
      if (projection > -0.5 && projection < 1.0) {
        nearest_idx_float += projection;
      }
    }
  }

  nearest_idx_float = std::max(0.0, nearest_idx_float);

  const uint32_t current_traj_id = (!opt->cmds.empty()) ? opt->cmds.front().trajectory_id : 0u;
  double current_idx_float = nearest_idx_float;
  current_idx_float = std::min(current_idx_float, static_cast<double>(n_cmds - 1));
  // double time_tracked = (now.seconds() - tracked_ref_time_.seconds());
  // if(time_tracked > 0.0 && std::hypot(curr.vx, curr.vy) < 0.05)
  // {
  //   double forced_idx = (time_tracked / planner_dt) * 0.5;
  //   current_idx_float = std::max(current_idx_float, tracked_ref_idx + forced_idx);
  // }
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    tracked_ref_idx_ = current_idx_float;
    tracked_ref_time_ = now;
    tracked_opt_traj_id_ = current_traj_id;
    has_tracked_ref_ = true;
  }

  const double current_traj_time = current_idx_float * planner_dt;

  const int N = model_config_.horizon;
  const double mpc_dt = model_config_.dt;
  out_ref.clear();
  out_ref.reserve(static_cast<size_t>(N));

  for (int k = 0; k < N; ++k) {
    double target_time = current_traj_time + k * mpc_dt;
    double target_idx_float = target_time / planner_dt;

    if (target_idx_float < 0.0) target_idx_float = 0.0;
    if (target_idx_float > static_cast<double>(n_cmds - 1)) {
      target_idx_float = static_cast<double>(n_cmds - 1);
    }

    size_t target_idx = static_cast<size_t>(std::floor(target_idx_float));
    size_t next_idx = target_idx + 1;
    double alpha = target_idx_float - static_cast<double>(target_idx);

    ReferencePoint rp;
    if (next_idx < n_cmds) {
      const double dt_interp = std::max(0.0, std::min(planner_dt, alpha * planner_dt));
      const double dt2 = dt_interp * dt_interp;
      const double dt3 = dt2 * dt_interp;

      const Eigen::Vector2d p_i(cmds[target_idx].position.x, cmds[target_idx].position.y);
      const Eigen::Vector2d v_i(cmds[target_idx].velocity.x, cmds[target_idx].velocity.y);
      const Eigen::Vector2d a_i(cmds[target_idx].acceleration.x, cmds[target_idx].acceleration.y);
      const Eigen::Vector2d j_i(cmds[target_idx].jerk.x, cmds[target_idx].jerk.y);

      rp.pos = p_i + v_i * dt_interp + 0.5 * a_i * dt2 + (1.0 / 6.0) * j_i * dt3;
      rp.vel = v_i + a_i * dt_interp + 0.5 * j_i * dt2;
      // acc: interpolate linearly (jerk is constant over segment)
      rp.acc = a_i + j_i * dt_interp;

      rp.yaw = interpolateYaw(cmds[target_idx].yaw, cmds[next_idx].yaw, alpha);
      if (!use_small_gyro_mode_ && rp.vel.norm() > 0.2) {
        double tangent_yaw = std::atan2(rp.vel.y(), rp.vel.x());
        double diff = std::atan2(std::sin(tangent_yaw - rp.yaw), std::cos(tangent_yaw - rp.yaw));
        // 将参考 Yaw 强行向物理运动方向拉近，消除多项式偏差
        rp.yaw = rp.yaw + diff * 0.85; 
      }
      rp.yaw_rate = interpolate(cmds[target_idx].yaw_dot, cmds[next_idx].yaw_dot, alpha);
      // yaw_acc: finite difference of yaw_dot
      rp.yaw_acc = (cmds[next_idx].yaw_dot - cmds[target_idx].yaw_dot) * model_config_.planner_freq;
    } else {
      const auto& p_end = cmds.back();
      rp.pos = Eigen::Vector2d(p_end.position.x, p_end.position.y);
      rp.vel = Eigen::Vector2d(0.0, 0.0);
      rp.acc = Eigen::Vector2d(0.0, 0.0);
      rp.yaw = p_end.yaw;
      rp.yaw_rate = 0.0;
      rp.yaw_acc = 0.0;
    }
    out_ref.push_back(rp);
  }

  // Yaw continuity
  if (!out_ref.empty()) {
    double diff = out_ref[0].yaw - curr.yaw;
    diff = std::atan2(std::sin(diff), std::cos(diff));
    out_ref[0].yaw = curr.yaw + diff;

    for (size_t i = 1; i < out_ref.size(); ++i) {
      double d_yaw = out_ref[i].yaw - out_ref[i - 1].yaw;
      d_yaw = std::atan2(std::sin(d_yaw), std::cos(d_yaw));
      out_ref[i].yaw = out_ref[i - 1].yaw + d_yaw;
    }
  }

  return true;
}

// === Main control loop ===

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
  auto node = node_.lock();
  const rclcpp::Time now = node->now();

  nav_msgs::msg::Odometry::SharedPtr latest_odom;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    latest_odom = latest_odom_;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = global_frame_;

  // Goal check
  if (goal_checker != nullptr) {
    geometry_msgs::msg::PoseStamped goal_pose;
    bool has_goal_pose = false;
    {
      std::lock_guard<std::mutex> lk(plan_mtx_);
      if (!global_plan_.poses.empty()) {
        goal_pose = global_plan_.poses.back();
        has_goal_pose = true;
      }
    }
    if (has_goal_pose && goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
      std::lock_guard<std::mutex> lk(data_mtx_);
      has_tracked_ref_ = false;
      latest_opt_path_.reset();
      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;
      cmd.twist.angular.z = 0.0;
      return cmd;
    }
  }

  // 1) State fusion: 6D state in map frame
  State curr;

  // 1a. Position from Nav2 pose
  curr.px = pose.pose.position.x;
  curr.py = pose.pose.position.y;

  if (latest_odom) {
    // 1b. Attitude from radar odometry
    Attitude attitude;
    extractAttitudeFromOdom(latest_odom, attitude);
    curr.yaw = attitude.yaw;

    // 1c. Extract lidar-frame velocity
    const double v_lx = latest_odom->twist.twist.linear.x;
    const double v_ly = latest_odom->twist.twist.linear.y;
    const double omega = latest_odom->twist.twist.angular.z;

    // 1d. Lever arm compensation: lidar center -> base_link center (body frame)
    const double v_body_x = v_lx + omega * lidar_offset_y_;
    const double v_body_y = v_ly - omega * lidar_offset_x_;

    // 1e. Rotate base_link velocity to map frame
    const double cos_yaw = std::cos(curr.yaw);
    const double sin_yaw = std::sin(curr.yaw);
    curr.vx = v_body_x * cos_yaw - v_body_y * sin_yaw;
    curr.vy = v_body_x * sin_yaw + v_body_y * cos_yaw;
    curr.omega = omega;

    // Noise deadzone
    const double noise_threshold = 0.03;
    if (std::abs(curr.vx) < noise_threshold) curr.vx = 0.0;
    if (std::abs(curr.vy) < noise_threshold) curr.vy = 0.0;
    if (std::abs(curr.omega) < noise_threshold) curr.omega = 0.0;
  } else {
    curr.yaw = normalizeAngle(tf2::getYaw(pose.pose.orientation));
    curr.vx = 0.0;
    curr.vy = 0.0;
    curr.omega = 0.0;
  }

  // 2) Build reference trajectory (no state advancement — delay handled at control extraction)
  std::vector<ReferencePoint> ref;
  if (!buildReferenceFromOptPath(curr, ref) || !model_builder_ || !solver_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // 3) Extract attitude for gravity projection
  Attitude attitude;
  attitude.yaw = curr.yaw;
  attitude.roll = 0.0;
  attitude.pitch = 0.0;
  if (latest_odom) {
    extractAttitudeFromOdom(latest_odom, attitude);
  }

  // 4) Build QP via ModelBuilder
  QPProblem qp;
#ifdef MINCO_DEBUG
  auto t_start = std::chrono::high_resolution_clock::now();
#endif

  if (!model_builder_->buildQP(curr.asVector(), attitude, ref, qp)) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // 5) Solve via MpcSolver (extract delay_idx-th control for delay compensation)
  const int delay_idx = static_cast<int>(std::round(control_delay_compensation_ / model_config_.dt));
  Eigen::Vector3d optimal_force;
  if (!solver_->solve(qp, optimal_force, delay_idx)) {
    std::cout << color_text::RED << "[MincoMpc] Solver Failed!" << color_text::RESET << std::endl;
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

#ifdef MINCO_DEBUG
  auto t_end = std::chrono::high_resolution_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  custom_log::log_block(std::string("\033[34m[MincoMpc] "),
      NV(optimal_force(0)), NV(optimal_force(1)), NV(optimal_force(2)),
      NV(dt_ms));
#endif

  // 6) Force Governor: predict velocity → clamp to V_max → back-compute safe force
  const double mass_inv = 1.0 / model_config_.mass;
  const double iz_inv = 1.0 / model_config_.inertia_z;
  const double dt = model_config_.dt;

  double vx_pred = curr.vx + optimal_force(0) * mass_inv * dt;
  double vy_pred = curr.vy + optimal_force(1) * mass_inv * dt;
  const double v_pred_norm = std::hypot(vx_pred, vy_pred);

  if (v_pred_norm > V_max_ && v_pred_norm > 1e-6) {
    const double scale = V_max_ / v_pred_norm;
    vx_pred *= scale;
    vy_pred *= scale;
    optimal_force(0) = (vx_pred - curr.vx) * model_config_.mass / dt;
    optimal_force(1) = (vy_pred - curr.vy) * model_config_.mass / dt;
  }

  const double v_cmd_wz = curr.omega + optimal_force(2) * iz_inv * dt;

  // 7) Post-solve coupled polytope safety clip
  ModelBuilder::clampCoupledForce(optimal_force, model_config_.f_max, model_config_.chassis_radius);

  // 7.5) Low-pass filter to prevent stick-slip force oscillation on slopes.
  // The MPC re-optimizes every cycle; adjacent QP solutions can differ
  // significantly on inclines where gravity creates a large steady-state
  // disturbance. Smoothing the force command breaks the move-stop cycle.
  if (prev_force_.squaredNorm() > 1e-12) {
    optimal_force = force_smooth_alpha_ * optimal_force + (1.0 - force_smooth_alpha_) * prev_force_;
  }
  prev_force_ = optimal_force;

  // 8) Publish force command on /cmd_force_mpc
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = now;
  wrench_msg.header.frame_id = map_frame_;
  wrench_msg.wrench.force.x = optimal_force(0);
  wrench_msg.wrench.force.y = optimal_force(1);
  wrench_msg.wrench.torque.z = optimal_force(2);
  cmd_force_pub_->publish(wrench_msg);

  // 9) Publish /cmd_vel_mpc as fallback
  if (cmd_vel_mpc_pub_) {
    geometry_msgs::msg::Twist diag_cmd;
    diag_cmd.linear.x = vx_pred;
    diag_cmd.linear.y = vy_pred;
    diag_cmd.angular.z = v_cmd_wz;
    cmd_vel_mpc_pub_->publish(diag_cmd);
  }

  // 10) Return TwistStamped for Nav2
  cmd.twist.linear.x = vx_pred;
  cmd.twist.linear.y = vy_pred;
  cmd.twist.angular.z = v_cmd_wz;

  // Speed limit
  if (speed_limit_ > 1e-6) {
    const double v_norm = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
    if (v_norm > 1e-6) {
      double scale = speed_limit_percentage_ ? std::clamp(speed_limit_ / 100.0, 0.0, 1.0)
                                             : std::min(1.0, speed_limit_ / v_norm);
      cmd.twist.linear.x *= scale;
      cmd.twist.linear.y *= scale;
    }
  }

  // Deadzone
  if (std::hypot(cmd.twist.linear.x, cmd.twist.linear.y) < deadzone_speed_threshold_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
  }

  // Small gyro mode
  if (use_small_gyro_mode_) {
    cmd.twist.angular.z = fixed_wz_;
  }

  // Publish visualization
  publishVisualization(curr);

  return cmd;
}

// === Visualization ===

void MincoMpcController::publishVisualization(const State& curr_state)
{
  auto node = node_.lock();
  if (!node) return;

  rclcpp::Time now = node->now();

  // Publish actual path
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now;
  ps.header.frame_id = global_frame_;
  ps.pose.position.x = curr_state.px;
  ps.pose.position.y = curr_state.py;
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, curr_state.yaw);
  ps.pose.orientation = tf2::toMsg(q);

  if (real_path_history_.empty()) {
    real_path_history_.push_back(ps);
  } else {
    const auto& last = real_path_history_.back();
    double dist = std::hypot(last.pose.position.x - ps.pose.position.x,
                             last.pose.position.y - ps.pose.position.y);
    if (dist > 0.02) {
      real_path_history_.push_back(ps);
    }
  }

  if (real_path_history_.size() > 5000) {
    size_t remove_count = real_path_history_.size() - 5000;
    real_path_history_.erase(real_path_history_.begin(), real_path_history_.begin() + remove_count);
  }

  if (mpc_real_path_pub_ && mpc_real_path_pub_->get_subscription_count() > 0) {
    if ((now - last_real_path_pub_time_).seconds() > 1.0) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now;
      path_msg.header.frame_id = global_frame_;
      path_msg.poses = real_path_history_;
      mpc_real_path_pub_->publish(path_msg);
      last_real_path_pub_time_ = now;
    }
  }
}

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::MincoMpcController, nav2_core::Controller)
