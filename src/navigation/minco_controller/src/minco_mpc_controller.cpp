#include "minco_controller/minco_mpc_controller.hpp"
#include "log.hpp"
#include "color_text.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <limits>

#ifdef MINCO_DEBUG
#include <iostream>
#include <chrono>
#include <iomanip>
#endif

#include "nav2_util/node_utils.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace custom_log
{
  void log_info_line(std::string_view text)
  {
    std::cout << text;
  }
}

namespace minco_controller
{

static Eigen::Vector2d rotate2d(const Eigen::Vector2d & v, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}

double MincoMpcController::normalizeYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

void MincoMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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

  // 声明/读取参数
  nav2_util::declare_parameter_if_not_declared(node, name + ".dt", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".horizon", rclcpp::ParameterValue(10));

  nav2_util::declare_parameter_if_not_declared(node, name + ".Q", rclcpp::ParameterValue(std::vector<double>{5.0, 5.0, 2.0}));
  nav2_util::declare_parameter_if_not_declared(node, name + ".R", rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));

  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_max", rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(node, name + ".use_acc_constraints", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_min", rclcpp::ParameterValue(-4.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_max", rclcpp::ParameterValue(4.0));

  nav2_util::declare_parameter_if_not_declared(node, name + ".odom_frame", rclcpp::ParameterValue("camera_init"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".map_frame", rclcpp::ParameterValue("map"));

  double dt = 0.05;
  double lookahead_time = 0.5;
  node->get_parameter(name + ".dt", dt);
  node->get_parameter(name + ".lookahead_time", lookahead_time);

  std::vector<double> Qv;
  std::vector<double> Rv;
  node->get_parameter(name + ".Q", Qv);
  node->get_parameter(name + ".R", Rv);

  mpc_config_.dt = dt;
  mpc_config_.lookahead_time = lookahead_time;
  mpc_config_.horizon = static_cast<int>(std::ceil(lookahead_time / dt));
  if (Qv.size() == 3) {
    mpc_config_.Q = Eigen::Vector3d(Qv[0], Qv[1], Qv[2]);
  }
  if (Rv.size() == 3) {
    mpc_config_.R = Eigen::Vector3d(Rv[0], Rv[1], Rv[2]);
  }

  node->get_parameter(name + ".vx_min", mpc_config_.vx_min);
  node->get_parameter(name + ".vx_max", mpc_config_.vx_max);
  node->get_parameter(name + ".vy_min", mpc_config_.vy_min);
  node->get_parameter(name + ".vy_max", mpc_config_.vy_max);
  node->get_parameter(name + ".omega_min", mpc_config_.omega_min);
  node->get_parameter(name + ".omega_max", mpc_config_.omega_max);

  node->get_parameter(name + ".use_acc_constraints", mpc_config_.use_acc_constraints);
  node->get_parameter(name + ".ax_min", mpc_config_.ax_min);
  node->get_parameter(name + ".ax_max", mpc_config_.ax_max);
  node->get_parameter(name + ".ay_min", mpc_config_.ay_min);
  node->get_parameter(name + ".ay_max", mpc_config_.ay_max);
  node->get_parameter(name + ".alpha_min", mpc_config_.alpha_min);
  node->get_parameter(name + ".alpha_max", mpc_config_.alpha_max);

  node->get_parameter(name + ".odom_frame", odom_frame_);
  node->get_parameter(name + ".map_frame", map_frame_);

  solver_ = std::make_unique<MpcSolver>(mpc_config_);

  // 订阅优化轨迹：/opt_path
  opt_path_sub_ = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOptPath, this, std::placeholders::_1));

  // 订阅里程计：用于延迟补偿
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  mpc_predict_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_predict_path", 1);
  mpc_real_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_real_path", 1);
  real_path_history_.clear();
  last_real_path_pub_time_ = node->now();

  RCLCPP_INFO(logger_, "%s: MincoMpcController configured (dt=%.3f, lookahead_time=%.3f)", name_.c_str(), dt, lookahead_time);
}

void MincoMpcController::cleanup()
{
  opt_path_sub_.reset();
  odom_sub_.reset();
  mpc_predict_path_pub_.reset();
  mpc_real_path_pub_.reset();
  solver_.reset();

  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_opt_path_.reset();
  latest_odom_.reset();
}

void MincoMpcController::activate()
{
}

void MincoMpcController::deactivate()
{
}

void MincoMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lk(plan_mtx_);
  global_plan_ = path;
}

void MincoMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

void MincoMpcController::onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_opt_path_ = msg;
}

void MincoMpcController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_odom_ = msg;
}

bool MincoMpcController::transformPathToOdom(
  const ros_interfaces::msg::MpcPositionCommand::SharedPtr & opt,
  std::vector<ros_interfaces::msg::PositionCommand> & out_cmds) const
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
    geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);

    out_cmds = opt->cmds; // Copy
    for (auto & cmd : out_cmds) {
      // Transform position
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.frame_id = source_frame;
      p_in.point = cmd.position;
      tf2::doTransform(p_in, p_out, transform);
      cmd.position = p_out.point;

      // Transform velocity (vector)
      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = source_frame;
      v_in.vector = cmd.velocity;
      tf2::doTransform(v_in, v_out, transform);
      cmd.velocity = v_out.vector;

      // Transform acceleration (vector)
      geometry_msgs::msg::Vector3Stamped a_in, a_out;
      a_in.header.frame_id = source_frame;
      a_in.vector = cmd.acceleration;
      tf2::doTransform(a_in, a_out, transform);
      cmd.acceleration = a_out.vector;

      // Transform jerk (vector)
      geometry_msgs::msg::Vector3Stamped j_in, j_out;
      j_in.header.frame_id = source_frame;
      j_in.vector = cmd.jerk;
      tf2::doTransform(j_in, j_out, transform);
      cmd.jerk = j_out.vector;

      // Transform yaw
      double yaw_diff = tf2::getYaw(transform.transform.rotation);
      cmd.yaw = normalizeYaw(cmd.yaw + yaw_diff);
    }

    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_INFO_THROTTLE(logger_, *node_ptr->get_clock(), 5000,
        "Transformed opt_path from %s to %s", source_frame.c_str(), target_frame.c_str());
    }

  } catch (tf2::TransformException & ex) {
    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_WARN_THROTTLE(logger_, *node_ptr->get_clock(), 2000,
        "Could not transform opt_path from %s to %s: %s", 
        source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return false;
  }
  return true;
}

bool MincoMpcController::buildReferenceFromOptPath(const State & curr, std::vector<ReferencePoint> & out_ref) const
{
  ros_interfaces::msg::MpcPositionCommand::SharedPtr opt;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    opt = latest_opt_path_;
  }

  if (!opt || opt->cmds.empty()) {
    return false;
  }

  // 坐标系转换处理
  std::vector<ros_interfaces::msg::PositionCommand> cmds;
  if (!transformPathToOdom(opt, cmds)) {
    return false;
  }

  const size_t n_cmds = cmds.size();
  const double planner_dt = 1.0 / mpc_config_.planner_freq;
  if (planner_dt <= 1.0e-6) {
    return false;
  }

  // ===== 原“时间投影”方式（按需求仅注释，不删除） =====
  // auto node = node_.lock();
  // if (!node) {
  //   return false;
  // }
  // double t_pass = (node->now() - opt->header.stamp).seconds();
  // t_pass = std::max(0.0, t_pass);
  // const double current_idx_float = t_pass / planner_dt;

  // ===== 改为最近点搜索 =====
  size_t best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n_cmds; ++i) {
    const double dx = cmds[i].position.x - curr.x;
    const double dy = cmds[i].position.y - curr.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  double current_idx_float = static_cast<double>(best_idx);

  // ===== 原“投影到下一段”方式（按需求仅注释，不删除） =====
  if (best_idx < n_cmds - 1) {
    const auto & p_curr = cmds[best_idx];
    const auto & p_next = cmds[best_idx + 1];
  
    Eigen::Vector2d a_vec(
      p_next.position.x - p_curr.position.x,
      p_next.position.y - p_curr.position.y);
    Eigen::Vector2d b_vec(
      curr.x - p_curr.position.x,
      curr.y - p_curr.position.y);
  
    double len_sq = a_vec.squaredNorm();
    if (len_sq > 1e-6) {
      double projection = a_vec.dot(b_vec) / len_sq;
      if (projection > -0.5 && projection < 1.0) {
        current_idx_float += projection;
      }
    }
  }

  current_idx_float = std::max(0.0, current_idx_float);

  double current_traj_time = current_idx_float * planner_dt;

  // 让 MPC 始终去追踪未来 0.15 秒的参考点，提前应对弯道，消除系统延迟感
  double lookahead_time = 0.15;
  current_traj_time += lookahead_time;

  const int N = mpc_config_.horizon;
  const double mpc_dt = mpc_config_.dt;
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
    if (next_idx < n_cmds)
    {
      // Second-order feed-forward interpolation (Taylor expansion) using P/V/A/J.
      // We use the left knot (target_idx) as expansion point.
      const double dt = std::max(0.0, std::min(planner_dt, alpha * planner_dt));
      const double dt2 = dt * dt;
      const double dt3 = dt2 * dt;

      const Eigen::Vector2d p_i(cmds[target_idx].position.x, cmds[target_idx].position.y);
      const Eigen::Vector2d v_i(cmds[target_idx].velocity.x, cmds[target_idx].velocity.y);
      const Eigen::Vector2d a_i(cmds[target_idx].acceleration.x, cmds[target_idx].acceleration.y);
      const Eigen::Vector2d j_i(cmds[target_idx].jerk.x, cmds[target_idx].jerk.y);

      rp.pos = p_i + v_i * dt + 0.5 * a_i * dt2 + (1.0 / 6.0) * j_i * dt3;
      rp.vel = v_i + a_i * dt + 0.5 * j_i * dt2;
      rp.yaw = interpolateYaw(cmds[target_idx].yaw, cmds[next_idx].yaw, alpha);
      rp.yaw_rate = interpolate(
        cmds[target_idx].yaw_dot,
        cmds[next_idx].yaw_dot,
        alpha);
    } else {
      const auto & p_end = cmds.back();
      rp.pos = Eigen::Vector2d(p_end.position.x, p_end.position.y);
      rp.vel = Eigen::Vector2d(0.0, 0.0);
      rp.yaw = p_end.yaw;
      rp.yaw_rate = 0.0;
    }
    out_ref.push_back(rp);
  }

  if (!out_ref.empty()) {
    // 1. 先把第一个点的 yaw 对齐到 curr.yaw 附近
    double diff = out_ref[0].yaw - curr.yaw;
    // 将 diff 限制在 [-PI, PI]
    diff = std::atan2(std::sin(diff), std::cos(diff));
    out_ref[0].yaw = curr.yaw + diff;

    // 2. 后续点相对于前一个点进行平滑连续化
    for (size_t i = 1; i < out_ref.size(); ++i) {
      double d_yaw = out_ref[i].yaw - out_ref[i - 1].yaw;
      d_yaw = std::atan2(std::sin(d_yaw), std::cos(d_yaw));
      out_ref[i].yaw = out_ref[i - 1].yaw + d_yaw;
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();

  const rclcpp::Time now = node->now();
  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_for_base_transform = yaw_pose;

  State curr;
  curr.x = pose.pose.position.x;
  curr.y = pose.pose.position.y;
  curr.yaw = normalizeYaw(yaw_pose);
  curr.vx = 0.0;
  curr.vy = 0.0;
  curr.omega = 0.0;

  // 2) 构造参考序列：优先 /opt_path
  std::vector<ReferencePoint> ref;
  bool ok_ref = buildReferenceFromOptPath(curr, ref);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = base_frame_;

  if (!ok_ref || !solver_) {
    // 无参考则输出 0
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // 3) 调用 MPC 求解（输出 global_frame_ 系速度，此处为 camera_init）
  Control u_global;
  std::vector<State> pred_states;
#ifdef MINCO_DEBUG
  auto t_start = std::chrono::high_resolution_clock::now();
#endif
  // std::cout << color_text::BLUE << "[MincoMpc] Start Solving..." << color_text::RESET << std::endl;
  bool success = solver_->solve(curr, ref, u_global, &pred_states);
  // std::cout << color_text::BLUE << "[MincoMpc] Solve Finished." << color_text::RESET << std::endl;
  
  publishVisualization(pred_states, curr);

#ifdef MINCO_DEBUG
  auto t_end = std::chrono::high_resolution_clock::now();
  if (success && !ref.empty()) {
    double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    double plan_vx = u_global.vx;
    double plan_vy = u_global.vy;
    double ref_vx = ref[0].vel.x();
    double ref_vy = ref[0].vel.y();
    double v_err_x = plan_vx - ref_vx;
    double v_err_y = plan_vy - ref_vy;

    double curr_x = curr.x;
    double curr_y = curr.y;
    double ref_x = ref[0].pos.x();
    double ref_y = ref[0].pos.y();
    double p_err_x = curr_x - ref_x;
    double p_err_y = curr_y - ref_y;

    custom_log::log_block(std::string("\033[34m[MincoMpc] "),
      NV(plan_vx), NV(plan_vy),
      NV(ref_vx), NV(ref_vy),
      NV(v_err_x), NV(v_err_y),
      NV(curr_x), NV(curr_y),
      NV(ref_x), NV(ref_y),
      NV(p_err_x), NV(p_err_y),
      NV(dt_ms));
  }
#endif

  if (!success) {
    std::cout << color_text::RED << "[MincoMpc] Solver Failed!" << color_text::RESET << std::endl;
    // Debug info for failure
    if (!ref.empty()) {
        std::cout << "[MincoMpc] Ref Size: " << ref.size() << std::endl;
        std::cout << "[MincoMpc] Curr: " << curr.x << ", " << curr.y << ", " << curr.yaw << std::endl;
        std::cout << "[MincoMpc] Ref[0]: " << ref[0].pos.x() << ", " << ref[0].pos.y() << ", " << ref[0].yaw << std::endl;
    }

    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }
  // std::cout << color_text::GREEN << "[MincoMpc] Solver Success!" << color_text::RESET << std::endl;
  // 4) 将全局控制律 [vx, vy, omega] 转换为机器人坐标系 (base)
  // 使用 odom 提供的 yaw（若无 odom 则回退到 pose yaw）。
  const Eigen::Vector2d u_global_v(u_global.vx, u_global.vy);
  const Eigen::Vector2d u_base_v = rotate2d(u_global_v, -yaw_for_base_transform);

  double vx = u_base_v.x();
  double vy = u_base_v.y();
  double wz = u_global.omega;

  // 5) 处理 Nav2 setSpeedLimit
  if (speed_limit_ > 1e-6) {
    const double v_norm = std::hypot(vx, vy);
    if (v_norm > 1e-6) {
      double scale = 1.0;
      if (speed_limit_percentage_) {
        scale = std::clamp(speed_limit_ / 100.0, 0.0, 1.0);
      } else {
        scale = std::min(1.0, speed_limit_ / v_norm);
      }
      vx *= scale;
      vy *= scale;
    }
  }

  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.angular.z = wz;
  return cmd;
}

void MincoMpcController::publishVisualization(const std::vector<State> & pred_path, const State & curr_state)
{
  auto node = node_.lock();
  if (!node) return;
  
  rclcpp::Time now = node->now();

  // 1. 发布预测路径
  if (mpc_predict_path_pub_ && mpc_predict_path_pub_->get_subscription_count() > 0 && !pred_path.empty()) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = global_frame_; 

    for (const auto & s : pred_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      ps.pose.position.z = 0.0;
      
      tf2::Quaternion q;
      q.setRPY(0, 0, s.yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path_msg.poses.push_back(ps);
    }
    mpc_predict_path_pub_->publish(path_msg);
  }

  // 2. 发布实际路径
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now;
  ps.header.frame_id = global_frame_;
  ps.pose.position.x = curr_state.x;
  ps.pose.position.y = curr_state.y;
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, curr_state.yaw);
  ps.pose.orientation = tf2::toMsg(q);

  if (real_path_history_.empty()) {
    real_path_history_.push_back(ps);
  } else {
    const auto & last = real_path_history_.back();
    double dist = std::hypot(last.pose.position.x - ps.pose.position.x, 
                             last.pose.position.y - ps.pose.position.y);
    // 简单的距离过滤，避免原地不动时数据堆积
    if (dist > 0.02) { 
      real_path_history_.push_back(ps);
    }
  }

  // 保持历史长度
  if (real_path_history_.size() > 5000) {
    size_t remove_count = real_path_history_.size() - 5000;
    real_path_history_.erase(real_path_history_.begin(), real_path_history_.begin() + remove_count);
  }

  if (mpc_real_path_pub_ && mpc_real_path_pub_->get_subscription_count() > 0) {
    // 降频发布：1.0 Hz
    if ((now - last_real_path_pub_time_).seconds() > 1.0) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now;
        path_msg.header.frame_id = global_frame_;
        // 如果历史太长，可以只发布最近的一部分，或者对历史进行下采样（本例直接发布全部，但频率低）
        path_msg.poses = real_path_history_;
        mpc_real_path_pub_->publish(path_msg);
        last_real_path_pub_time_ = now;
    }
  }
}

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::MincoMpcController, nav2_core::Controller)
