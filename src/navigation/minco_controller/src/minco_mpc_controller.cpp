#include "minco_controller/minco_mpc_controller.hpp"
#include "log.hpp"

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
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
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

  solver_ = std::make_unique<MpcSolver>(mpc_config_);

  // 订阅优化轨迹：/opt_path
  opt_path_sub_ = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOptPath, this, std::placeholders::_1));

  // 订阅里程计：用于延迟补偿
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "%s: MincoMpcController configured (dt=%.3f, lookahead_time=%.3f)", name_.c_str(), dt, lookahead_time);
}

void MincoMpcController::cleanup()
{
  opt_path_sub_.reset();
  odom_sub_.reset();
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

  const auto & cmds = opt->cmds;
  const size_t n_cmds = cmds.size();
  // 最近点搜索
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
  
  // 尝试向后投影 (best_idx -> best_idx + 1)
  if (best_idx < n_cmds - 1) {
    const auto & p_curr = cmds[best_idx];
    const auto & p_next = cmds[best_idx + 1];
    
    // 向量 A: 轨迹段向量
    Eigen::Vector2d a_vec(
      p_next.position.x - p_curr.position.x,
      p_next.position.y - p_curr.position.y);
    // 向量 B: 机器人相对于 p_curr 的向量
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

  double planner_dt = 1 / mpc_config_.planner_freq;
  double current_traj_time = current_idx_float * planner_dt;
  const int N = mpc_config_.horizon;
  const double mpc_dt = mpc_config_.dt;
  out_ref.clear();
  out_ref.reserve(static_cast<size_t>(N));

  for (int k = 0; k < N; ++k) {
    double target_time = current_traj_time + k * mpc_dt;
    double target_idx_float = target_time / planner_dt; 
    size_t target_idx = static_cast<size_t>(std::floor(target_idx_float)); 
    size_t next_idx = target_idx + 1;
    double alpha = target_idx_float - static_cast<double>(target_idx);

    ReferencePoint rp;
    if (next_idx < n_cmds)
    {
      rp.pos = interpolate(
        Eigen::Vector2d(opt->cmds[target_idx].position.x, opt->cmds[target_idx].position.y),
        Eigen::Vector2d(opt->cmds[next_idx].position.x, opt->cmds[next_idx].position.y),
        alpha);
      rp.vel = interpolate(
        Eigen::Vector2d(opt->cmds[target_idx].velocity.x, opt->cmds[target_idx].velocity.y),
        Eigen::Vector2d(opt->cmds[next_idx].velocity.x, opt->cmds[next_idx].velocity.y),
        alpha);
      rp.yaw = interpolateYaw(opt->cmds[target_idx].yaw, opt->cmds[next_idx].yaw, alpha);
      rp.yaw_rate = interpolate(
        opt->cmds[target_idx].yaw_dot,
        opt->cmds[next_idx].yaw_dot,
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

  return true;
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();

  // 1) 延迟补偿：根据里程计时间戳外推当前位姿
  nav_msgs::msg::Odometry::SharedPtr odom;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    odom = latest_odom_;
  }

  const rclcpp::Time now = node->now();
  double dt_delay = 0.0;
  Eigen::Vector2d v_base(0.0, 0.0);
  double w_base = 0.0;

  if (odom) {
    dt_delay = (now - odom->header.stamp).seconds();
    dt_delay = std::clamp(dt_delay, 0.0, 0.2);

    v_base.x() = odom->twist.twist.linear.x;
    v_base.y() = odom->twist.twist.linear.y;
    w_base = odom->twist.twist.angular.z;
  }

  const double yaw_odom = tf2::getYaw(pose.pose.orientation);
  const Eigen::Vector2d v_map = rotate2d(v_base, yaw_odom);

  State curr;
  curr.x = pose.pose.position.x + v_map.x() * dt_delay;
  curr.y = pose.pose.position.y + v_map.y() * dt_delay;
  curr.yaw = normalizeYaw(yaw_odom + w_base * dt_delay);

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

  // 3) 调用 MPC 求解（输出 map 系速度）
  Control u_map;
#ifdef MINCO_DEBUG
  auto t_start = std::chrono::high_resolution_clock::now();
#endif
  bool success = solver_->solve(curr, ref, u_map);
#ifdef MINCO_DEBUG
  auto t_end = std::chrono::high_resolution_clock::now();
  if (success && !ref.empty()) {
    double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    double v_err = std::hypot(u_map.vx - ref[0].vel.x(), u_map.vy - ref[0].vel.y());
    double p_err = std::hypot(curr.x - ref[0].pos.x(), curr.y - ref[0].pos.y());
    
    double plan_vx = u_map.vx;
    double plan_vy = u_map.vy;
    double ref_vx = ref[0].vel.x();
    double ref_vy = ref[0].vel.y();

    custom_log::log_block(std::string("\033[34m[MincoMpc] "),
      NV(plan_vx), NV(plan_vy),
      NV(ref_vx), NV(ref_vy),
      NV(v_err), NV(p_err), NV(dt_ms));
  }
#endif

  if (!success) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // 4) 将全局控制律 [vx, vy, omega] 转换为机器人坐标系 (base)
  const Eigen::Vector2d u_map_v(u_map.vx, u_map.vy);
  const Eigen::Vector2d u_base_v = rotate2d(u_map_v, -curr.yaw);

  double vx = u_base_v.x();
  double vy = u_base_v.y();
  double wz = u_map.omega;

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

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::MincoMpcController, nav2_core::Controller)
