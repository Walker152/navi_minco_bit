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
#include <iostream>
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

double MincoMpcController::normalizeYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

void MincoMpcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".Q", rclcpp::ParameterValue(std::vector<double>{5.0, 5.0, 2.0}));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".R", rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));

  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".fixed_wz", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".deadzone_speed_threshold", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".control_delay_compensation", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_small_gyro_mode", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_acc_constraints", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_min", rclcpp::ParameterValue(-4.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_max", rclcpp::ParameterValue(4.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_frame", rclcpp::ParameterValue("camera_init"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".map_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lidar_roll_offset", rclcpp::ParameterValue(0.0));

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
  node->get_parameter(name + ".fixed_wz", fixed_wz_);
  node->get_parameter(name + ".deadzone_speed_threshold", deadzone_speed_threshold_);
  node->get_parameter(name + ".control_delay_compensation", control_delay_compensation_);
  node->get_parameter(name + ".use_small_gyro_mode", use_small_gyro_mode_);

  node->get_parameter(name + ".use_acc_constraints", mpc_config_.use_acc_constraints);
  node->get_parameter(name + ".ax_min", mpc_config_.ax_min);
  node->get_parameter(name + ".ax_max", mpc_config_.ax_max);
  node->get_parameter(name + ".ay_min", mpc_config_.ay_min);
  node->get_parameter(name + ".ay_max", mpc_config_.ay_max);
  node->get_parameter(name + ".alpha_min", mpc_config_.alpha_min);
  node->get_parameter(name + ".alpha_max", mpc_config_.alpha_max);

  node->get_parameter(name + ".odom_frame", odom_frame_);
  node->get_parameter(name + ".map_frame", map_frame_);
  node->get_parameter(name + ".lidar_offset_x", lidar_offset_x_);
  node->get_parameter(name + ".lidar_offset_y", lidar_offset_y_);
  node->get_parameter(name + ".lidar_roll_offset", lidar_roll_offset_);

  solver_ = std::make_unique<MpcSolver>(mpc_config_);

  // 订阅优化轨迹：/opt_path
  opt_path_sub_ = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>("/opt_path",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOptPath, this, std::placeholders::_1));

  // 订阅里程计：用于延迟补偿和mpc输入状态
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  mpc_predict_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_predict_path", 1);
  mpc_real_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_real_path", 1);
  cmd_vel_mpc_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_mpc", 1);
  real_path_history_.clear();
  last_real_path_pub_time_ = node->now();

  RCLCPP_INFO(logger_,
    "%s: MincoMpcController configured (dt=%.3f, lookahead_time=%.3f, deadzone=%.3f, delay_comp=%.3f, "
    "small_gyro=%s, fixed_wz=%.3f, lidar_offset_x=%.3f, lidar_offset_y=%.3f, lidar_roll_offset=%.3f)",
    name_.c_str(),
    dt,
    lookahead_time,
    deadzone_speed_threshold_,
    control_delay_compensation_,
    use_small_gyro_mode_ ? "true" : "false",
    fixed_wz_,
    lidar_offset_x_,
    lidar_offset_y_,
    lidar_roll_offset_);
}

void MincoMpcController::compensateLeverArm(double v_lidar_x,
  double v_lidar_y,
  double omega_z,
  double yaw,
  double & vx_global,
  double & vy_global,
  double & omega_global) const
{
  const double v_body_x = v_lidar_x + omega_z * lidar_offset_y_;
  const double v_body_y = v_lidar_y - omega_z * lidar_offset_x_;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  vx_global = v_body_x * cos_yaw - v_body_y * sin_yaw;
  vy_global = v_body_x * sin_yaw + v_body_y * cos_yaw;
  omega_global = omega_z;
  // std::cout << "Raw velocities: vx_lidar=" << v_lidar_x << ", vy_lidar=" << v_lidar_y
  //           << ", omega_z=" << omega_z << ", yaw=" << yaw << std::endl;
  // std::cout << "Compensated velocities: vx_global=" << vx_global << ", vy_global=" << vy_global
  //           << ", omega_global=" << omega_global << std::endl;
}

void MincoMpcController::extractGlobalVelocityAndYaw(const nav_msgs::msg::Odometry::SharedPtr & odom,
  double & vx_global,
  double & vy_global,
  double & omega_global,
  double & yaw_global) const
{
  // 1. 提取 3D 体轴速度 (IMU 物理中心的局部速度)
  Eigen::Vector3d v_body_imu(
    odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

  // 2. 提取全量 3D 姿态四元数
  Eigen::Quaterniond q(odom->pose.pose.orientation.w,
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z);

  // 3. 3D 旋转：将倾斜机体系下的速度无损映射回与地面平行的全局系
  Eigen::Vector3d v_imu_global = q * v_body_imu;

  // 4. 提取角速度与 Yaw 角
  omega_global = odom->twist.twist.angular.z;
  yaw_global = tf2::getYaw(odom->pose.pose.orientation);

  // 5. 计算全局杆臂补偿 (将底盘水平安装偏置旋转到全局系)
  double offset_global_x = std::cos(yaw_global) * lidar_offset_x_ - std::sin(yaw_global) * lidar_offset_y_;
  double offset_global_y = std::sin(yaw_global) * lidar_offset_x_ + std::cos(yaw_global) * lidar_offset_y_;
  // 6. 计算底盘旋转中心的全局速度
  vx_global = v_imu_global.x() + omega_global * offset_global_y;
  vy_global = v_imu_global.y() - omega_global * offset_global_x;
}

void MincoMpcController::cleanup()
{
  opt_path_sub_.reset();
  odom_sub_.reset();
  mpc_predict_path_pub_.reset();
  mpc_real_path_pub_.reset();
  cmd_vel_mpc_pub_.reset();
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

  const uint32_t new_traj_id = (msg && !msg->cmds.empty()) ? msg->cmds.front().trajectory_id : 0u;
  const uint32_t old_traj_id = (latest_opt_path_ && !latest_opt_path_->cmds.empty())
                                 ? latest_opt_path_->cmds.front().trajectory_id
                                 : 0u;

  // 仅在轨迹 ID 切换时重置跟踪状态；同一轨迹重复发布（更新时间戳）不重置
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

bool MincoMpcController::transformPathToOdom(const ros_interfaces::msg::MpcPositionCommand::SharedPtr & opt,
  std::vector<ros_interfaces::msg::PositionCommand> & out_cmds) const
{
  std::string source_frame = opt->header.frame_id;
  if (source_frame.empty()) {
    source_frame = map_frame_;
    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_WARN_THROTTLE(logger_,
        *node_ptr->get_clock(),
        10000,
        "Received opt_path with empty frame_id, defaulting to '%s'",
        source_frame.c_str());
    }
  }

  // 从costmap获取全局坐标系，优先使用global_frame_，如果未设置则退回到odom_frame_
  std::string target_frame = global_frame_;
  if (target_frame.empty()) {
    target_frame = odom_frame_;
  }

  if (source_frame == target_frame) {
    out_cmds = opt->cmds;
    return true;
  }

  // 如果坐标系不同，尝试进行转换
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    out_cmds = opt->cmds;
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

  } catch (tf2::TransformException & ex) {
    return false;
  }
  return true;
}

bool MincoMpcController::buildReferenceFromOptPath(
  const State & curr, std::vector<ReferencePoint> & out_ref) const
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  const rclcpp::Time now = node->now();

  // 获取最新的优化轨迹和里程计数据，以及当前跟踪状态（索引、时间戳、轨迹ID）
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

  // 最近点搜索（基于最小欧式距离），并计算更精确的最近点投影索引
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

  double nearest_idx_float = static_cast<double>(best_idx);

  if (best_idx < n_cmds - 1) {
    const auto & p_curr = cmds[best_idx];
    const auto & p_next = cmds[best_idx + 1];

    Eigen::Vector2d a_vec(p_next.position.x - p_curr.position.x, p_next.position.y - p_curr.position.y);
    Eigen::Vector2d b_vec(curr.x - p_curr.position.x, curr.y - p_curr.position.y);

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

  // 轨迹未更新时，按时间持续向前推进参考索引，且不允许回退
  // const bool same_opt_traj = has_tracked_ref && tracked_opt_traj_id == current_traj_id;

  // double progress_idx_float = nearest_idx_float;
  // if (same_opt_traj) {
  //   double dt_pass = (now - tracked_ref_time).seconds();
  //   dt_pass = std::max(0.0, dt_pass);
  //   progress_idx_float = tracked_ref_idx + dt_pass / planner_dt;
  // }
  // double current_idx_float = std::max(nearest_idx_float, progress_idx_float);

  // // 同一条轨迹若超过阈值仍未更新，判定规划器卡死
  // const double traj_stamp_sec =
  //   static_cast<double>(opt->header.stamp.sec) + static_cast<double>(opt->header.stamp.nanosec) * 1.0e-9;
  // const double absolute_age = now.seconds() - traj_stamp_sec;
  // if (same_opt_traj && absolute_age > 0.5) {
  //   return false;
  // }

  double current_idx_float = nearest_idx_float;
  current_idx_float = std::min(current_idx_float, static_cast<double>(n_cmds - 1));

  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    tracked_ref_idx_ = current_idx_float;
    tracked_ref_time_ = now;
    tracked_opt_traj_id_ = current_traj_id;
    has_tracked_ref_ = true;
  }

  double current_traj_time = current_idx_float * planner_dt;

  // 让 MPC 始终去追踪未来一段时间的参考点，用于控制延迟补偿
  double control_delay = control_delay_compensation_;

  current_traj_time += control_delay;

  const int N = mpc_config_.horizon;
  const double mpc_dt = mpc_config_.dt;
  out_ref.clear();
  out_ref.reserve(static_cast<size_t>(N));

  for (int k = 0; k < N; ++k) {
    double target_time = current_traj_time + k * mpc_dt;
    double target_idx_float = target_time / planner_dt;

    if (target_idx_float < 0.0)
      target_idx_float = 0.0;
    if (target_idx_float > static_cast<double>(n_cmds - 1)) {
      target_idx_float = static_cast<double>(n_cmds - 1);
    }

    size_t target_idx = static_cast<size_t>(std::floor(target_idx_float));
    size_t next_idx = target_idx + 1;
    double alpha = target_idx_float - static_cast<double>(target_idx);

    ReferencePoint rp;
    if (next_idx < n_cmds) {
      // 二次前馈插值（泰勒展开），使用 P/V/A/J 进行插值，补偿控制延迟带来的误差。
      const double dt = std::max(0.0, std::min(planner_dt, alpha * planner_dt));
      const double dt2 = dt * dt;
      const double dt3 = dt2 * dt;

      const Eigen::Vector2d p_i(cmds[target_idx].position.x, cmds[target_idx].position.y);
      const Eigen::Vector2d v_i(cmds[target_idx].velocity.x, cmds[target_idx].velocity.y);
      const Eigen::Vector2d a_i(cmds[target_idx].acceleration.x, cmds[target_idx].acceleration.y);
      const Eigen::Vector2d j_i(cmds[target_idx].jerk.x, cmds[target_idx].jerk.y);

      rp.pos = p_i + v_i * dt + 0.5 * a_i * dt2 + (1.0 / 6.0) * j_i * dt3;
      rp.vel = v_i + a_i * dt + 0.5 * j_i * dt2;

      // 线性角度插值
      rp.yaw = interpolateYaw(cmds[target_idx].yaw, cmds[next_idx].yaw, alpha);
      rp.yaw_rate = interpolate(cmds[target_idx].yaw_dot, cmds[next_idx].yaw_dot, alpha);
    } else {
      const auto & p_end = cmds.back();
      rp.pos = Eigen::Vector2d(p_end.position.x, p_end.position.y);
      rp.vel = Eigen::Vector2d(0.0, 0.0);
      rp.yaw = p_end.yaw;
      rp.yaw_rate = 0.0;
    }
    out_ref.push_back(rp);
  }

  // 对参考序列的 yaw 进行连续性处理
  if (!out_ref.empty()) {
    double diff = out_ref[0].yaw - curr.yaw;
    // 将 diff 限制在 [-PI, PI]
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

void MincoMpcController::applyGravityCompensation(
  const nav_msgs::msg::Odometry::SharedPtr & odom, double & vx, double & vy)
{
  if (!odom) {
    return;
  }

  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // std::cout << "Original roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
  const double true_roll = roll - lidar_roll_offset_;
  constexpr double angle_threshold = 0.05;
  constexpr double k_gravity_x = 1.0;
  constexpr double k_gravity_y = 20.0;


  double body_comp_x = 0.0;
  double body_comp_y = 0.0;
  if (std::abs(pitch) > angle_threshold) {
    body_comp_x = k_gravity_x * std::sin(-pitch);
  }
  if (std::abs(true_roll) > angle_threshold) {
    body_comp_y = k_gravity_y * std::sin(std::abs(true_roll));
  }

  if (body_comp_x == 0.0 && body_comp_y == 0.0) {
    return;
  }

  const double global_comp_x = body_comp_x * std::cos(yaw) - body_comp_y * std::sin(yaw);
  const double global_comp_y = body_comp_x * std::sin(yaw) + body_comp_y * std::cos(yaw);

  vx += global_comp_x;
  vy += global_comp_y;

  vx = std::clamp(vx, mpc_config_.vx_min, mpc_config_.vx_max);
  vy = std::clamp(vy, mpc_config_.vy_min, mpc_config_.vy_max);
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = node_.lock();

  const rclcpp::Time now = node->now();
  nav_msgs::msg::Odometry::SharedPtr latest_odom;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    latest_odom = latest_odom_;
  }

  // 1) 构造当前状态（基于 pose + 最新里程计速度，用于延迟补偿）
  State curr;
  curr.x = pose.pose.position.x;
  curr.y = pose.pose.position.y;
  curr.yaw = normalizeYaw(tf2::getYaw(pose.pose.orientation));
  if (latest_odom) {
    double vx = latest_odom->twist.twist.linear.x;
    double vy = latest_odom->twist.twist.linear.y;
    double omega = latest_odom->twist.twist.angular.z;
    double yaw = normalizeYaw(tf2::getYaw(latest_odom->pose.pose.orientation));
    // extractGlobalVelocityAndYaw(latest_odom, vx, vy, omega, yaw);
    compensateLeverArm(vx, vy, omega, yaw, curr.vx, curr.vy, curr.omega);

    // curr.yaw = normalizeYaw(yaw);
    // curr.vx = vx;
    // curr.vy = vy;
    // curr.omega = omega;

    const double noise_threshold = 0.03;
    if (std::abs(curr.vx) < noise_threshold) {
      curr.vx = 0.0;
    }
    if (std::abs(curr.vy) < noise_threshold) {
      curr.vy = 0.0;
    }
    if (std::abs(curr.omega) < noise_threshold) {
      curr.omega = 0.0;
    }
  } else {
    curr.vx = 0.0;
    curr.vy = 0.0;
    curr.omega = 0.0;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = global_frame_;

  // Sync Nav2 success semantics: if GoalChecker says reached, output zero command immediately.
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

  if (control_delay_compensation_ > 1e-3) {
    curr.x += curr.vx * control_delay_compensation_;
    curr.y += curr.vy * control_delay_compensation_;
    curr.yaw += curr.omega * control_delay_compensation_;
    curr.yaw = normalizeYaw(curr.yaw);
  }

  // 2) 构造参考序列：优先 /opt_path
  std::vector<ReferencePoint> ref;
  bool ok_ref = buildReferenceFromOptPath(curr, ref);

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
  bool success = solver_->solve(curr, ref, u_global, &pred_states);

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
      NV(plan_vx),
      NV(plan_vy),
      NV(ref_vx),
      NV(ref_vy),
      NV(v_err_x),
      NV(v_err_y),
      NV(curr_x),
      NV(curr_y),
      NV(ref_x),
      NV(ref_y),
      NV(p_err_x),
      NV(p_err_y),
      NV(dt_ms));
  }
#endif

  if (!success) {
    std::cout << color_text::RED << "[MincoMpc] Solver Failed!" << color_text::RESET << std::endl;
    // Debug info for failure
    if (!ref.empty()) {
      std::cout << "[MincoMpc] Ref Size: " << ref.size() << std::endl;
      std::cout << "[MincoMpc] Curr: " << curr.x << ", " << curr.y << ", " << curr.yaw << std::endl;
      std::cout << "[MincoMpc] Ref[0]: " << ref[0].pos.x() << ", " << ref[0].pos.y() << ", " << ref[0].yaw
                << std::endl;
    }

    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

  // 5) 直接下发全局坐标系速度
  double vx_mpc = u_global.vx;
  double vy_mpc = u_global.vy;
  double wz = fixed_wz_;

  if (cmd_vel_mpc_pub_) {
    geometry_msgs::msg::Twist raw_cmd;
    raw_cmd.linear.x = vx_mpc;
    raw_cmd.linear.y = vy_mpc;
    raw_cmd.angular.z = u_global.omega;
    cmd_vel_mpc_pub_->publish(raw_cmd);
  }

  // 小陀螺模式：当启用时，始终输出固定的旋转速度，而不使用 MPC 输出的角速度。
  if (!use_small_gyro_mode_) {
    wz = std::min(mpc_config_.omega_max, std::max(mpc_config_.omega_min, u_global.omega));
  }

  double output_delay = 0.025;
  double phase_delay = curr.omega * output_delay;
  double cos_phase = std::cos(phase_delay);
  double sin_phase = std::sin(phase_delay);
  double vx = cos_phase * vx_mpc + sin_phase * vy_mpc;
  double vy = -sin_phase * vx_mpc + cos_phase * vy_mpc;
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

  // 6) 死区截断
  const double deadzone = deadzone_speed_threshold_;
  if (std::hypot(vx, vy) < deadzone) {
    vx = 0.0;
    vy = 0.0;
  }
  // applyGravityCompensation(latest_odom, vx, vy);
  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.angular.z = wz;
  return cmd;
}

void MincoMpcController::publishVisualization(
  const std::vector<State> & pred_path, const State & curr_state)
{
  auto node = node_.lock();
  if (!node)
    return;

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
    double dist =
      std::hypot(last.pose.position.x - ps.pose.position.x, last.pose.position.y - ps.pose.position.y);
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
