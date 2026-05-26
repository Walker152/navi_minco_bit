#ifndef MINCO_PLANNER__MINCO_UTILS_HPP_
#define MINCO_PLANNER__MINCO_UTILS_HPP_

#include "minco_core/header.hpp"

namespace geometry_utils {
class Trajectory;
}  // namespace geometry_utils

namespace traj_opt {
using Trajectory = geometry_utils::Trajectory;
}  // namespace traj_opt

namespace minco_planner::utils {

// === Generic Utilities ===
template <typename T> inline T clampValue(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
double calCurvatureDecay(
  double angle, double global_vmax, double deadzone, double saturation, double min_vel, double decay_power);

// === Path Geometry Utilities ===
// --- Velocity Profile Mapping ---
double getDistFromTrapezoid(
  double t, double total_length, double a_ref, double v_peak, double t_acc, double t_flat, double t_dec);

// --- Arc-Length Interpolation ---
Eigen::Vector3d interpolateByArcLength(
  const std::vector<Eigen::Vector3d> & path, const std::vector<double> & accumulated_dist, double s);

std::vector<Eigen::Vector3d> getSparseWaypoints(const std::vector<Eigen::Vector3d> & path,
  double max_vel,
  double max_acc,
  bool goal_reached,
  const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> & is_line_free);

double LimitLocalVel(const std::vector<Eigen::Vector3d> & sparse_path,
  int seg_idx,
  double global_vmax,
  double deadzone,
  double saturation,
  double min_turn_vel,
  double decay_power);

double ComputeNextSpeed(double v_curr, double seg_len, double remain_after, double amax, double local_vmax);

double ComputeSegmentTime(
  double seg_len, double v_curr, double v_next, double local_vmax, double amax, double min_seg_time);

void VelPropogation(const std::vector<double> & seg_len, double amax, std::vector<double> & local_vmaxs);

// === Trajectory Command Publishing ===
// --- Optimized Trajectory Publishing ---

void publishOptimizedTrajectory(const traj_opt::Trajectory & opt_traj,
  const traj_opt::Trajectory & yaw_traj,
  const rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr & pub,
  uint32_t & trajectory_id_counter,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step);

// --- Backup Trajectory Publishing ---
void publishBackupTrajectory(const traj_opt::Trajectory & backup_traj,
  const rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr & pub,
  uint32_t & trajectory_id_counter,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step,
  double fallback_yaw);

// --- Escape Command Publishing ---
void publishEscapeCommand(const geometry_msgs::msg::PoseStamped & current_pose,
  const Eigen::Vector2d & escape_vel,
  double current_yaw,
  const rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr & pub,
  uint32_t & trajectory_id_counter,
  const std_msgs::msg::Header & header);

// === Costmap Utilities ===
// --- Collision / Visibility Checks ---

bool isLineFree(
  nav2_costmap_2d::Costmap2D * costmap, const Eigen::Vector3d & p1, const Eigen::Vector3d & p2);

// --- Coordinate Conversion ---
bool worldToMap(nav2_costmap_2d::Costmap2D * costmap,
  const rclcpp::Logger & logger,
  double wx,
  double wy,
  unsigned int & mx,
  unsigned int & my);

void mapToWorld(nav2_costmap_2d::Costmap2D * costmap, double mx, double my, double & wx, double & wy);

// --- Costmap Cell Editing ---
void clearRobotCell(nav2_costmap_2d::Costmap2D * costmap, unsigned int mx, unsigned int my);

void compensateLeverArm(double v_lidar_x,
  double v_lidar_y,
  double omega_z,
  double yaw,
  double & vx_global,
  double & vy_global,
  double & omega_global);

}  // namespace minco_planner::utils
#endif  // MINCO_PLANNER__MINCO_UTILS_HPP_
