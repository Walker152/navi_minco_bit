#ifndef MINCO_PLANNER__MINCO_UTILS_HPP_
#define MINCO_PLANNER__MINCO_UTILS_HPP_

// Standard library
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

// Third party
#include <Eigen/Core>

// ROS2
#include "rclcpp/rclcpp.hpp"

// Project
#include "ros_interfaces/msg/mpc_position_command.hpp"
#include "std_msgs/msg/header.hpp"

namespace geometry_utils {
class Trajectory;
}  // namespace geometry_utils

namespace traj_opt {
using Trajectory = geometry_utils::Trajectory;
}  // namespace traj_opt

namespace minco_planner::utils {

template <typename T>
inline T clampValue(T v, T lo, T hi)
{
  return std::min(std::max(v, lo), hi);
}

// Convert time along a (symmetric) trapezoid/triangle velocity profile to traveled distance.
// Returns s in [0, total_length].
double getDistFromTrapezoid(
  double t,
  double total_length,
  double a_ref,
  double v_peak,
  double t_acc,
  double t_flat);

// Interpolate a path by arc-length mapping.
Eigen::Vector3d interpolateByArcLength(
  const std::vector<Eigen::Vector3d> & path,
  const std::vector<double> & accumulated_dist,
  double s);

void publishOptimizedTrajectory(
  const traj_opt::Trajectory & opt_traj,
  const rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr & pub,
  uint32_t & trajectory_id_counter,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step);

void publishBackupTrajectory(
  const traj_opt::Trajectory & backup_traj,
  const rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr & pub,
  uint32_t & trajectory_id_counter,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step);

std::vector<Eigen::Vector3d> getSparseWaypoints(
  const std::vector<Eigen::Vector3d> & path,
  double max_vel,
  double max_acc,
  const std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> & is_line_free);

}  // namespace minco_planner::utils

#endif  // MINCO_PLANNER__MINCO_UTILS_HPP_
