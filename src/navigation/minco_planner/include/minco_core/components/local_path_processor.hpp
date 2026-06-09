#ifndef MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
#define MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class PlannerModeContext;

struct LocalPathSeed
{
  bool valid{false};
  bool local_end_is_goal{false};
  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> sparse_waypoints;
  std::vector<double> local_magnitudes;
};

class LocalPathProcessor
{
public:
  void configure(double lookahead_dist,
    double max_vel,
    double max_acc,
    double traj_goal_tolerance,
    rclcpp::Logger logger);

  void updateLimits(double max_vel, double max_acc, double traj_goal_tolerance);

  LocalPathSeed buildSeed(const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context) const;

private:
  std::vector<Eigen::Vector3d> extractLocalPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const Eigen::Vector3d & cur_pos) const;

  bool clipLocalPathByRogBoundary(
    std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const;

  double lookahead_dist_{5.0};
  double max_vel_{2.0};
  double max_acc_{4.0};
  double traj_goal_tolerance_{0.5};
  rclcpp::Logger logger_{rclcpp::get_logger("LocalPathProcessor")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
