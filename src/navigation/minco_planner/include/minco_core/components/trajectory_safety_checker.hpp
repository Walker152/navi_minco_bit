#ifndef MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class TrajectorySafetyChecker
{
public:
  void configure(double safe_dist, double sample_dt, rclcpp::Logger logger);
  void setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query);

  bool checkPoint(const Eigen::Vector3d & pos) const;
  bool checkTrajectory(const traj_opt::Trajectory & traj) const;
  double getDistance(const Eigen::Vector3d & pos) const;
  bool projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const;

private:
  bool ensureQueryAvailable() const;

  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  double safe_dist_{0.0};
  double sample_dt_{0.05};
  rclcpp::Logger logger_{rclcpp::get_logger("TrajectorySafetyChecker")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
