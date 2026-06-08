#include "minco_core/components/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"

namespace minco_planner {

void TrajectorySafetyChecker::configure(double safe_dist, double sample_dt, rclcpp::Logger logger)
{
  safe_dist_ = safe_dist;
  sample_dt_ = sample_dt > 1e-6 ? sample_dt : 0.05;
  logger_ = logger;
}

void TrajectorySafetyChecker::setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query)
{
  dynamic_query_ = std::move(dynamic_query);
}

bool TrajectorySafetyChecker::ensureQueryAvailable() const
{
  if (dynamic_query_) {
    return true;
  }
  RCLCPP_ERROR_THROTTLE(
    logger_, *rclcpp::Clock::make_shared(), 1000,
    "[MincoPlanner] ROGMap dynamic query is unavailable for trajectory safety check.");
  return false;
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d & pos) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!dynamic_query_->worldToMap(pos.x(), pos.y(), mx, my)) {
    return false;
  }
  const unsigned char cost = dynamic_query_->value(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  dynamic_query_->evaluate(pos, esdf_dist, esdf_grad);
  return esdf_dist > safe_dist_;
}

bool TrajectorySafetyChecker::checkTrajectory(const traj_opt::Trajectory & traj) const
{
  if (!ensureQueryAvailable()) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  for (double t = 0.0; t <= dur; t += sample_dt_) {
    if (!checkPoint(traj.getPos(t))) {
      return false;
    }
  }
  return true;
}

double TrajectorySafetyChecker::getDistance(const Eigen::Vector3d & pos) const
{
  if (!dynamic_query_) {
    return 0.0;
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  dynamic_query_->evaluate(pos, dist, grad);
  return dist;
}

bool TrajectorySafetyChecker::projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const
{
  if (!dynamic_query_) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  dynamic_query_->evaluate(pos, esdf_dist, esdf_grad);
  if (esdf_dist < 0.0 && esdf_grad.norm() > 1e-6) {
    pos += (margin - esdf_dist) * esdf_grad.normalized();
    return true;
  }
  return false;
}

}  // namespace minco_planner
