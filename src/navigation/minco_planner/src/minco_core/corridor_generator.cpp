#include "minco_core/corridor_generator.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner {

void SimpleCorridorGenerator::setSafetyMargins(double robot_radius, double extra_margin)
{
  if (std::isfinite(robot_radius) && robot_radius > 0.0) {
    robot_radius_ = robot_radius;
  }
  if (std::isfinite(extra_margin) && extra_margin >= 0.0) {
    extra_margin_ = extra_margin;
  }
}

void SimpleCorridorGenerator::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  map_ = map;
}

PolyhedronH SimpleCorridorGenerator::generateSafeBox(
  const Eigen::Vector3d & center, double max_radius) const
{
  // 1. Query ROGMap distance field at the box center
  constexpr double kMinHalfSize = 0.05;  // meters

  double dist = 0.0;
  if (map_) {
    Eigen::Vector3d grad;
    map_->evaluate(center, dist, grad);
  }

  if (!std::isfinite(dist) || dist < 0.0) {
    dist = 0.0;
  }

  // 2. Convert distance field value to a conservative box half-size
  // Robot radius + extra clearance margin
  const double safe_dist = std::max(dist - robot_radius_ - extra_margin_, 0.0);
  double box_half_size = (safe_dist / 1.414) * 0.9;

  if (!std::isfinite(box_half_size) || box_half_size < kMinHalfSize) {
    box_half_size = kMinHalfSize;
  }

  // Limit by caller-provided radius
  if (max_radius > 0.0 && std::isfinite(max_radius)) {
    box_half_size = std::min(box_half_size, max_radius);
  }

  const double x_min = center.x() - box_half_size;
  const double x_max = center.x() + box_half_size;
  const double y_min = center.y() - box_half_size;
  const double y_max = center.y() + box_half_size;
  const double z_min = center.z() - box_half_size;
  const double z_max = center.z() + box_half_size;

  // 3. Build an axis-aligned box in half-space form
  PolyhedronH h(6, 4);
  // x >= x_min  -> -x < -x_min
  h.row(0) << -1.0, 0.0, 0.0, -x_min;
  // x <= x_max  ->  x <  x_max
  h.row(1) << 1.0, 0.0, 0.0, x_max;
  // y >= y_min  -> -y < -y_min
  h.row(2) << 0.0, -1.0, 0.0, -y_min;
  // y <= y_max  ->  y <  y_max
  h.row(3) << 0.0, 1.0, 0.0, y_max;
  // z >= z_min  -> -z < -z_min
  h.row(4) << 0.0, 0.0, -1.0, -z_min;
  // z <= z_max  ->  z <  z_max
  h.row(5) << 0.0, 0.0, 1.0, z_max;

  return h;
}

}  // namespace minco_planner
