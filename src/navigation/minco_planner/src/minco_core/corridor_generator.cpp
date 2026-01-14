#include "minco_core/corridor_generator.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

SimpleCorridorGenerator::SimpleCorridorGenerator(small_rog_map::HybridESDFMap::Ptr esdf_map)
: esdf_map_(esdf_map)
{
}

PolyhedronH SimpleCorridorGenerator::generateSafeBox(
  const Eigen::Vector3d & center,
  double max_radius) const
{
  // 1. Query ESDF distance at the box center
  constexpr double kMinHalfSize = 0.05;  // meters

  double dist = 0.0;
  if (esdf_map_) {
    Eigen::Vector3d grad;
    esdf_map_->evaluate(center, dist, grad);
  }

  if (!std::isfinite(dist) || dist < 0.0) {
    dist = 0.0;
  }

  // 2. Convert ESDF distance to a conservative box half-size
  // Robot radius margin
  double safe_dist = std::max(dist - 0.4, 0.0);
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

  // 4. Build an axis-aligned box in half-space form
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
