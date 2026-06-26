#ifndef MINCO_PLANNER__CORRIDOR_GENERATOR_HPP_
#define MINCO_PLANNER__CORRIDOR_GENERATOR_HPP_

#include <memory>

// Third party
#include <Eigen/Core>

// Project
#include "rog_map/map_query_interface.hpp"

namespace minco_planner {

// Represents half-space constraints in the form: n_x * x + n_y * y + n_z * z < d
// A box typically has 6 faces (x_min, x_max, y_min, y_max, z_min, z_max).
using PolyhedronH = Eigen::MatrixX4d;

class SimpleCorridorGenerator
{
public:
  using Ptr = std::shared_ptr<SimpleCorridorGenerator>;

  SimpleCorridorGenerator() = default;

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);

  void setSafetyMargins(double robot_radius, double extra_margin);

  PolyhedronH generateSafeBox(const Eigen::Vector3d & center, double max_radius = 2.0) const;

private:
  std::shared_ptr<rog_map::MapQueryInterface> map_;

  double robot_radius_{0.4};
  double extra_margin_{0.15};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__CORRIDOR_GENERATOR_HPP_
