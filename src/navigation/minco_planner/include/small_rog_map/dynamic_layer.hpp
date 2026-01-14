#pragma once

#include <Eigen/Core>

#include <mutex>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace small_rog_map
{

class DynamicLayer
{
public:
  DynamicLayer();

  // Update from nav_msgs::msg::OccupancyGrid.
  // Obstacles: cells with value >= 99.
  // Dilation: expand obstacles by dilation_radius_m before EDT.
  void updateFromOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double dilation_radius_m,
    bool treat_unknown_as_obstacle = false);

  // Update directly from Nav2 Costmap2D.
  // Obstacles: LETHAL_OBSTACLE (254) and INSCRIBED_INFLATED_OBSTACLE (253).
  // Unknown: NO_INFORMATION (255) handled by treat_unknown_as_obstacle.
  // Dilation: expand obstacles by dilation_radius_m before EDT.
  void updateFromCostmap2D(
    nav2_costmap_2d::Costmap2D * costmap,
    double dilation_radius_m,
    bool treat_unknown_as_obstacle);

  // Query distance (meters) and gradient. If pos is outside the grid, returns far distance.
  void evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const;

  bool isValid() const;
  bool isInside(const Eigen::Vector2d & pos_xy) const;

  int width() const;
  int height() const;
  double resolution() const;
  Eigen::Vector2d origin() const;

private:
  static constexpr double kFarDistance = 10.0;

  void buildDilationOffsets(int radius_cells, std::vector<Eigen::Vector2i> & offsets) const;

  mutable std::mutex mutex_;

  std::vector<double> dist_m_;
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};
};

}  // namespace small_rog_map
