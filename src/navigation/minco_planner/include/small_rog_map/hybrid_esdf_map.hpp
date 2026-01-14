#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "small_rog_map/dynamic_layer.hpp"
#include "small_rog_map/static_layer.hpp"

namespace small_rog_map
{

class HybridESDFMap
{
public:
  using Ptr = std::shared_ptr<HybridESDFMap>;

  HybridESDFMap();

  bool loadStaticMap(const std::string & pcd_path, double resolution);

  void updateDynamicMap(
    const nav_msgs::msg::OccupancyGrid & grid,
    double dilation_radius_m,
    bool treat_unknown_as_obstacle = false);

  void updateDynamicMap(
    nav2_costmap_2d::Costmap2D * costmap,
    double dilation_radius_m,
    bool treat_unknown_as_obstacle = false);

  // Fused query: dist = min(d_static, d_dynamic).
  void evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const;

  const std::shared_ptr<StaticLayer> & staticLayer() const;
  const std::shared_ptr<DynamicLayer> & dynamicLayer() const;

private:
  static constexpr double kFarDistance = 10.0;

  std::shared_ptr<StaticLayer> static_layer_;
  std::shared_ptr<DynamicLayer> dynamic_layer_;
};

}  // namespace small_rog_map
