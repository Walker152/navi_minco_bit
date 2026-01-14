#include "small_rog_map/hybrid_esdf_map.hpp"

#include <algorithm>
#include <limits>

namespace small_rog_map
{

HybridESDFMap::HybridESDFMap()
: static_layer_(std::make_shared<StaticLayer>()),
  dynamic_layer_(std::make_shared<DynamicLayer>())
{
}

bool HybridESDFMap::loadStaticMap(const std::string & pcd_path, double resolution)
{
  if (!static_layer_) {
    static_layer_ = std::make_shared<StaticLayer>();
  }
  return static_layer_->loadFromPCD(pcd_path, resolution);
}

void HybridESDFMap::updateDynamicMap(
  const nav_msgs::msg::OccupancyGrid & grid,
  double dilation_radius_m,
  bool treat_unknown_as_obstacle)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }
  dynamic_layer_->updateFromOccupancyGrid(grid, dilation_radius_m, treat_unknown_as_obstacle);
}

void HybridESDFMap::updateDynamicMap(
  nav2_costmap_2d::Costmap2D * costmap,
  double dilation_radius_m,
  bool treat_unknown_as_obstacle)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }
  dynamic_layer_->updateFromCostmap2D(costmap, dilation_radius_m, treat_unknown_as_obstacle);
}

void HybridESDFMap::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  double d_static = kFarDistance;
  Eigen::Vector3d g_static(0.0, 0.0, 0.0);

  double d_dynamic = kFarDistance;
  Eigen::Vector3d g_dynamic(0.0, 0.0, 0.0);

  if (static_layer_ && static_layer_->isValid()) {
    static_layer_->evaluate(pos, d_static, g_static);
  }

  bool dynamic_valid = false;
  if (dynamic_layer_ && dynamic_layer_->isValid()) {
    dynamic_valid = dynamic_layer_->isInside(Eigen::Vector2d(pos.x(), pos.y()));
    if (dynamic_valid) {
      dynamic_layer_->evaluate(pos, d_dynamic, g_dynamic);
    }
  }

  if (dynamic_valid && d_dynamic <= d_static) {
    dist = d_dynamic;
    grad = g_dynamic;
  } else {
    dist = d_static;
    grad = g_static;
  }
}

const std::shared_ptr<StaticLayer> & HybridESDFMap::staticLayer() const { return static_layer_; }
const std::shared_ptr<DynamicLayer> & HybridESDFMap::dynamicLayer() const { return dynamic_layer_; }

}  // namespace small_rog_map
