#include "small_rog_map/hybrid_esdf_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace small_rog_map
{

HybridESDFMap::HybridESDFMap()
: static_layer_(std::make_shared<StaticLayer>()),
  dynamic_layer_(std::make_shared<DynamicLayer>())
{
}

void HybridESDFMap::initRos(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & topic)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }
  dynamic_layer_->configure(node, topic);
}

bool HybridESDFMap::loadStaticMap(const std::string & pcd_path, double resolution)
{
  if (!static_layer_) {
    static_layer_ = std::make_shared<StaticLayer>();
  }

  const bool ok = static_layer_->loadFromPCD(pcd_path, resolution);
  if (!ok) {
    return false;
  }

  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }
  if (static_layer_->isValid()) {
    // 2. Align dynamic grid geometry with the static layer
    dynamic_layer_->setGeometry(
      static_layer_->width(),
      static_layer_->height(),
      static_layer_->resolution(),
      static_layer_->origin());
  }

  return true;
}

void HybridESDFMap::updateDynamicMapFromPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const StaticLayer & reference_layer,
  double dilation_radius_m)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }

  // 1. Update the dynamic ESDF using the static layer as geometry reference
  dynamic_layer_->updateFromPointCloud(
    cloud,
    reference_layer.width(),
    reference_layer.height(),
    reference_layer.resolution(),
    reference_layer.origin(),
    dilation_radius_m);
}

void HybridESDFMap::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  // 1. Clamp the max distance used by the optimizer
  // [Theory] For trajectory optimization, we mainly care about distances near obstacles.
  // Clamping far distances keeps the cost and gradients stable.
  constexpr double kMaxDist = 3.0;

  auto clamp = [](double & d, Eigen::Vector3d & g) {
    if (!std::isfinite(d) || d > kMaxDist) {
      d = kMaxDist;
      g.setZero();
      return;
    }
    if (d < 0.0) {
      d = 0.0;
      g.setZero();
    }
  };

  // 2. Query the static layer
  double d_static = kMaxDist;
  Eigen::Vector3d g_static(0.0, 0.0, 0.0);

  // 3. Query the dynamic layer (only if inside its grid)
  double d_dynamic = kMaxDist;
  Eigen::Vector3d g_dynamic(0.0, 0.0, 0.0);

  if (static_layer_ && static_layer_->isValid()) {
    static_layer_->evaluate(pos, d_static, g_static);
    clamp(d_static, g_static);
  }

  bool dynamic_valid = false;
  if (dynamic_layer_ && dynamic_layer_->isValid()) {
    dynamic_valid = dynamic_layer_->isInside(Eigen::Vector2d(pos.x(), pos.y()));
    if (dynamic_valid) {
      dynamic_layer_->evaluate(pos, d_dynamic, g_dynamic);
      clamp(d_dynamic, g_dynamic);
    }
  }

  // 4. Fuse distances by taking the minimum
  // [Theory] The distance-to-nearest-obstacle is the min over all obstacle sources.
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
