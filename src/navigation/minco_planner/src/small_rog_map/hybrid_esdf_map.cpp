#include "small_rog_map/hybrid_esdf_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace small_rog_map {

HybridESDFMap::HybridESDFMap()
: static_layer_(std::make_shared<StaticLayer>()), dynamic_layer_(std::make_shared<DynamicLayer>())
{
}

void HybridESDFMap::initRos(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::string & topic,
  double resolution,
  double dynamic_size_m,
  double dynamic_dilation_radius_m)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }
  dynamic_layer_->configure(node, topic, resolution, dynamic_size_m, dynamic_dilation_radius_m);
}

void HybridESDFMap::setRobotPosition(double x, double y)
{
  dynamic_layer_->setRobotPosition(x, y);
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

  return true;
}

void HybridESDFMap::updateDynamicMapFromPointCloud(const sensor_msgs::msg::PointCloud2 & cloud,
  const StaticLayer & reference_layer,
  double dilation_radius_m)
{
  if (!dynamic_layer_) {
    dynamic_layer_ = std::make_shared<DynamicLayer>();
  }

  // 1. Update the dynamic ESDF using the static layer as geometry reference
  dynamic_layer_->updateFromPointCloud(cloud,
    reference_layer.width(),
    reference_layer.height(),
    reference_layer.resolution(),
    reference_layer.origin(),
    dilation_radius_m);
}

void HybridESDFMap::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  // Clamping far distances keeps the cost and gradients stable.
  // 1. Clamp the max and min distance used by the optimizer
  constexpr double kMaxDist = 3.0;
  // 定义最大穿透深度 (负值)，这取决于你的机器人半径或狭窄通道的容忍度
  // 比如设置为 -1.0 表示深入障碍物1米内仍有梯度指引出来
  constexpr double kMinDist = -1.5;

  auto clamp = [](double & d, Eigen::Vector3d & g) {
    if (!std::isfinite(d)) {
      d = (d < 0.0) ? kMinDist : kMaxDist;
      g.setZero();
      return;
    }
    if (d > kMaxDist) {
      d = kMaxDist;
      g.setZero();
      return;
    }
    if (d < kMinDist) {
      d = kMinDist;
      g.setZero();  // 穿透太深超过阈值，不再提供梯度(避免数值发散)
      return;
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
      auto time_start = std::chrono::steady_clock::now();
      dynamic_layer_->evaluate(pos, d_dynamic, g_dynamic);
      auto time_end = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = time_end - time_start;
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

const std::shared_ptr<StaticLayer> & HybridESDFMap::staticLayer() const
{
  return static_layer_;
}
const std::shared_ptr<DynamicLayer> & HybridESDFMap::dynamicLayer() const
{
  return dynamic_layer_;
}

}  // namespace small_rog_map
