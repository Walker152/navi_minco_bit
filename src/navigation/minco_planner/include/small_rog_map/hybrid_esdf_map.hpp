#pragma once

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "small_rog_map/dynamic_layer.hpp"
#include "small_rog_map/static_layer.hpp"

namespace small_rog_map
{

class HybridESDFMap
{
public:
  using Ptr = std::shared_ptr<HybridESDFMap>;

  HybridESDFMap();

  void initRos(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & topic);

  bool loadStaticMap(const std::string & pcd_path, double resolution);

  void updateDynamicMapFromPointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const StaticLayer & reference_layer,
    double dilation_radius_m);

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
