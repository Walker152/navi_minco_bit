#pragma once

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace small_rog_map
{

class DynamicLayer
{
public:
  DynamicLayer();

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & topic);
  void setGeometry(int w, int h, double res, const Eigen::Vector2d & origin);

  // Update from sparse dynamic obstacle point cloud (e.g. STVL voxel_grid).
  // The grid size/resolution/origin should be aligned with the static layer.
  void updateFromPointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    double dilation_radius_m);

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
  static constexpr double kESDFStrength = -std::numeric_limits<double>::infinity();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void buildDilationOffsets(int radius_cells, std::vector<Eigen::Vector2i> & offsets) const;

  mutable std::mutex mutex_;

  std::vector<double> dist_m_;
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
};

}  // namespace small_rog_map
