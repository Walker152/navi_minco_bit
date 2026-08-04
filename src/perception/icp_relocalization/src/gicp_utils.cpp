#include "gicp_utils.hpp"

#include "color_text.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace icp_relocalization::gicp_utils {

namespace {

PointCloud::Ptr removeNonFinitePoints(const PointCloud::Ptr & cloud)
{
  if (!cloud || cloud->empty()) {
    return std::make_shared<PointCloud>();
  }

  PointCloud::Ptr filtered(new PointCloud());
  filtered->reserve(cloud->size());
  for (const auto & p : cloud->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      filtered->points.push_back(p);
    }
  }

  filtered->width = static_cast<std::uint32_t>(filtered->points.size());
  filtered->height = 1;
  filtered->is_dense = true;
  return filtered;
}

double computeSafeLeafSize(const PointCloud::Ptr & cloud, double requested_leaf)
{
  constexpr double kMinLeaf = 1.0e-4;
  constexpr double kCellLimit = static_cast<double>(std::numeric_limits<int>::max()) * 0.5;

  double safe_leaf = std::max(requested_leaf, kMinLeaf);
  if (!cloud || cloud->empty()) {
    return safe_leaf;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();

  for (const auto & p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    min_z = std::min(min_z, p.z);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
    max_z = std::max(max_z, p.z);
  }

  if (!std::isfinite(min_x) || !std::isfinite(max_x) || !std::isfinite(min_y) || !std::isfinite(max_y) ||
      !std::isfinite(min_z) || !std::isfinite(max_z)) {
    return safe_leaf;
  }

  const double span_x = static_cast<double>(max_x) - static_cast<double>(min_x);
  const double span_y = static_cast<double>(max_y) - static_cast<double>(min_y);
  const double span_z = static_cast<double>(max_z) - static_cast<double>(min_z);
  const double max_span = std::max({span_x, span_y, span_z, 0.0});

  if (max_span <= 0.0 || !std::isfinite(max_span)) {
    return safe_leaf;
  }

  const double min_leaf_axis = max_span / kCellLimit;
  const double min_leaf_xy = std::sqrt(std::max(0.0, (span_x * span_y) / kCellLimit));
  const double min_leaf_xz = std::sqrt(std::max(0.0, (span_x * span_z) / kCellLimit));
  const double min_leaf_yz = std::sqrt(std::max(0.0, (span_y * span_z) / kCellLimit));
  const double min_leaf_xyz = std::cbrt(std::max(0.0, (span_x * span_y * span_z) / kCellLimit));

  safe_leaf = std::max({safe_leaf, min_leaf_axis, min_leaf_xy, min_leaf_xz, min_leaf_yz, min_leaf_xyz});
  if (!std::isfinite(safe_leaf)) {
    safe_leaf = std::max(requested_leaf, kMinLeaf);
  }

  return safe_leaf;
}

}  // namespace

void publishCurrentTransform(const std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_broadcaster,
  const std::string & map_frame,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Time & stamp)
{
  if (!tf_broadcaster) {
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = map_frame;
  t.child_frame_id = "camera_init";

  const float x = map_to_camera_init(0, 3);
  const float y = map_to_camera_init(1, 3);
  const float yaw = std::atan2(map_to_camera_init(1, 0), map_to_camera_init(0, 0));

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(static_cast<double>(yaw) * 0.5);
  t.transform.rotation.w = std::cos(static_cast<double>(yaw) * 0.5);

  tf_broadcaster->sendTransform(t);
}

void publishStaticTf(const std::shared_ptr<tf2_ros::StaticTransformBroadcaster> & static_tf_broadcaster,
  const std::string & map_frame,
  const std::string & cloud_frame_id,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Time & stamp)
{
  if (!static_tf_broadcaster || cloud_frame_id.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = map_frame;
  t.child_frame_id = cloud_frame_id;

  Eigen::Vector3f tf_pos = map_to_camera_init.block<3, 1>(0, 3);
  Eigen::Quaternionf tf_quat(map_to_camera_init.block<3, 3>(0, 0));

  t.transform.translation.x = tf_pos.x();
  t.transform.translation.y = tf_pos.y();
  t.transform.translation.z = tf_pos.z();
  t.transform.rotation.x = tf_quat.x();
  t.transform.rotation.y = tf_quat.y();
  t.transform.rotation.z = tf_quat.z();
  t.transform.rotation.w = tf_quat.w();

  static_tf_broadcaster->sendTransform(t);
  std::cout << color_text::GREEN << "[GICP] Published static TF: " << map_frame << " -> " << cloud_frame_id
            << color_text::RESET << std::endl;
}

void publishVisualization(const PointCloud::Ptr & cloud,
  const rclcpp::Time & stamp,
  bool visualization_en,
  bool gicp_initialized,
  const std::string & map_frame,
  const Eigen::Matrix4f & map_to_camera_init,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & aligned_cloud_pub)
{
  if (!visualization_en || !cloud || cloud->empty()) {
    return;
  }

  if (aligned_cloud_pub && aligned_cloud_pub->get_subscription_count() > 0 && gicp_initialized) {
    PointCloud::Ptr aligned_cloud(new PointCloud());
    pcl::transformPointCloud(*cloud, *aligned_cloud, map_to_camera_init);

    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned_cloud, aligned_msg);
    aligned_msg.header.frame_id = map_frame;
    aligned_msg.header.stamp = stamp;
    aligned_cloud_pub->publish(aligned_msg);
  }
}

void printEvaluation(
  const Eigen::Matrix4f & initial_guess, const Eigen::Matrix4f & final_transformation, double time_ms)
{
  float init_x = initial_guess(0, 3);
  float init_y = initial_guess(1, 3);
  float final_x = final_transformation(0, 3);
  float final_y = final_transformation(1, 3);

  float dx = final_x - init_x;
  float dy = final_y - init_y;

  float init_yaw = std::atan2(initial_guess(1, 0), initial_guess(0, 0));
  float final_yaw = std::atan2(final_transformation(1, 0), final_transformation(0, 0));
  float dyaw = final_yaw - init_yaw;

  const double PI = 3.14159265358979323846;
  while (dyaw > PI)
    dyaw -= 2 * PI;
  while (dyaw < -PI)
    dyaw += 2 * PI;

  std::cout << color_text::GREEN << "[GICP Eval] Time: " << time_ms << " ms"
            << " | Init[x=" << init_x << ", y=" << init_y << ", yaw=" << init_yaw << "]"
            << " | Final[x=" << final_x << ", y=" << final_y << ", yaw=" << final_yaw << "]"
            << " | Dev[dx=" << dx << ", dy=" << dy << ", dyaw=" << dyaw << "]" << color_text::RESET
            << std::endl;
}

void publishTargetCroppedDebug(bool visualization_en,
  const std::string & map_frame,
  const rclcpp::Time & stamp,
  const GicpFilter * gicp_filter,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_target_cropped)
{
  if (!visualization_en || !pub_target_cropped || !gicp_filter) {
    return;
  }

  PointCloud::Ptr target_cropped = gicp_filter->getCurrentLocalMapCloud();
  if (!target_cropped || target_cropped->empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 target_cropped_msg;
  pcl::toROSMsg(*target_cropped, target_cropped_msg);
  target_cropped_msg.header.frame_id = map_frame;
  target_cropped_msg.header.stamp = stamp;
  pub_target_cropped->publish(target_cropped_msg);
}

PointCloud::Ptr publishSourceCroppedDebug(bool visualization_en,
  const GicpFilter::Options & gicp_options,
  const std::string & cloud_frame,
  const PointCloud::Ptr & source,
  const rclcpp::Time & stamp,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_source_cropped)
{
  if (!visualization_en || !source || source->empty()) {
    return std::make_shared<PointCloud>();
  }

  PointCloud::Ptr source_finite = removeNonFinitePoints(source);
  if (!source_finite || source_finite->empty()) {
    return std::make_shared<PointCloud>();
  }

  PointCloud::Ptr source_height(new PointCloud(*source_finite));
  if (gicp_options.height_filter_enabled) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(source_finite);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(gicp_options.height_filter_min_z, gicp_options.height_filter_max_z);
    source_height = std::make_shared<PointCloud>();
    pass.filter(*source_height);
  }

  PointCloud::Ptr source_cropped(new PointCloud(*source_height));
  if (gicp_options.source_crop_enabled) {
    pcl::CropBox<pcl::PointXYZ> source_crop;
    source_crop.setInputCloud(source_height);
    source_crop.setMin(Eigen::Vector4f(static_cast<float>(gicp_options.source_crop_min_x),
      static_cast<float>(gicp_options.source_crop_min_y),
      static_cast<float>(gicp_options.source_crop_min_z),
      1.0f));
    source_crop.setMax(Eigen::Vector4f(static_cast<float>(gicp_options.source_crop_max_x),
      static_cast<float>(gicp_options.source_crop_max_y),
      static_cast<float>(gicp_options.source_crop_max_z),
      1.0f));
    source_cropped = std::make_shared<PointCloud>();
    source_crop.filter(*source_cropped);
  }

  if (!source_cropped || source_cropped->empty()) {
    return std::make_shared<PointCloud>();
  }

  const double safe_leaf = computeSafeLeafSize(source_cropped, gicp_options.source_voxel_leaf_size);

  PointCloud::Ptr source_voxel(new PointCloud());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(safe_leaf, safe_leaf, safe_leaf);
  vg.setInputCloud(source_cropped);
  vg.filter(*source_voxel);
  if (!source_voxel || source_voxel->empty()) {
    return std::make_shared<PointCloud>();
  }

  if (pub_source_cropped && pub_source_cropped->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 source_cropped_msg;
    pcl::toROSMsg(*source_voxel, source_cropped_msg);
    source_cropped_msg.header.frame_id = cloud_frame;
    source_cropped_msg.header.stamp = stamp;
    pub_source_cropped->publish(source_cropped_msg);
  }

  return source_voxel;
}

}  // namespace icp_relocalization::gicp_utils
