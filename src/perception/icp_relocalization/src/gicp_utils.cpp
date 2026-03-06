#include "gicp_utils.hpp"

#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "color_text.hpp"

namespace icp_relocalization::gicp_utils
{

void publishStaticTf(const std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& static_tf_broadcaster,
                     const std::string& map_frame,
                     const std::string& cloud_frame_id,
                     const Eigen::Matrix4f& map_to_camera_init,
                     const rclcpp::Time& stamp)
{
  if(!static_tf_broadcaster || cloud_frame_id.empty())
  {
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
  std::cout << color_text::MAGENTA << "[GICP] Published static TF: "
            << map_frame << " -> " << cloud_frame_id << color_text::RESET << std::endl;
}

void publishVisualization(const PointCloud::Ptr& cloud,
                          const rclcpp::Time& stamp,
                          bool visualization_en,
                          bool gicp_initialized,
                          const std::string& map_frame,
                          const Eigen::Matrix4f& map_to_camera_init,
                          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& source_pub,
                          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& aligned_cloud_pub)
{
  if(!visualization_en || !cloud)
  {
    return;
  }

  if(source_pub && source_pub->get_subscription_count() > 0)
  {
    sensor_msgs::msg::PointCloud2 source_msg;
    pcl::toROSMsg(*cloud, source_msg);
    source_msg.header.frame_id = "camera_init";
    source_msg.header.stamp = stamp;
    source_pub->publish(source_msg);
  }

  if(aligned_cloud_pub && aligned_cloud_pub->get_subscription_count() > 0 && gicp_initialized)
  {
    PointCloud::Ptr aligned_cloud(new PointCloud());
    pcl::transformPointCloud(*cloud, *aligned_cloud, map_to_camera_init);

    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned_cloud, aligned_msg);
    aligned_msg.header.frame_id = map_frame;
    aligned_msg.header.stamp = stamp;
    aligned_cloud_pub->publish(aligned_msg);
  }
}

void printEvaluation(const Eigen::Matrix4f& initial_guess,
                     const Eigen::Matrix4f& final_transformation,
                     double fitness_score,
                     double time_ms)
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
  while(dyaw > PI) dyaw -= 2 * PI;
  while(dyaw < -PI) dyaw += 2 * PI;

  std::cout << color_text::GREEN << "--------------------------------------------------" << color_text::RESET << std::endl;
  printf("%s[GICP Eval] Score: %.4f, Time: %.2f ms%s\n", color_text::GREEN.c_str(), fitness_score, time_ms, color_text::RESET.c_str());
  printf("%sExpected(Init): x=%.3f, y=%.3f, yaw=%.3f%s\n", color_text::GREEN.c_str(), init_x, init_y, init_yaw, color_text::RESET.c_str());
  printf("%sActual(Final) : x=%.3f, y=%.3f, yaw=%.3f%s\n", color_text::GREEN.c_str(), final_x, final_y, final_yaw, color_text::RESET.c_str());
  printf("%sDeviation     : dx=%.3f, dy=%.3f, dyaw=%.3f%s\n", color_text::GREEN.c_str(), dx, dy, dyaw, color_text::RESET.c_str());
  std::cout << color_text::GREEN << "--------------------------------------------------" << color_text::RESET << std::endl;
}

void publishTargetCroppedDebug(bool visualization_en,
                               const std::string& map_frame,
                               const rclcpp::Time& stamp,
                               const GicpFilter* gicp_filter,
                               const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_target_cropped)
{
  if(!visualization_en || !pub_target_cropped || !gicp_filter)
  {
    return;
  }

  PointCloud::Ptr target_cropped = gicp_filter->getCurrentLocalMapCloud();
  if(!target_cropped || target_cropped->empty())
  {
    return;
  }

  sensor_msgs::msg::PointCloud2 target_cropped_msg;
  pcl::toROSMsg(*target_cropped, target_cropped_msg);
  target_cropped_msg.header.frame_id = map_frame;
  target_cropped_msg.header.stamp = stamp;
  pub_target_cropped->publish(target_cropped_msg);
}

void publishSourceCroppedDebug(bool visualization_en,
                               const GicpFilter::Options& gicp_options,
                               const std::string& map_frame,
                               const PointCloud::Ptr& source,
                               const Eigen::Matrix4f& initial_guess,
                               const rclcpp::Time& stamp,
                               const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_source_cropped)
{
  if(!visualization_en || !pub_source_cropped || !source || source->empty())
  {
    return;
  }

  PointCloud::Ptr source_voxel(new PointCloud());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(gicp_options.source_voxel_leaf_size,
                 gicp_options.source_voxel_leaf_size,
                 gicp_options.source_voxel_leaf_size);
  vg.setInputCloud(source);
  vg.filter(*source_voxel);

  PointCloud::Ptr source_height(new PointCloud(*source_voxel));
  if(gicp_options.height_filter_enabled)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(source_voxel);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(gicp_options.height_filter_min_z, gicp_options.height_filter_max_z);
    source_height = std::make_shared<PointCloud>();
    pass.filter(*source_height);
  }

  PointCloud::Ptr source_cropped(new PointCloud(*source_height));
  if(gicp_options.source_crop_enabled)
  {
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

  if(!source_cropped || source_cropped->empty())
  {
    return;
  }

  PointCloud::Ptr source_cropped_in_map(new PointCloud());
  pcl::transformPointCloud(*source_cropped, *source_cropped_in_map, initial_guess);

  sensor_msgs::msg::PointCloud2 source_cropped_msg;
  pcl::toROSMsg(*source_cropped_in_map, source_cropped_msg);
  source_cropped_msg.header.frame_id = map_frame;
  source_cropped_msg.header.stamp = stamp;
  pub_source_cropped->publish(source_cropped_msg);
}

}  // namespace icp_relocalization::gicp_utils
