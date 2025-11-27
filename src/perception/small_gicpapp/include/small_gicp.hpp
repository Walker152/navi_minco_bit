#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace small_gicp_relocalization
{

  class SmallGicpRelocalizationNode : public rclcpp::Node
  {
  public:
    explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions& options);

  private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void loadGlobalMap(const std::string& file_name);
    void performRegistration();
    void printRegistrationResult();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;

    int num_threads_;
    int num_neighbors_;
    float global_leaf_size_;
    float registered_leaf_size_;
    float max_dist_sq_;
    std::vector<double> init_pose_;

    std::string prior_pcd_file_;
    Eigen::Isometry3d result_t_;
    Eigen::Isometry3d init_guess_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
    pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;

    // 帧计数相关
    int frames_to_accumulate_;  // 需要累积的帧数
    int current_frame_count_;   // 当前已累积的帧数

    // 配准成功次数
    int pass_count_;

    // 静态TF广播器
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  };

}  // namespace small_gicp_relocalization
