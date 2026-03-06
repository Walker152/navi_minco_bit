#pragma once

#include <Eigen/Core>
#include <memory>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include <string>

namespace icp_relocalization
{

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointNormal = pcl::PointCloud<pcl::Normal>;
  using Feature = pcl::FPFHSignature33;
  using FPFHFeature = pcl::PointCloud<Feature>;
  using SmallGicpPointCloud = pcl::PointCloud<pcl::PointCovariance>;
  using SmallGicpKdTree = small_gicp::KdTree<SmallGicpPointCloud>;
  using SmallGicpRegister =
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;

  // GICP算法的核心封装，不含任何ROS依赖
  class GicpFilter
  {
  public:
    struct Options
    {
      // Height filter (PassThrough on z)
      bool height_filter_enabled = false;
      double height_filter_min_z = -1000.0;
      double height_filter_max_z = 1000.0;

      // Voxel Grid
      double target_voxel_leaf_size = 2.0;
      double source_voxel_leaf_size = 2.0;

      // Source local crop for registration input
      bool source_crop_enabled = true;
      double source_crop_min_x = -20.0;
      double source_crop_max_x = 20.0;
      double source_crop_min_y = -20.0;
      double source_crop_max_y = 20.0;
      double source_crop_min_z = -2.5;
      double source_crop_max_z = 2.5;

      // Optional self-body exclusion box around origin
      bool source_self_crop_enabled = false;
      double source_self_crop_min_x = -1.0;
      double source_self_crop_max_x = 1.5;
      double source_self_crop_min_y = -0.8;
      double source_self_crop_max_y = 0.8;
      double source_self_crop_min_z = -1.5;
      double source_self_crop_max_z = 1.8;

      // SAC-IA
      double sac_ia_min_sample_distance = 0.5;
      int sac_ia_correspondence_randomness = 6;
      int sac_ia_num_samples = 3;
      double sac_ia_max_correspondence_distance = 1.0;
      int feature_k_search = 10;

      // GICP
      double max_correspondence_distance = 10.0;
      int max_iterations = 100;
      double transformation_epsilon = 0.01;
      double euclidean_fitness_epsilon = 0.01;
    };

    struct Result
    {
      bool converged = false;
      double fitness_score = -1.0;
      Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
    };

    // 构造函数，加载地图并进行预处理
    explicit GicpFilter(const std::string& target_pcd_path, const Options& options);

    // 构造函数，直接使用点云数据
    GicpFilter(const PointCloud::Ptr& target_cloud, const Options& options);

    // 全局初始定位：使用SAC-IA粗定位 + GICP精细定位
    Result initialAlign(const PointCloud::Ptr& source_cloud);

    // 增量定位：使用给定的初始猜测进行GICP定位
    Result align(const PointCloud::Ptr& source_cloud, const Eigen::Matrix4f& initial_guess);

    // 获取预处理后的目标点云（地图）
    PointCloud::Ptr getTargetCloud() const
    {
      return target_cloud_filtered_;
    }

  private:
    // Apply optional height(z) pass-through filter.
    // Returns the original cloud when disabled.
    PointCloud::Ptr applyHeightFilter(const PointCloud::Ptr& cloud) const;

    // 对 source cloud 进行空间裁剪（含可选自车剔除）
    PointCloud::Ptr cropSourceCloud(const PointCloud::Ptr& source_cloud) const;

    // 初始化处理流程
    void preprocessMap(const PointCloud::Ptr& cloud);

    // 计算点云的FPFH特征
    FPFHFeature::Ptr computeFPFH(const PointCloud::Ptr& cloud);

    // 根据当前位姿更新局部地图
    void updateLocalMap(const Eigen::Matrix4f& current_pose);

    Options options_;
    PointCloud::Ptr target_cloud_filtered_;
    FPFHFeature::Ptr target_features_;
    SmallGicpPointCloud::Ptr small_gicp_target_;
    SmallGicpKdTree::Ptr target_tree_;
    SmallGicpKdTree::Ptr source_tree_;

    Eigen::Vector3f last_local_map_center_ = Eigen::Vector3f::Zero();
    bool local_map_initialized_ = false;
    double local_map_radius_ = 20.0;
    PointCloud::Ptr global_map_;
  };

}  // namespace icp_relocalization
