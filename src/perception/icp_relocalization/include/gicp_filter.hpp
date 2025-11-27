#pragma once

#include <Eigen/Core>
#include <memory>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <string>

namespace icp_relocalization
{

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointNormal = pcl::PointCloud<pcl::Normal>;
  using Feature = pcl::FPFHSignature33;
  using FPFHFeature = pcl::PointCloud<Feature>;

  // GICP算法的核心封装，不含任何ROS依赖
  class GicpFilter
  {
  public:
    struct Options
    {
      // Voxel Grid
      double target_voxel_leaf_size = 2.0;
      double source_voxel_leaf_size = 2.0;

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
    // 初始化处理流程
    void preprocessMap(const PointCloud::Ptr& cloud);

    // 计算点云的FPFH特征
    FPFHFeature::Ptr computeFPFH(const PointCloud::Ptr& cloud);

    Options options_;
    PointCloud::Ptr target_cloud_filtered_;
    FPFHFeature::Ptr target_features_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr target_tree_;
  };

}  // namespace icp_relocalization
