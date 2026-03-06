#include "gicp_filter.hpp"
#include <iostream>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

namespace icp_relocalization
{

  PointCloud::Ptr GicpFilter::applyHeightFilter(const PointCloud::Ptr& cloud) const
  {
    if(!options_.height_filter_enabled || !cloud || cloud->empty())
    {
      return cloud;
    }

    PointCloud::Ptr filtered(new PointCloud());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(options_.height_filter_min_z, options_.height_filter_max_z);
    pass.filter(*filtered);
    return filtered;
  }

  PointCloud::Ptr GicpFilter::cropSourceCloud(const PointCloud::Ptr& source_cloud) const
  {
    if(!source_cloud || source_cloud->empty())
    {
      return std::make_shared<PointCloud>();
    }

    if(!options_.source_crop_enabled)
    {
      return source_cloud;
    }

    PointCloud::Ptr source_cropped(new PointCloud());
    pcl::CropBox<pcl::PointXYZ> source_crop;
    source_crop.setInputCloud(source_cloud);
    source_crop.setMin(Eigen::Vector4f(static_cast<float>(options_.source_crop_min_x),
                                       static_cast<float>(options_.source_crop_min_y),
                                       static_cast<float>(options_.source_crop_min_z),
                                       1.0f));
    source_crop.setMax(Eigen::Vector4f(static_cast<float>(options_.source_crop_max_x),
                                       static_cast<float>(options_.source_crop_max_y),
                                       static_cast<float>(options_.source_crop_max_z),
                                       1.0f));
    source_crop.filter(*source_cropped);

    if(options_.source_self_crop_enabled)
    {
      PointCloud::Ptr source_no_self(new PointCloud());
      pcl::CropBox<pcl::PointXYZ> self_crop;
      self_crop.setInputCloud(source_cropped);
      self_crop.setMin(Eigen::Vector4f(static_cast<float>(options_.source_self_crop_min_x),
                                       static_cast<float>(options_.source_self_crop_min_y),
                                       static_cast<float>(options_.source_self_crop_min_z),
                                       1.0f));
      self_crop.setMax(Eigen::Vector4f(static_cast<float>(options_.source_self_crop_max_x),
                                       static_cast<float>(options_.source_self_crop_max_y),
                                       static_cast<float>(options_.source_self_crop_max_z),
                                       1.0f));
      self_crop.setNegative(true);
      self_crop.filter(*source_no_self);
      source_cropped = source_no_self;
    }

    return source_cropped;
  }

  GicpFilter::GicpFilter(const std::string& target_pcd_path, const Options& options)
    : options_(options)
  {
    PointCloud::Ptr target_cloud(new PointCloud());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_path, *target_cloud) == -1)
    {
      throw std::runtime_error("Couldn't read file " + target_pcd_path);
    }
    preprocessMap(target_cloud);
  }

  GicpFilter::GicpFilter(const PointCloud::Ptr& target_cloud, const Options& options)
    : options_(options)
  {
    preprocessMap(target_cloud);
  }

  void GicpFilter::preprocessMap(const PointCloud::Ptr& cloud)
  {
    // 移除NaN点
    PointCloud::Ptr cloud_no_nan(new PointCloud());
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

    // 高度滤波（可选）：先裁剪再降采样/特征，减少计算量
    PointCloud::Ptr cloud_filtered = applyHeightFilter(cloud_no_nan);

    // 保存完整全局地图
    global_map_ = std::make_shared<PointCloud>();
    *global_map_ = *cloud_filtered;

    // 对地图进行降采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(options_.target_voxel_leaf_size, options_.target_voxel_leaf_size, options_.target_voxel_leaf_size);
    vg.setInputCloud(cloud_filtered);
    target_cloud_filtered_ = std::make_shared<PointCloud>();
    vg.filter(*target_cloud_filtered_);

    // 为降采样后的地图计算FPFH特征
    target_features_ = computeFPFH(target_cloud_filtered_);

    // small_gicp 后端使用的地图预处理：voxel + covariance + kd-tree
    constexpr int kCovNeighbors = 20;
    constexpr int kCovThreads = 8;

    small_gicp_target_ =
        small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(*cloud_filtered,
                                                                             options_.target_voxel_leaf_size);
    small_gicp::estimate_covariances_omp(*small_gicp_target_, kCovNeighbors, kCovThreads);

    target_tree_ = std::make_shared<SmallGicpKdTree>(small_gicp_target_,
                                                     small_gicp::KdTreeBuilderOMP(kCovThreads));
  }

  void GicpFilter::updateLocalMap(const Eigen::Matrix4f& current_pose)
  {
    if(!global_map_ || global_map_->empty())
    {
      return;
    }

    const Eigen::Vector3f current_pos = current_pose.block<3, 1>(0, 3);
    if(local_map_initialized_)
    {
      const double center_shift = (current_pos - last_local_map_center_).norm();
      if(center_shift < 5.0)
      {
        return;
      }
    }

    PointCloud::Ptr cropped_cloud(new PointCloud());
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(global_map_);
    crop_box.setMin(Eigen::Vector4f(current_pos.x() - local_map_radius_,
                                    current_pos.y() - local_map_radius_,
                                    current_pos.z() - local_map_radius_,
                                    1.0f));
    crop_box.setMax(Eigen::Vector4f(current_pos.x() + local_map_radius_,
                                    current_pos.y() + local_map_radius_,
                                    current_pos.z() + local_map_radius_,
                                    1.0f));
    crop_box.filter(*cropped_cloud);

    if(!cropped_cloud || cropped_cloud->empty())
    {
      return;
    }

    constexpr int kCovNeighbors = 20;
    constexpr int kCovThreads = 8;

    small_gicp_target_ =
        small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(*cropped_cloud,
                                                                             options_.target_voxel_leaf_size);

    if(!small_gicp_target_ || small_gicp_target_->empty())
    {
      return;
    }

    small_gicp::estimate_covariances_omp(*small_gicp_target_, kCovNeighbors, kCovThreads);
    target_tree_ = std::make_shared<SmallGicpKdTree>(small_gicp_target_,
                                                     small_gicp::KdTreeBuilderOMP(kCovThreads));

    last_local_map_center_ = current_pos;
    local_map_initialized_ = true;
  }

  FPFHFeature::Ptr GicpFilter::computeFPFH(const PointCloud::Ptr& cloud)
  {
    // 估算法线
    PointNormal::Ptr normals(new PointNormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch(options_.feature_k_search);
    norm_est.setInputCloud(cloud);
    norm_est.compute(*normals);

    // 估算FPFH特征
    FPFHFeature::Ptr features(new FPFHFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setKSearch(options_.feature_k_search);
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.compute(*features);

    return features;
  }

  GicpFilter::Result GicpFilter::initialAlign(const PointCloud::Ptr& source_cloud)
  {
    // 高度滤波（可选）：先裁剪再降采样/特征，减少计算量
    PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);

    // 先验点云同样通过统一的crop流程
    PointCloud::Ptr source_cropped = cropSourceCloud(source_filtered_height);
    if(!source_cropped || source_cropped->empty())
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    // 1. 对源点云进行降采样
    PointCloud::Ptr source_cloud_filtered(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(options_.source_voxel_leaf_size, options_.source_voxel_leaf_size, options_.source_voxel_leaf_size);
    vg.setInputCloud(source_cropped);
    vg.filter(*source_cloud_filtered);

    // 2. 为源点云计算FPFH特征
    FPFHFeature::Ptr source_features = computeFPFH(source_cloud_filtered);

    // 3. 配置并运行SAC-IA进行粗略对齐
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMinSampleDistance(options_.sac_ia_min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance(options_.sac_ia_max_correspondence_distance);
    sac_ia.setMaximumIterations(1000);  // 增加迭代次数以提高成功率
    sac_ia.setCorrespondenceRandomness(options_.sac_ia_correspondence_randomness);
    sac_ia.setNumberOfSamples(options_.sac_ia_num_samples);

    sac_ia.setInputSource(source_cloud_filtered);
    sac_ia.setSourceFeatures(source_features);

    sac_ia.setInputTarget(target_cloud_filtered_);
    sac_ia.setTargetFeatures(target_features_);

    PointCloud final_cloud;
    sac_ia.align(final_cloud);

    if(!sac_ia.hasConverged())
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    // 4. 使用SAC-IA的结果作为GICP的初始猜测进行精细对齐
    return align(source_cloud, sac_ia.getFinalTransformation());
  }

  GicpFilter::Result GicpFilter::align(const PointCloud::Ptr& source_cloud, const Eigen::Matrix4f& initial_guess)
  {
    // 高度滤波
    PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);
    if(!source_filtered_height || source_filtered_height->empty() || !global_map_ || global_map_->empty())
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    constexpr int kCovNeighbors = 20;
    constexpr int kCovThreads = 8;

    PointCloud::Ptr source_cropped = cropSourceCloud(source_filtered_height);

    if(!source_cropped || source_cropped->empty())
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    SmallGicpPointCloud::Ptr source_small =
        small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(*source_cropped,
                                                                             options_.source_voxel_leaf_size);
    if(!source_small || source_small->empty())
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    small_gicp::estimate_covariances_omp(*source_small, kCovNeighbors, kCovThreads);
    source_tree_ = std::make_shared<SmallGicpKdTree>(source_small,
                                                     small_gicp::KdTreeBuilderOMP(kCovThreads));

    // 在精配准前根据当前先验位姿更新局部地图
    updateLocalMap(initial_guess);
    if(!small_gicp_target_ || small_gicp_target_->empty() || !target_tree_)
    {
      Result result;
      result.converged = false;
      result.fitness_score = -1.0;
      return result;
    }

    SmallGicpRegister register_engine;
    register_engine.reduction.num_threads = kCovThreads;
    register_engine.rejector.max_dist_sq =
        options_.max_correspondence_distance * options_.max_correspondence_distance;
    register_engine.optimizer.max_iterations = options_.max_iterations;

    Eigen::Isometry3d init_guess_d = Eigen::Isometry3d::Identity();
    init_guess_d.matrix() = initial_guess.cast<double>();

    const small_gicp::RegistrationResult small_gicp_result =
        register_engine.align(*small_gicp_target_, *source_small, *target_tree_, init_guess_d);

    // 整理结果
    Result result;
    result.converged = small_gicp_result.converged;
    result.fitness_score = small_gicp_result.error;
    result.final_transformation = small_gicp_result.T_target_source.matrix().cast<float>();

    return result;
  }

}  // namespace icp_relocalization
