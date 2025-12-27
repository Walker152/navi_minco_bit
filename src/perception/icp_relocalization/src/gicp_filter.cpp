#include "gicp_filter.hpp"
#include <iostream>

#include <pcl/filters/passthrough.h>

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

    // 对地图进行降采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(options_.target_voxel_leaf_size, options_.target_voxel_leaf_size, options_.target_voxel_leaf_size);
    vg.setInputCloud(cloud_filtered);
    target_cloud_filtered_ = std::make_shared<PointCloud>();
    vg.filter(*target_cloud_filtered_);

    // 为降采样后的地图计算FPFH特征
    target_features_ = computeFPFH(target_cloud_filtered_);
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

    // 1. 对源点云进行降采样
    PointCloud::Ptr source_cloud_filtered(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(options_.source_voxel_leaf_size, options_.source_voxel_leaf_size, options_.source_voxel_leaf_size);
    vg.setInputCloud(source_filtered_height);
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
    // 高度滤波（可选）：先裁剪再降采样，减少计算量
    PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);

    // 对源点云进行降采样
    PointCloud::Ptr source_cloud_filtered(new PointCloud());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(options_.source_voxel_leaf_size, options_.source_voxel_leaf_size, options_.source_voxel_leaf_size);
    vg.setInputCloud(source_filtered_height);
    vg.filter(*source_cloud_filtered);

    // 配置GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_cloud_filtered);
    gicp.setInputTarget(target_cloud_filtered_);
    gicp.setMaxCorrespondenceDistance(options_.max_correspondence_distance);
    gicp.setMaximumIterations(options_.max_iterations);
    gicp.setTransformationEpsilon(options_.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(options_.euclidean_fitness_epsilon);

    // 执行配准
    PointCloud final_cloud;
    gicp.align(final_cloud, initial_guess);

    // 整理结果
    Result result;
    result.converged = gicp.hasConverged();
    result.fitness_score = gicp.getFitnessScore();
    result.final_transformation = gicp.getFinalTransformation();

    return result;
  }

}  // namespace icp_relocalization
