#include "gicp_filter.hpp"
#include <iostream>
#include <limits>
#include <cmath>

#include <Eigen/Geometry>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

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
    local_map_cloud_ = target_cloud_filtered_;
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

    local_map_cloud_ = cropped_cloud;

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

  GicpFilter::Result GicpFilter::initialAlign(const PointCloud::Ptr& source_cloud)
  {
    PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);
    PointCloud::Ptr source_cropped = cropSourceCloud(source_filtered_height);

    if(!source_cropped || source_cropped->empty())
    {
      Result result;
      result.converged = false;
      result.final_transformation = Eigen::Matrix4f::Identity();
      return result;
    }

    constexpr int kCovNeighbors = 20;
    constexpr int kCovThreads = 8;

    SmallGicpPointCloud::Ptr source_small =
        small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(*source_cropped,
                                                                             options_.source_voxel_leaf_size);
    if(!source_small || source_small->empty())
    {
      Result result;
      result.converged = false;
      result.final_transformation = Eigen::Matrix4f::Identity();
      return result;
    }

    small_gicp::estimate_covariances_omp(*source_small, kCovNeighbors, kCovThreads);
    source_tree_ = std::make_shared<SmallGicpKdTree>(source_small,
                                                     small_gicp::KdTreeBuilderOMP(kCovThreads));

    if(!small_gicp_target_ || small_gicp_target_->empty() || !target_tree_)
    {
      Result result;
      result.converged = false;
      result.final_transformation = Eigen::Matrix4f::Identity();
      return result;
    }

    if(options_.step_x <= 0.0 || options_.step_y <= 0.0 || options_.step_yaw <= 0.0)
    {
      Result result;
      result.converged = false;
      result.final_transformation = Eigen::Matrix4f::Identity();
      return result;
    }

    if(options_.search_areas.empty() || options_.z_candidates.empty())
    {
      Result result;
      result.converged = false;
      result.final_transformation = Eigen::Matrix4f::Identity();
      return result;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(local_map_cloud_);

    double best_overlap_ratio = 0.0;
    Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
    bool best_converged = false;

    SmallGicpRegister register_engine;
    register_engine.reduction.num_threads = kCovThreads;
    register_engine.rejector.max_dist_sq =
        options_.max_correspondence_distance * options_.max_correspondence_distance;
    register_engine.optimizer.max_iterations = options_.max_iterations;

    for(double z : options_.z_candidates)
    {
      for(const auto& area : options_.search_areas)
      {
        for(double x = area.min_x; x <= area.max_x; x += options_.step_x)
        {
          for(double y = area.min_y; y <= area.max_y; y += options_.step_y)
          {
            for(double yaw = -M_PI; yaw <= M_PI; yaw += options_.step_yaw)
            {
              Eigen::Isometry3d init_guess_d = Eigen::Isometry3d::Identity();
              init_guess_d.translation() << x, y, z;
              init_guess_d.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

              const small_gicp::RegistrationResult reg_result =
                  register_engine.align(*small_gicp_target_, *source_small, *target_tree_, init_guess_d);

              if(!reg_result.converged)
              {
                continue;
              }

              PointCloud::Ptr transformed_source(new PointCloud());
              pcl::transformPointCloud(*source_cropped,
                                       *transformed_source,
                                       reg_result.T_target_source.matrix().cast<float>());

              if(!transformed_source || transformed_source->empty())
              {
                continue;
              }

              std::vector<int> nearest_indices(1);
              std::vector<float> nearest_dist_sq(1);
              size_t inlier_count = 0;

              for(const auto& point : transformed_source->points)
              {
                if(kdtree.nearestKSearch(point, 1, nearest_indices, nearest_dist_sq) > 0 &&
                   nearest_dist_sq[0] < 0.09f)
                {
                  ++inlier_count;
                }
              }

              const double overlap_ratio =
                  static_cast<double>(inlier_count) / static_cast<double>(transformed_source->points.size());

              if(overlap_ratio > best_overlap_ratio)
              {
                best_overlap_ratio = overlap_ratio;
                best_pose = reg_result.T_target_source.matrix().cast<float>();
                best_converged = true;
              }
            }
          }
        }
      }
    }

    Result result;
    result.converged = best_converged;
    result.overlap_ratio = best_overlap_ratio;
    result.final_transformation = best_pose;
    return result;
  }

  GicpFilter::Result GicpFilter::align(const PointCloud::Ptr& source_cloud, const Eigen::Matrix4f& initial_guess)
  {
    // 高度滤波
    PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);
    if(!source_filtered_height || source_filtered_height->empty() || !global_map_ || global_map_->empty())
    {
      Result result;
      result.converged = false;
      return result;
    }

    constexpr int kCovNeighbors = 20;
    constexpr int kCovThreads = 8;

    PointCloud::Ptr source_cropped = cropSourceCloud(source_filtered_height);

    if(!source_cropped || source_cropped->empty())
    {
      Result result;
      result.converged = false;
      return result;
    }

    SmallGicpPointCloud::Ptr source_small =
        small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(*source_cropped,
                                                                             options_.source_voxel_leaf_size);
    if(!source_small || source_small->empty())
    {
      Result result;
      result.converged = false;
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
    result.final_transformation = small_gicp_result.T_target_source.matrix().cast<float>();

    if(small_gicp_result.converged && local_map_cloud_ && !local_map_cloud_->empty())
    {
      PointCloud::Ptr transformed_source(new PointCloud());
      pcl::transformPointCloud(*source_cropped, *transformed_source, result.final_transformation);

      if(transformed_source && !transformed_source->empty())
      {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(local_map_cloud_);

        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_dist_sq(1);
        size_t inlier_count = 0;

        for(const auto& point : transformed_source->points)
        {
          if(kdtree.nearestKSearch(point, 1, nearest_indices, nearest_dist_sq) > 0 &&
             nearest_dist_sq[0] < 0.09f)
          {
            ++inlier_count;
          }
        }

        result.overlap_ratio = static_cast<double>(inlier_count) /
                               static_cast<double>(transformed_source->points.size());
      }
    }

    return result;
  }

}  // namespace icp_relocalization
