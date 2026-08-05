#include "gicp_filter.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace icp_relocalization {

namespace {

SmallGicpPointCloud::Ptr buildSourceSmallGicpCloud(
  const PointCloud::Ptr & source_cloud, const double voxel_leaf_size)
{
  if (!source_cloud || source_cloud->empty()) {
    return std::make_shared<SmallGicpPointCloud>();
  }

  if (voxel_leaf_size > 0.0) {
    return small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(
      *source_cloud, voxel_leaf_size);
  }

  SmallGicpPointCloud::Ptr source_small(new SmallGicpPointCloud());
  source_small->points.reserve(source_cloud->points.size());
  for (const auto & pt : source_cloud->points) {
    pcl::PointCovariance q;
    q.x = pt.x;
    q.y = pt.y;
    q.z = pt.z;
    source_small->points.push_back(q);
  }
  source_small->width = static_cast<uint32_t>(source_small->points.size());
  source_small->height = 1;
  source_small->is_dense = source_cloud->is_dense;
  return source_small;
}

}  // namespace

PointCloud::Ptr GicpFilter::applyHeightFilter(const PointCloud::Ptr & cloud) const
{
  if (!options_.height_filter_enabled || !cloud || cloud->empty()) {
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

PointCloud::Ptr GicpFilter::cropSourceCloud(const PointCloud::Ptr & source_cloud) const
{
  if (!source_cloud || source_cloud->empty()) {
    return std::make_shared<PointCloud>();
  }

  if (!options_.source_crop_enabled) {
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

GicpFilter::GicpFilter(const std::string & target_pcd_path, const Options & options)
: options_(options), local_map_radius_(options.local_map_radius)
{
  PointCloud::Ptr target_cloud(new PointCloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_path, *target_cloud) == -1) {
    throw std::runtime_error("Couldn't read file " + target_pcd_path);
  }
  preprocessMap(target_cloud);
}

GicpFilter::GicpFilter(const PointCloud::Ptr & target_cloud, const Options & options)
: options_(options), local_map_radius_(options.local_map_radius)
{
  preprocessMap(target_cloud);
  local_map_cloud_ = target_cloud_filtered_;
}

void GicpFilter::preprocessMap(const PointCloud::Ptr & cloud)
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
  vg.setLeafSize(
    options_.target_voxel_leaf_size, options_.target_voxel_leaf_size, options_.target_voxel_leaf_size);
  vg.setInputCloud(cloud_filtered);
  target_cloud_filtered_ = std::make_shared<PointCloud>();
  vg.filter(*target_cloud_filtered_);

  // small_gicp 后端使用的地图预处理：voxel + covariance + kd-tree
  constexpr int kCovNeighbors = 20;
  constexpr int kCovThreads = 8;

  small_gicp_target_ = small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(
    *cloud_filtered, options_.target_voxel_leaf_size);
  small_gicp::estimate_covariances_omp(*small_gicp_target_, kCovNeighbors, kCovThreads);

  target_tree_ =
    std::make_shared<SmallGicpKdTree>(small_gicp_target_, small_gicp::KdTreeBuilderOMP(kCovThreads));
}

void GicpFilter::updateLocalMap(const Eigen::Matrix4f & current_pose)
{
  if (!global_map_ || global_map_->empty()) {
    return;
  }

  const Eigen::Vector3f current_pos = current_pose.block<3, 1>(0, 3);
  if (local_map_initialized_) {
    const double center_shift = (current_pos - last_local_map_center_).norm();
    if (center_shift < 5.0) {
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

  if (!cropped_cloud || cropped_cloud->empty()) {
    return;
  }

  local_map_cloud_ = cropped_cloud;

  constexpr int kCovNeighbors = 20;
  constexpr int kCovThreads = 8;

  small_gicp_target_ = small_gicp::voxelgrid_sampling_omp<PointCloud, SmallGicpPointCloud>(
    *cropped_cloud, options_.target_voxel_leaf_size);

  if (!small_gicp_target_ || small_gicp_target_->empty()) {
    return;
  }

  small_gicp::estimate_covariances_omp(*small_gicp_target_, kCovNeighbors, kCovThreads);
  target_tree_ =
    std::make_shared<SmallGicpKdTree>(small_gicp_target_, small_gicp::KdTreeBuilderOMP(kCovThreads));

  last_local_map_center_ = current_pos;
  local_map_initialized_ = true;
}

void GicpFilter::updateResultQuality(Result & result,
  const PointCloud::Ptr & source_cropped,
  const std::size_t source_points,
  const double max_correspondence_distance) const
{
  result.source_points = source_points;
  if (result.num_inliers > 0 && std::isfinite(result.score)) {
    result.normalized_score = result.score / static_cast<double>(result.num_inliers);
  }
  if (source_points > 0) {
    result.inlier_ratio = static_cast<double>(result.num_inliers) / static_cast<double>(source_points);
  }

  if (!result.converged || !source_cropped || source_cropped->empty() || !local_map_cloud_ ||
      local_map_cloud_->empty()) {
    return;
  }

  Eigen::Vector2d source_center = Eigen::Vector2d::Zero();
  for (const auto & point : source_cropped->points) {
    source_center += Eigen::Vector2d(point.x, point.y);
  }
  source_center /= static_cast<double>(source_cropped->size());

  double squared_radius_sum = 0.0;
  for (const auto & point : source_cropped->points) {
    const Eigen::Vector2d offset = Eigen::Vector2d(point.x, point.y) - source_center;
    squared_radius_sum += offset.squaredNorm();
  }
  result.planar_yaw_scale =
    std::max(1.0, std::sqrt(squared_radius_sum / static_cast<double>(source_cropped->size())));

  if (result.num_inliers > 0 && result.information.allFinite()) {
    constexpr std::array<int, 3> kPlanarIndices = {2, 3, 4};    // yaw, x, y
    constexpr std::array<int, 3> kNuisanceIndices = {0, 1, 5};  // roll, pitch, z
    Eigen::Matrix<double, 6, 6> normalized_information =
      result.information / static_cast<double>(result.num_inliers);
    normalized_information = 0.5 * (normalized_information + normalized_information.transpose()).eval();

    Eigen::Matrix3d planar_information = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d nuisance_information = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d planar_nuisance_information = Eigen::Matrix3d::Zero();
    for (std::size_t row = 0; row < kPlanarIndices.size(); ++row) {
      for (std::size_t col = 0; col < kPlanarIndices.size(); ++col) {
        planar_information(row, col) = normalized_information(kPlanarIndices[row], kPlanarIndices[col]);
        nuisance_information(row, col) =
          normalized_information(kNuisanceIndices[row], kNuisanceIndices[col]);
        planar_nuisance_information(row, col) =
          normalized_information(kPlanarIndices[row], kNuisanceIndices[col]);
      }
    }

    // Marginalize roll/pitch/z so a strong ground constraint cannot hide weak XY/yaw geometry.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> nuisance_solver(nuisance_information);
    if (nuisance_solver.info() == Eigen::Success && nuisance_solver.eigenvalues().allFinite()) {
      const double max_nuisance_eigenvalue = nuisance_solver.eigenvalues().cwiseAbs().maxCoeff();
      const double inverse_threshold = std::max(1.0e-12, max_nuisance_eigenvalue * 1.0e-9);
      Eigen::Vector3d inverse_eigenvalues = Eigen::Vector3d::Zero();
      for (Eigen::Index i = 0; i < inverse_eigenvalues.size(); ++i) {
        if (nuisance_solver.eigenvalues()(i) > inverse_threshold) {
          inverse_eigenvalues(i) = 1.0 / nuisance_solver.eigenvalues()(i);
        }
      }
      const Eigen::Matrix3d nuisance_inverse = nuisance_solver.eigenvectors() *
                                               inverse_eigenvalues.asDiagonal() *
                                               nuisance_solver.eigenvectors().transpose();
      planar_information -=
        planar_nuisance_information * nuisance_inverse * planar_nuisance_information.transpose();
    }

    Eigen::Matrix3d unit_scale = Eigen::Matrix3d::Identity();
    unit_scale(0, 0) = 1.0 / result.planar_yaw_scale;
    planar_information = unit_scale.transpose() * planar_information * unit_scale;
    planar_information = 0.5 * (planar_information + planar_information.transpose()).eval();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(planar_information);
    if (eigen_solver.info() == Eigen::Success && eigen_solver.eigenvalues().allFinite()) {
      const double min_eigenvalue = eigen_solver.eigenvalues().minCoeff();
      const double max_eigenvalue = eigen_solver.eigenvalues().maxCoeff();
      if (min_eigenvalue >= 0.0 && max_eigenvalue > 0.0) {
        result.planar_min_eigenvalue = min_eigenvalue;
        result.planar_eigen_ratio = min_eigenvalue / max_eigenvalue;
      }
    }
  }

  PointCloud::Ptr transformed_source(new PointCloud());
  pcl::transformPointCloud(*source_cropped, *transformed_source, result.final_transformation);
  if (!transformed_source || transformed_source->empty()) {
    return;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(local_map_cloud_);

  const float max_dist_sq = static_cast<float>(max_correspondence_distance * max_correspondence_distance);
  std::vector<int> nearest_indices(1);
  std::vector<float> nearest_dist_sq(1);
  std::size_t overlap_count = 0;
  for (const auto & point : transformed_source->points) {
    if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_dist_sq) > 0 &&
        nearest_dist_sq[0] < max_dist_sq) {
      ++overlap_count;
    }
  }

  result.overlap_ratio =
    static_cast<double>(overlap_count) / static_cast<double>(transformed_source->size());
}

GicpFilter::PreparedSource GicpFilter::prepareSource(const PointCloud::Ptr & source_cloud) const
{
  const auto start_time = std::chrono::high_resolution_clock::now();
  PreparedSource prepared;
  PointCloud::Ptr source_filtered_height = applyHeightFilter(source_cloud);
  prepared.cropped_cloud = cropSourceCloud(source_filtered_height);

  if (!prepared.cropped_cloud || prepared.cropped_cloud->empty()) {
    prepared.preprocessing_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time)
        .count();
    return prepared;
  }

  constexpr int kCovNeighbors = 20;
  constexpr int kCovThreads = 8;
  prepared.registration_cloud =
    buildSourceSmallGicpCloud(prepared.cropped_cloud, options_.source_voxel_leaf_size);
  if (!prepared.registration_cloud || prepared.registration_cloud->empty()) {
    prepared.preprocessing_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time)
        .count();
    return prepared;
  }

  small_gicp::estimate_covariances_omp(*prepared.registration_cloud, kCovNeighbors, kCovThreads);
  prepared.preprocessing_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start_time)
      .count();
  return prepared;
}

GicpFilter::Result GicpFilter::initialAlign(
  const PointCloud::Ptr & source_cloud, const double min_inlier_ratio)
{
  return initialAlignPrepared(prepareSource(source_cloud), min_inlier_ratio);
}

GicpFilter::Result GicpFilter::initialAlignPrepared(
  const PreparedSource & source, const double min_inlier_ratio)
{
  if (!source.cropped_cloud || source.cropped_cloud->empty() || !source.registration_cloud ||
      source.registration_cloud->empty()) {
    Result result;
    result.converged = false;
    result.final_transformation = Eigen::Matrix4f::Identity();
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    return result;
  }

  constexpr int kCovThreads = 8;

  if (!small_gicp_target_ || small_gicp_target_->empty() || !target_tree_) {
    Result result;
    result.converged = false;
    result.final_transformation = Eigen::Matrix4f::Identity();
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    return result;
  }

  if (options_.step_x <= 0.0 || options_.step_y <= 0.0 || options_.step_yaw <= 0.0) {
    Result result;
    result.converged = false;
    result.final_transformation = Eigen::Matrix4f::Identity();
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    return result;
  }

  if (options_.search_areas.empty() || options_.z_candidates.empty()) {
    Result result;
    result.converged = false;
    result.final_transformation = Eigen::Matrix4f::Identity();
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    return result;
  }

  double best_error = std::numeric_limits<double>::infinity();
  double best_normalized_score = std::numeric_limits<double>::infinity();
  std::size_t best_num_inliers = 0;
  Eigen::Matrix4f best_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix<double, 6, 6> best_information = Eigen::Matrix<double, 6, 6>::Zero();
  bool best_converged = false;

  SmallGicpRegister register_engine;
  register_engine.reduction.num_threads = kCovThreads;
  register_engine.rejector.max_dist_sq =
    options_.max_correspondence_distance * options_.max_correspondence_distance;
  register_engine.optimizer.max_iterations = options_.max_iterations;
  const auto registration_start = std::chrono::high_resolution_clock::now();

  for (double z : options_.z_candidates) {
    for (const auto & area : options_.search_areas) {
      for (double x = area.min_x; x <= area.max_x; x += options_.step_x) {
        for (double y = area.min_y; y <= area.max_y; y += options_.step_y) {
          for (double yaw = -M_PI; yaw <= M_PI; yaw += options_.step_yaw) {
            Eigen::Isometry3d init_guess_d = Eigen::Isometry3d::Identity();
            init_guess_d.translation() << x, y, z;
            init_guess_d.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            const small_gicp::RegistrationResult reg_result = register_engine.align(
              *small_gicp_target_, *source.registration_cloud, *target_tree_, init_guess_d);

            if (!reg_result.converged) {
              continue;
            }

            if (!std::isfinite(reg_result.error) || reg_result.num_inliers == 0) {
              continue;
            }

            const double inlier_ratio = static_cast<double>(reg_result.num_inliers) /
                                        static_cast<double>(source.registration_cloud->size());
            const double normalized_score = reg_result.error / static_cast<double>(reg_result.num_inliers);
            if (inlier_ratio >= min_inlier_ratio && normalized_score < best_normalized_score) {
              best_error = reg_result.error;
              best_normalized_score = normalized_score;
              best_num_inliers = reg_result.num_inliers;
              best_pose = reg_result.T_target_source.matrix().cast<float>();
              best_information = reg_result.H;
              best_converged = true;
            }
          }
        }
      }
    }
  }

  Result result;
  result.converged = best_converged;
  result.score = best_error;
  result.num_inliers = best_num_inliers;
  result.information = best_information;
  result.final_transformation = best_pose;
  result.source_preprocess_time_ms = source.preprocessing_time_ms;
  result.registration_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - registration_start)
                                  .count();
  result.coarse_registration_time_ms = result.registration_time_ms;
  const auto quality_start = std::chrono::high_resolution_clock::now();
  updateResultQuality(
    result, source.cropped_cloud, source.registration_cloud->size(), options_.max_correspondence_distance);
  result.quality_evaluation_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - quality_start)
      .count();
  return result;
}

GicpFilter::Result GicpFilter::align(const PointCloud::Ptr & source_cloud,
  const Eigen::Matrix4f & initial_guess,
  const double max_correspondence_distance)
{
  return alignPrepared(prepareSource(source_cloud), initial_guess, max_correspondence_distance, true);
}

GicpFilter::Result GicpFilter::alignPrepared(const PreparedSource & source,
  const Eigen::Matrix4f & initial_guess,
  const double max_correspondence_distance,
  const bool evaluate_quality)
{
  if (!source.cropped_cloud || source.cropped_cloud->empty() || !source.registration_cloud ||
      source.registration_cloud->empty() || !global_map_ || global_map_->empty()) {
    Result result;
    result.converged = false;
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    return result;
  }

  constexpr int kCovThreads = 8;

  // 在精配准前根据当前先验位姿更新局部地图
  const auto local_map_start = std::chrono::high_resolution_clock::now();
  updateLocalMap(initial_guess);
  const double local_map_update_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - local_map_start)
      .count();
  if (!small_gicp_target_ || small_gicp_target_->empty() || !target_tree_) {
    Result result;
    result.converged = false;
    result.source_preprocess_time_ms = source.preprocessing_time_ms;
    result.local_map_update_time_ms = local_map_update_time_ms;
    return result;
  }

  SmallGicpRegister register_engine;
  register_engine.reduction.num_threads = kCovThreads;
  const double effective_max_correspondence_distance =
    std::isfinite(max_correspondence_distance) && max_correspondence_distance > 0.0
      ? max_correspondence_distance
      : options_.max_correspondence_distance;
  register_engine.rejector.max_dist_sq =
    effective_max_correspondence_distance * effective_max_correspondence_distance;
  register_engine.optimizer.max_iterations = options_.max_iterations;

  Eigen::Isometry3d init_guess_d = Eigen::Isometry3d::Identity();
  init_guess_d.matrix() = initial_guess.cast<double>();

  const auto registration_start = std::chrono::high_resolution_clock::now();
  const small_gicp::RegistrationResult small_gicp_result =
    register_engine.align(*small_gicp_target_, *source.registration_cloud, *target_tree_, init_guess_d);
  const double registration_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now() - registration_start)
                                        .count();

  // 整理结果
  Result result;
  result.converged = small_gicp_result.converged;
  result.score = small_gicp_result.error;
  result.num_inliers = small_gicp_result.num_inliers;
  result.information = small_gicp_result.H;
  result.final_transformation = small_gicp_result.T_target_source.matrix().cast<float>();
  result.source_preprocess_time_ms = source.preprocessing_time_ms;
  result.local_map_update_time_ms = local_map_update_time_ms;
  result.registration_time_ms = registration_time_ms;
  result.source_points = source.registration_cloud->size();
  if (result.num_inliers > 0 && std::isfinite(result.score)) {
    result.normalized_score = result.score / static_cast<double>(result.num_inliers);
  }
  if (result.source_points > 0) {
    result.inlier_ratio =
      static_cast<double>(result.num_inliers) / static_cast<double>(result.source_points);
  }
  if (evaluate_quality) {
    const auto quality_start = std::chrono::high_resolution_clock::now();
    updateResultQuality(result,
      source.cropped_cloud,
      source.registration_cloud->size(),
      effective_max_correspondence_distance);
    result.quality_evaluation_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - quality_start)
        .count();
  }

  return result;
}

}  // namespace icp_relocalization
