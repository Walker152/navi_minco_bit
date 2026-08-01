#include "dual_lidar_calibration/frame_calibrator.hpp"

#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace dual_lidar_calibration {

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud::Ptr toFilteredCloud(const std::vector<Eigen::Vector3d> & points, const double voxel_size)
{
  Cloud::Ptr input(new Cloud());
  input->reserve(points.size());
  for (const auto & point : points) {
    if (point.allFinite()) {
      input->push_back(pcl::PointXYZ(
        static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())));
    }
  }
  input->width = static_cast<std::uint32_t>(input->size());
  input->height = 1U;
  input->is_dense = false;

  Cloud::Ptr filtered(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(static_cast<float>(voxel_size),
    static_cast<float>(voxel_size),
    static_cast<float>(voxel_size));
  voxel.setInputCloud(input);
  voxel.filter(*filtered);
  return filtered;
}

void setRejected(FrameCalibrationResult & result, const std::string & reason)
{
  result.accepted = false;
  result.rejection_reason = reason;
}

}  // namespace

FrameCalibrationResult calibrateFrame(const std::size_t pair_index,
  const std::int64_t main_stamp_ns,
  const std::int64_t secondary_stamp_ns,
  const std::vector<Eigen::Vector3d> & main_points,
  const std::vector<Eigen::Vector3d> & secondary_points,
  const Eigen::Isometry3d & initial_secondary_to_main,
  const Eigen::Matrix3d & imu_rotation,
  const CalibrationConfig & config)
{
  FrameCalibrationResult result;
  result.pair_index = pair_index;
  result.main_stamp_ns = main_stamp_ns;
  result.secondary_stamp_ns = secondary_stamp_ns;
  result.secondary_to_main = initial_secondary_to_main;

  const Cloud::Ptr main_cloud = toFilteredCloud(main_points, config.voxel_size);
  const Cloud::Ptr secondary_cloud = toFilteredCloud(secondary_points, config.voxel_size);
  if (main_cloud->size() < config.gicp.min_inliers ||
      secondary_cloud->size() < config.gicp.min_inliers) {
    setRejected(result, "too_few_downsampled_points");
    return result;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputTarget(main_cloud);
  gicp.setInputSource(secondary_cloud);
  gicp.setMaxCorrespondenceDistance(config.gicp.max_correspondence_distance);
  gicp.setMaximumIterations(config.gicp.max_iterations);
  gicp.setTransformationEpsilon(config.gicp.transformation_epsilon);
  Cloud aligned;
  gicp.align(aligned, initial_secondary_to_main.matrix().cast<float>());
  result.converged = gicp.hasConverged();
  if (!result.converged) {
    setRejected(result, "gicp_not_converged");
    return result;
  }

  result.secondary_to_main.matrix() = gicp.getFinalTransformation().cast<double>();
  if (!result.secondary_to_main.matrix().allFinite() ||
      std::abs(result.secondary_to_main.rotation().determinant() - 1.0) > 1.0e-3) {
    setRejected(result, "non_rigid_or_nonfinite_transform");
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(main_cloud);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  const double maximum_squared_distance = config.gicp.max_correspondence_distance *
                                          config.gicp.max_correspondence_distance;
  double squared_error_sum = 0.0;
  for (const auto & source_point : secondary_cloud->points) {
    const Eigen::Vector3d source(source_point.x, source_point.y, source_point.z);
    const Eigen::Vector3d transformed = result.secondary_to_main * source;
    const pcl::PointXYZ query(static_cast<float>(transformed.x()),
      static_cast<float>(transformed.y()),
      static_cast<float>(transformed.z()));
    if (tree.nearestKSearch(query, 1, nearest_index, nearest_squared_distance) > 0 &&
        nearest_squared_distance[0] <= maximum_squared_distance) {
      ++result.inliers;
      squared_error_sum += nearest_squared_distance[0];
    }
  }
  result.overlap_ratio = secondary_cloud->empty() ? 0.0 :
                           static_cast<double>(result.inliers) /
                             static_cast<double>(secondary_cloud->size());
  result.rmse = result.inliers > 0U ?
                  std::sqrt(squared_error_sum / static_cast<double>(result.inliers)) :
                  std::numeric_limits<double>::infinity();

  if (result.inliers < config.gicp.min_inliers) {
    setRejected(result, "insufficient_inliers");
  } else if (result.overlap_ratio < config.gicp.min_overlap_ratio) {
    setRejected(result, "insufficient_overlap");
  } else if (result.rmse > config.gicp.max_rmse) {
    setRejected(result, "rmse_too_large");
  } else if (rotationErrorRad(result.secondary_to_main.rotation(), imu_rotation) >
             config.gicp.max_rotation_deviation_rad) {
    setRejected(result, "rotation_disagrees_with_imu");
  } else if ((result.secondary_to_main.translation() - initial_secondary_to_main.translation()).norm() >
             config.gicp.max_translation_deviation) {
    setRejected(result, "translation_too_far_from_initial_guess");
  } else {
    result.accepted = true;
    result.rejection_reason.clear();
  }
  return result;
}

}  // namespace dual_lidar_calibration
