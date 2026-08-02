#pragma once

#include <Eigen/Core>

#include <vector>

namespace dual_lidar_calibration {

struct AngularVelocityPair
{
  Eigen::Vector3d main{Eigen::Vector3d::Zero()};
  Eigen::Vector3d secondary{Eigen::Vector3d::Zero()};
  double weight{1.0};
};

struct ImuRotationResult
{
  bool observable{false};
  Eigen::Matrix3d secondary_to_main{Eigen::Matrix3d::Identity()};
  double rms_residual{0.0};
  std::size_t used_samples{0U};
  Eigen::Vector3d singular_values{Eigen::Vector3d::Zero()};
};

ImuRotationResult estimateImuRotation(
  const std::vector<AngularVelocityPair> & samples, double min_vector_norm);

Eigen::Matrix3d planeConstrainedRotation(const Eigen::Matrix3d & initial_rotation,
  const Eigen::Vector3d & main_gravity,
  const Eigen::Vector3d & secondary_gravity);

double rotationErrorRad(const Eigen::Matrix3d & lhs, const Eigen::Matrix3d & rhs);

}  // namespace dual_lidar_calibration
