#pragma once

#include "dual_lidar_calibration/calibration_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace dual_lidar_calibration {

bool imuCoversInterval(
  const std::vector<ImuSample> & samples, std::int64_t begin_ns, std::int64_t end_ns);

Eigen::Vector3d interpolateAngularVelocity(
  const std::vector<ImuSample> & samples, std::int64_t stamp_ns);

Eigen::Vector3d deskewPointToReference(const TimedPoint & point,
  std::int64_t reference_stamp_ns,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation);

std::vector<Eigen::Vector3d> deskewCloud(const TimedCloud & cloud,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation);

std::vector<Eigen::Vector3d> deskewCloudToReference(const TimedCloud & cloud,
  std::int64_t reference_stamp_ns,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation);

}  // namespace dual_lidar_calibration
