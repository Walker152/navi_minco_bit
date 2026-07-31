#pragma once

#include "dual_lidar_calibration/calibration_config.hpp"
#include "dual_lidar_calibration/calibration_types.hpp"
#include "dual_lidar_calibration/extrinsic_aggregator.hpp"
#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace dual_lidar_calibration {

struct CalibrationRunResult
{
  bool success{false};
  bool used_static_plane_constraint{false};
  std::string message;
  std::size_t synchronized_pairs{0U};
  std::size_t imu_coverage_rejections{0U};
  Eigen::Vector3d main_gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d secondary_gyro_bias{Eigen::Vector3d::Zero()};
  ImuRotationResult imu_rotation;
  std::vector<FrameCalibrationResult> frame_results;
  AggregateResult aggregate;
  std::vector<Eigen::Vector3d> preview_main;
  std::vector<Eigen::Vector3d> preview_secondary;
};

CalibrationRunResult runOfflineCalibration(
  const std::string & bag_path,
  const CalibrationConfig & config,
  const std::string & static_bag_path = "");

}  // namespace dual_lidar_calibration
