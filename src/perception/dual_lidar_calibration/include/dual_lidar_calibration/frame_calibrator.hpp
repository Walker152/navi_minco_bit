#pragma once

#include "dual_lidar_calibration/calibration_config.hpp"
#include "dual_lidar_calibration/calibration_types.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace dual_lidar_calibration {

FrameCalibrationResult calibrateFrame(std::size_t pair_index,
  std::int64_t main_stamp_ns,
  std::int64_t secondary_stamp_ns,
  const std::vector<Eigen::Vector3d> & main_points,
  const std::vector<Eigen::Vector3d> & secondary_points,
  const Eigen::Isometry3d & initial_secondary_to_main,
  const Eigen::Matrix3d & imu_rotation,
  const CalibrationConfig & config);

}  // namespace dual_lidar_calibration
