#pragma once

#include "dual_lidar_calibration/calibration_types.hpp"
#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace dual_lidar_calibration {

struct AggregateResult
{
  bool success{false};
  Eigen::Isometry3d secondary_to_main{Eigen::Isometry3d::Identity()};
  std::size_t used_frames{0U};
  double translation_stddev{0.0};
  double rotation_stddev_rad{0.0};
};

AggregateResult aggregateExtrinsics(const std::vector<FrameCalibrationResult> & frames,
  const Eigen::Matrix3d & imu_rotation,
  double translation_outlier_threshold,
  double rotation_outlier_threshold_rad);

}  // namespace dual_lidar_calibration
