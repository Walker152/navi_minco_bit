#pragma once

#include "dual_lidar_calibration/calibration_config.hpp"
#include "dual_lidar_calibration/calibration_types.hpp"

#include <vector>

namespace dual_lidar_calibration {

struct BagData
{
  std::vector<TimedCloud> main_clouds;
  std::vector<TimedCloud> secondary_clouds;
  std::vector<ImuSample> main_imu;
  std::vector<ImuSample> secondary_imu;
};

Eigen::Vector3d livoxAccelerationToSi(const Eigen::Vector3d & acceleration_g);

BagData readCalibrationBag(
  const std::string & bag_path, const CalibrationConfig & config, bool include_lidar = true);

}  // namespace dual_lidar_calibration
