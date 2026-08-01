#pragma once

#include "dual_lidar_calibration/calibration_config.hpp"
#include "dual_lidar_calibration/offline_calibrator.hpp"

#include <string>

namespace dual_lidar_calibration {

void writeCalibrationResults(const std::string & output_directory,
  const CalibrationConfig & config,
  const CalibrationRunResult & result);

}  // namespace dual_lidar_calibration
