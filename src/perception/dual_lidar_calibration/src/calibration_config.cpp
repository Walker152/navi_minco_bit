#include "dual_lidar_calibration/calibration_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace dual_lidar_calibration {

namespace {

template<typename T>
T optionalValue(const YAML::Node & root, const char * key, const T & default_value)
{
  return root && root[key] ? root[key].as<T>() : default_value;
}

std::string requiredString(const YAML::Node & root, const char * key)
{
  if (!root[key]) {
    throw std::invalid_argument(std::string("Missing required calibration key: ") + key);
  }
  const std::string value = root[key].as<std::string>();
  if (value.empty()) {
    throw std::invalid_argument(std::string("Calibration key must not be empty: ") + key);
  }
  return value;
}

std::vector<double> fixedVector(
  const YAML::Node & root, const char * key, const std::vector<double> & default_value, std::size_t size)
{
  if (!root[key]) {
    return default_value;
  }
  const auto value = root[key].as<std::vector<double>>();
  if (value.size() != size) {
    throw std::invalid_argument(std::string("Calibration key has wrong vector size: ") + key);
  }
  for (const double scalar : value) {
    if (!std::isfinite(scalar)) {
      throw std::invalid_argument(std::string("Calibration key contains non-finite value: ") + key);
    }
  }
  return value;
}

void requirePositive(const double value, const char * key)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("Calibration key must be positive: ") + key);
  }
}

}  // namespace

Eigen::Matrix3d rotationFromRpy(const double roll, const double pitch, const double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & rotation)
{
  const Eigen::Vector3d yaw_pitch_roll = rotation.eulerAngles(2, 1, 0);
  return Eigen::Vector3d(yaw_pitch_roll.z(), yaw_pitch_roll.y(), yaw_pitch_roll.x());
}

CalibrationConfig loadCalibrationConfig(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  CalibrationConfig config;
  config.main_lidar_topic = requiredString(root, "main_lidar_topic");
  config.main_imu_topic = requiredString(root, "main_imu_topic");
  config.secondary_lidar_topic = requiredString(root, "secondary_lidar_topic");
  config.secondary_imu_topic = requiredString(root, "secondary_imu_topic");

  const auto initial = fixedVector(
    root, "initial_extrinsic", {0.0, 0.4, 0.0, -0.35453, 0.0, 0.0}, 6U);
  config.initial_secondary_to_main.linear() =
    rotationFromRpy(initial[3], initial[4], initial[5]);
  config.initial_secondary_to_main.translation() =
    Eigen::Vector3d(initial[0], initial[1], initial[2]);

  const auto main_lidar_to_imu =
    fixedVector(root, "main_lidar_to_imu_rpy", {0.0, 0.0, 0.0}, 3U);
  const auto secondary_lidar_to_imu =
    fixedVector(root, "secondary_lidar_to_imu_rpy", {0.0, 0.0, 0.0}, 3U);
  config.main_lidar_to_imu_rotation =
    rotationFromRpy(main_lidar_to_imu[0], main_lidar_to_imu[1], main_lidar_to_imu[2]);
  config.secondary_lidar_to_imu_rotation = rotationFromRpy(
    secondary_lidar_to_imu[0], secondary_lidar_to_imu[1], secondary_lidar_to_imu[2]);

  const double sync_tolerance_ms = optionalValue(root, "sync_tolerance_ms", 5.0);
  config.sync_tolerance_ns = static_cast<std::int64_t>(sync_tolerance_ms * 1.0e6);
  config.frame_stride = optionalValue<std::size_t>(root, "frame_stride", 5U);
  config.max_frame_pairs = optionalValue<std::size_t>(root, "max_frame_pairs", 200U);
  config.minimum_accepted_frames =
    optionalValue<std::size_t>(root, "minimum_accepted_frames", 8U);
  config.point_stride = optionalValue<std::size_t>(root, "point_stride", 1U);
  config.min_range = optionalValue(root, "min_range", 0.5);
  config.max_range = optionalValue(root, "max_range", 20.0);
  config.voxel_size = optionalValue(root, "voxel_size", 0.10);
  config.min_angular_speed = optionalValue(root, "min_angular_speed", 0.05);
  config.stationary_gyro_threshold = optionalValue(root, "stationary_gyro_threshold", 0.03);
  config.gravity_tolerance = optionalValue(root, "gravity_tolerance", 1.5);
  config.translation_outlier_threshold =
    optionalValue(root, "translation_outlier_threshold", 0.15);
  config.rotation_outlier_threshold_rad =
    optionalValue(root, "rotation_outlier_threshold_rad", 0.15);
  config.max_translation_stddev = optionalValue(root, "max_translation_stddev", 0.05);
  config.max_rotation_stddev_rad = optionalValue(root, "max_rotation_stddev_rad", 0.03);

  const YAML::Node gicp = root["gicp"];
  config.gicp.max_correspondence_distance =
    optionalValue(gicp, "max_correspondence_distance", 0.50);
  config.gicp.max_iterations = optionalValue(gicp, "max_iterations", 64);
  config.gicp.transformation_epsilon =
    optionalValue(gicp, "transformation_epsilon", 1.0e-6);
  config.gicp.min_overlap_ratio = optionalValue(gicp, "min_overlap_ratio", 0.30);
  config.gicp.min_inliers = optionalValue<std::size_t>(gicp, "min_inliers", 500U);
  config.gicp.max_rmse = optionalValue(gicp, "max_rmse", 0.15);
  config.gicp.max_rotation_deviation_rad =
    optionalValue(gicp, "max_rotation_deviation_rad", 0.20);
  config.gicp.max_translation_deviation =
    optionalValue(gicp, "max_translation_deviation", 0.50);

  requirePositive(sync_tolerance_ms, "sync_tolerance_ms");
  requirePositive(config.min_range, "min_range");
  requirePositive(config.max_range, "max_range");
  requirePositive(config.voxel_size, "voxel_size");
  requirePositive(config.gicp.max_correspondence_distance, "gicp.max_correspondence_distance");
  if (config.max_range <= config.min_range || config.frame_stride == 0U ||
      config.max_frame_pairs == 0U || config.minimum_accepted_frames < 3U ||
      config.point_stride == 0U || config.gicp.max_iterations <= 0) {
    throw std::invalid_argument("Invalid calibration range, sampling, or iteration configuration");
  }
  return config;
}

}  // namespace dual_lidar_calibration
