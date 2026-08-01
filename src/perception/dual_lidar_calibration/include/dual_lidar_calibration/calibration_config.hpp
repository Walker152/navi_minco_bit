#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <string>

namespace dual_lidar_calibration {

struct GicpConfig
{
  double max_correspondence_distance{0.5};
  int max_iterations{64};
  double transformation_epsilon{1.0e-6};
  double min_overlap_ratio{0.30};
  std::size_t min_inliers{500U};
  double max_rmse{0.15};
  double max_rotation_deviation_rad{0.20};
  double max_translation_deviation{0.50};
};

struct CalibrationConfig
{
  std::string main_lidar_topic;
  std::string main_imu_topic;
  std::string secondary_lidar_topic;
  std::string secondary_imu_topic;

  Eigen::Isometry3d initial_secondary_to_main{Eigen::Isometry3d::Identity()};
  Eigen::Matrix3d main_lidar_to_imu_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d secondary_lidar_to_imu_rotation{Eigen::Matrix3d::Identity()};

  std::int64_t sync_tolerance_ns{5'000'000LL};
  std::size_t frame_stride{5U};
  std::size_t max_frame_pairs{200U};
  std::size_t minimum_accepted_frames{8U};
  std::size_t point_stride{1U};
  double min_range{0.5};
  double max_range{20.0};
  double voxel_size{0.10};
  double min_angular_speed{0.05};
  double stationary_gyro_threshold{0.03};
  double gravity_tolerance{1.5};
  double translation_outlier_threshold{0.15};
  double rotation_outlier_threshold_rad{0.15};
  double max_translation_stddev{0.05};
  double max_rotation_stddev_rad{0.03};
  GicpConfig gicp;
};

CalibrationConfig loadCalibrationConfig(const std::string & yaml_path);

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw);
Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & rotation);

}  // namespace dual_lidar_calibration
