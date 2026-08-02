#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

#include "dual_lidar_calibration/frame_calibrator.hpp"
#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

namespace dual_lidar_calibration {
namespace {

TEST(FrameCalibrator, RecoversKnownTransformFromThreePlanes)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(0.10, 0.20, 0.05);

  std::vector<Eigen::Vector3d> main_points;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      const double a = 0.08 * static_cast<double>(i);
      const double b = 0.08 * static_cast<double>(j);
      main_points.emplace_back(a, b, 0.0);
      main_points.emplace_back(0.0, a, b);
      main_points.emplace_back(a, 1.5, b);
    }
  }
  std::vector<Eigen::Vector3d> secondary_points;
  secondary_points.reserve(main_points.size());
  for (const auto & point : main_points) {
    secondary_points.push_back(expected.inverse() * point);
  }

  Eigen::Isometry3d initial = expected;
  initial.translation() += Eigen::Vector3d(0.03, -0.02, 0.01);
  CalibrationConfig config;
  config.voxel_size = 0.05;
  config.gicp.max_correspondence_distance = 0.30;
  config.gicp.max_iterations = 80;
  config.gicp.min_overlap_ratio = 0.80;
  config.gicp.min_inliers = 300U;
  config.gicp.max_rmse = 0.05;
  config.gicp.max_rotation_deviation_rad = 0.10;
  config.gicp.max_translation_deviation = 0.20;

  const FrameCalibrationResult result = calibrateFrame(
    0U, 1'000'000'000LL, 1'000'000'000LL, main_points, secondary_points,
    initial, expected.rotation(), config);

  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_LT((result.secondary_to_main.translation() - expected.translation()).norm(), 0.02);
  EXPECT_LT(rotationErrorRad(result.secondary_to_main.rotation(), expected.rotation()), 0.02);
}

}  // namespace
}  // namespace dual_lidar_calibration
