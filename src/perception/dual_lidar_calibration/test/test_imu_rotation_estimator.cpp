#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

#include "dual_lidar_calibration/bag_reader.hpp"
#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

namespace dual_lidar_calibration {
namespace {

TEST(ImuRotationEstimator, RecoversKnownRotationFromExcitedAngularVelocities)
{
  const Eigen::Matrix3d expected =
    (Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();

  const std::vector<Eigen::Vector3d> secondary{
    {0.8, 0.1, 0.2},
    {-0.2, 0.9, 0.1},
    {0.1, -0.3, 1.0},
    {0.6, 0.5, -0.4},
    {-0.5, 0.4, 0.7},
    {0.3, -0.8, 0.6},
  };
  std::vector<AngularVelocityPair> samples;
  samples.reserve(secondary.size());
  for (const auto & omega_secondary : secondary) {
    samples.push_back(AngularVelocityPair{expected * omega_secondary, omega_secondary});
  }

  const ImuRotationResult result = estimateImuRotation(samples, 0.05);

  ASSERT_TRUE(result.observable);
  EXPECT_LT(rotationErrorRad(result.secondary_to_main, expected), 1.0e-9);
  EXPECT_LT(result.rms_residual, 1.0e-9);
}

TEST(ImuRotationEstimator, UsesGravityToCorrectPlaneWhileKeepingInitialYaw)
{
  const double yaw = 0.20;
  const Eigen::Matrix3d initial =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d expected =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.35, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  const Eigen::Vector3d secondary_gravity = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d main_gravity = expected * secondary_gravity;

  const Eigen::Matrix3d result =
    planeConstrainedRotation(initial, main_gravity, secondary_gravity);

  EXPECT_LT(rotationErrorRad(result, expected), 1.0e-9);
  EXPECT_LT((result * secondary_gravity - main_gravity).norm(), 1.0e-9);
}

TEST(LivoxImuConversion, ConvertsAccelerationFromGToMetersPerSecondSquared)
{
  const Eigen::Vector3d converted = livoxAccelerationToSi(Eigen::Vector3d::UnitZ());

  EXPECT_NEAR(converted.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(converted.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(converted.z(), 9.80665, 1.0e-12);
}

}  // namespace
}  // namespace dual_lidar_calibration
