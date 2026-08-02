#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

#include "dual_lidar_calibration/deskewer.hpp"

namespace dual_lidar_calibration {
namespace {

TEST(LivoxDeskewer, RemovesConstantAngularMotionAtReferenceTime)
{
  constexpr std::int64_t kReferenceNs = 1'000'000'000LL;
  constexpr std::int64_t kPointNs = kReferenceNs + 100'000'000LL;
  const std::vector<ImuSample> imu_samples{
    {kReferenceNs, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()},
    {kPointNs, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero()},
  };
  const Eigen::Vector3d world_point(2.0, 0.0, 0.0);
  const Eigen::Matrix3d point_rotation =
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const TimedPoint point{point_rotation.transpose() * world_point, kPointNs};

  const Eigen::Vector3d corrected = deskewPointToReference(
    point, kReferenceNs, imu_samples, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

  EXPECT_LT((corrected - world_point).norm(), 1.0e-9);
}

}  // namespace
}  // namespace dual_lidar_calibration
