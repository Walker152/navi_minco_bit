#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

#include "dual_lidar_calibration/extrinsic_aggregator.hpp"

namespace dual_lidar_calibration {
namespace {

TEST(ExtrinsicAggregator, RejectsOneLargeOutlier)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitX()).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(0.0, 0.4, 0.05);

  std::vector<FrameCalibrationResult> frames;
  for (int i = 0; i < 8; ++i) {
    FrameCalibrationResult frame;
    frame.accepted = true;
    frame.secondary_to_main = expected;
    frame.secondary_to_main.translation().x() += 0.001 * static_cast<double>(i - 4);
    frame.rmse = 0.03;
    frame.overlap_ratio = 0.7;
    frame.inliers = 2000U;
    frames.push_back(frame);
  }
  FrameCalibrationResult outlier;
  outlier.accepted = true;
  outlier.secondary_to_main = Eigen::Isometry3d::Identity();
  outlier.secondary_to_main.translation() = Eigen::Vector3d(2.0, -1.0, 0.8);
  outlier.rmse = 0.03;
  outlier.overlap_ratio = 0.7;
  outlier.inliers = 2000U;
  frames.push_back(outlier);

  const AggregateResult result = aggregateExtrinsics(frames, expected.rotation(), 0.15, 0.15);

  ASSERT_TRUE(result.success);
  EXPECT_LT((result.secondary_to_main.translation() - expected.translation()).norm(), 0.01);
  EXPECT_LT(rotationErrorRad(result.secondary_to_main.rotation(), expected.rotation()), 1.0e-6);
  EXPECT_EQ(result.used_frames, 8U);
}

}  // namespace
}  // namespace dual_lidar_calibration
