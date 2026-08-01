#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace dual_lidar_calibration {

struct TimedPoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  std::int64_t stamp_ns{0};
};

struct TimedCloud
{
  std::int64_t reference_stamp_ns{0};
  std::int64_t begin_stamp_ns{0};
  std::int64_t end_stamp_ns{0};
  std::vector<TimedPoint> points;
};

struct ImuSample
{
  std::int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
};

struct FrameCalibrationResult
{
  std::size_t pair_index{0U};
  std::int64_t main_stamp_ns{0};
  std::int64_t secondary_stamp_ns{0};
  bool converged{false};
  bool accepted{false};
  std::string rejection_reason;
  double rmse{std::numeric_limits<double>::infinity()};
  double overlap_ratio{0.0};
  std::size_t inliers{0U};
  Eigen::Isometry3d secondary_to_main{Eigen::Isometry3d::Identity()};
};

}  // namespace dual_lidar_calibration
