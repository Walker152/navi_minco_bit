#include "dual_lidar_calibration/deskewer.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace dual_lidar_calibration {

bool imuCoversInterval(
  const std::vector<ImuSample> & samples, const std::int64_t begin_ns, const std::int64_t end_ns)
{
  return !samples.empty() && samples.front().stamp_ns <= begin_ns && samples.back().stamp_ns >= end_ns;
}

Eigen::Vector3d interpolateAngularVelocity(
  const std::vector<ImuSample> & samples, const std::int64_t stamp_ns)
{
  if (samples.empty()) {
    return Eigen::Vector3d::Zero();
  }
  const auto upper = std::lower_bound(samples.begin(), samples.end(), stamp_ns,
    [](const ImuSample & sample, const std::int64_t stamp) { return sample.stamp_ns < stamp; });
  if (upper == samples.begin()) {
    return upper->angular_velocity;
  }
  if (upper == samples.end()) {
    return samples.back().angular_velocity;
  }
  const auto lower = std::prev(upper);
  const double span = static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  if (span <= 0.0) {
    return lower->angular_velocity;
  }
  const double ratio = static_cast<double>(stamp_ns - lower->stamp_ns) / span;
  return (1.0 - ratio) * lower->angular_velocity + ratio * upper->angular_velocity;
}

Eigen::Vector3d deskewPointToReference(const TimedPoint & point,
  const std::int64_t reference_stamp_ns,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation)
{
  const std::int64_t midpoint_ns = reference_stamp_ns + (point.stamp_ns - reference_stamp_ns) / 2;
  const Eigen::Vector3d omega_imu = interpolateAngularVelocity(imu_samples, midpoint_ns) - gyro_bias;
  const Eigen::Vector3d omega_lidar = lidar_to_imu_rotation.transpose() * omega_imu;
  const double dt = static_cast<double>(point.stamp_ns - reference_stamp_ns) * 1.0e-9;
  const Eigen::Vector3d rotation_vector = omega_lidar * dt;
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-12) {
    return point.position;
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle) * point.position;
}

std::vector<Eigen::Vector3d> deskewCloud(const TimedCloud & cloud,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation)
{
  return deskewCloudToReference(
    cloud, cloud.reference_stamp_ns, imu_samples, gyro_bias, lidar_to_imu_rotation);
}

std::vector<Eigen::Vector3d> deskewCloudToReference(const TimedCloud & cloud,
  const std::int64_t reference_stamp_ns,
  const std::vector<ImuSample> & imu_samples,
  const Eigen::Vector3d & gyro_bias,
  const Eigen::Matrix3d & lidar_to_imu_rotation)
{
  std::vector<Eigen::Vector3d> corrected;
  corrected.reserve(cloud.points.size());
  for (const auto & point : cloud.points) {
    corrected.push_back(deskewPointToReference(
      point, reference_stamp_ns, imu_samples, gyro_bias, lidar_to_imu_rotation));
  }
  return corrected;
}

}  // namespace dual_lidar_calibration
