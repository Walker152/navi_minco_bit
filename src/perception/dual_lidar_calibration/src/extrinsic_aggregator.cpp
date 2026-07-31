#include "dual_lidar_calibration/extrinsic_aggregator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dual_lidar_calibration {

namespace {

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double value = values[middle];
  if (values.size() % 2U == 0U) {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    value = 0.5 * (value + *lower);
  }
  return value;
}

Eigen::Vector3d medianTranslation(const std::vector<const FrameCalibrationResult *> & frames)
{
  Eigen::Vector3d result;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    values.reserve(frames.size());
    for (const auto * frame : frames) {
      values.push_back(frame->secondary_to_main.translation()[axis]);
    }
    result[axis] = median(std::move(values));
  }
  return result;
}

Eigen::Quaterniond weightedQuaternionMean(const std::vector<const FrameCalibrationResult *> & frames,
  const std::vector<double> & weights,
  const Eigen::Matrix3d & reference_rotation)
{
  Eigen::Quaterniond reference(reference_rotation);
  Eigen::Vector4d sum = Eigen::Vector4d::Zero();
  for (std::size_t i = 0; i < frames.size(); ++i) {
    Eigen::Quaterniond quaternion(frames[i]->secondary_to_main.rotation());
    if (reference.coeffs().dot(quaternion.coeffs()) < 0.0) {
      quaternion.coeffs() *= -1.0;
    }
    sum += weights[i] * quaternion.coeffs();
  }
  if (sum.norm() < 1.0e-12) {
    return reference;
  }
  Eigen::Quaterniond result;
  result.coeffs() = sum.normalized();
  return result;
}

}  // namespace

AggregateResult aggregateExtrinsics(const std::vector<FrameCalibrationResult> & frames,
  const Eigen::Matrix3d & imu_rotation,
  const double translation_outlier_threshold,
  const double rotation_outlier_threshold_rad)
{
  AggregateResult result;
  std::vector<const FrameCalibrationResult *> candidates;
  for (const auto & frame : frames) {
    if (frame.accepted && frame.secondary_to_main.matrix().allFinite() && std::isfinite(frame.rmse)) {
      candidates.push_back(&frame);
    }
  }
  if (candidates.size() < 3U) {
    return result;
  }

  const Eigen::Vector3d translation_center = medianTranslation(candidates);
  std::vector<const FrameCalibrationResult *> inliers;
  inliers.reserve(candidates.size());
  for (const auto * frame : candidates) {
    const double translation_error =
      (frame->secondary_to_main.translation() - translation_center).norm();
    const double rotation_error = rotationErrorRad(frame->secondary_to_main.rotation(), imu_rotation);
    if (translation_error <= translation_outlier_threshold &&
        rotation_error <= rotation_outlier_threshold_rad) {
      inliers.push_back(frame);
    }
  }
  if (inliers.size() < 3U) {
    return result;
  }

  std::vector<double> weights;
  weights.reserve(inliers.size());
  double weight_sum = 0.0;
  Eigen::Vector3d weighted_translation = Eigen::Vector3d::Zero();
  for (const auto * frame : inliers) {
    const double weight = std::max(1.0e-6, frame->overlap_ratio) /
                          std::max(1.0e-3, frame->rmse);
    weights.push_back(weight);
    weight_sum += weight;
    weighted_translation += weight * frame->secondary_to_main.translation();
  }
  weighted_translation /= weight_sum;
  const Eigen::Quaterniond rotation = weightedQuaternionMean(inliers, weights, imu_rotation);

  double translation_variance = 0.0;
  double rotation_variance = 0.0;
  for (std::size_t i = 0; i < inliers.size(); ++i) {
    translation_variance += weights[i] *
                            (inliers[i]->secondary_to_main.translation() - weighted_translation)
                              .squaredNorm();
    const double rotation_error =
      rotationErrorRad(inliers[i]->secondary_to_main.rotation(), rotation.toRotationMatrix());
    rotation_variance += weights[i] * rotation_error * rotation_error;
  }

  result.success = true;
  result.used_frames = inliers.size();
  result.secondary_to_main.linear() = rotation.toRotationMatrix();
  result.secondary_to_main.translation() = weighted_translation;
  result.translation_stddev = std::sqrt(translation_variance / weight_sum);
  result.rotation_stddev_rad = std::sqrt(rotation_variance / weight_sum);
  return result;
}

}  // namespace dual_lidar_calibration
