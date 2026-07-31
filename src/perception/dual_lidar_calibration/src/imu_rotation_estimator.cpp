#include "dual_lidar_calibration/imu_rotation_estimator.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace dual_lidar_calibration {

namespace {

Eigen::Matrix3d solveWeightedWahba(
  const std::vector<AngularVelocityPair> & samples, const std::vector<double> & robust_weights)
{
  Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double weight = samples[i].weight * robust_weights[i];
    cross_covariance.noalias() += weight * samples[i].main * samples[i].secondary.transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    cross_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  return svd.matrixU() * correction * svd.matrixV().transpose();
}

}  // namespace

ImuRotationResult estimateImuRotation(
  const std::vector<AngularVelocityPair> & input_samples, const double min_vector_norm)
{
  ImuRotationResult result;
  std::vector<AngularVelocityPair> samples;
  samples.reserve(input_samples.size());
  for (const auto & sample : input_samples) {
    if (!sample.main.allFinite() || !sample.secondary.allFinite() || !std::isfinite(sample.weight) ||
        sample.weight <= 0.0 || sample.main.norm() < min_vector_norm ||
        sample.secondary.norm() < min_vector_norm) {
      continue;
    }
    samples.push_back(sample);
  }
  result.used_samples = samples.size();
  if (samples.size() < 3U) {
    return result;
  }

  std::vector<double> robust_weights(samples.size(), 1.0);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  for (int iteration = 0; iteration < 4; ++iteration) {
    rotation = solveWeightedWahba(samples, robust_weights);
    std::vector<double> residuals;
    residuals.reserve(samples.size());
    for (const auto & sample : samples) {
      residuals.push_back((sample.main - rotation * sample.secondary).norm());
    }
    std::vector<double> sorted = residuals;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2U, sorted.end());
    const double median = sorted[sorted.size() / 2U];
    const double huber_delta = std::max(1.0e-6, 2.5 * median);
    for (std::size_t i = 0; i < residuals.size(); ++i) {
      robust_weights[i] = residuals[i] <= huber_delta ? 1.0 : huber_delta / residuals[i];
    }
  }

  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  double weighted_squared_error = 0.0;
  double total_weight = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double weight = samples[i].weight * robust_weights[i];
    information.noalias() += weight * samples[i].secondary * samples[i].secondary.transpose();
    weighted_squared_error +=
      weight * (samples[i].main - rotation * samples[i].secondary).squaredNorm();
    total_weight += weight;
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> information_svd(information);
  result.singular_values = information_svd.singularValues();
  result.observable = result.singular_values.y() > 1.0e-6 &&
                      result.singular_values.y() / std::max(result.singular_values.x(), 1.0e-12) > 0.01;
  result.secondary_to_main = rotation;
  result.rms_residual =
    total_weight > 0.0 ? std::sqrt(weighted_squared_error / total_weight) : 0.0;
  return result;
}

Eigen::Matrix3d planeConstrainedRotation(const Eigen::Matrix3d & initial_rotation,
  const Eigen::Vector3d & main_gravity,
  const Eigen::Vector3d & secondary_gravity)
{
  const Eigen::Vector3d predicted_main = initial_rotation * secondary_gravity.normalized();
  const Eigen::Quaterniond correction =
    Eigen::Quaterniond::FromTwoVectors(predicted_main, main_gravity.normalized());
  return correction.toRotationMatrix() * initial_rotation;
}

double rotationErrorRad(const Eigen::Matrix3d & lhs, const Eigen::Matrix3d & rhs)
{
  const Eigen::Matrix3d delta = lhs.transpose() * rhs;
  const Eigen::Vector3d skew(
    delta(2, 1) - delta(1, 2), delta(0, 2) - delta(2, 0), delta(1, 0) - delta(0, 1));
  const double sine = 0.5 * skew.norm();
  const double cosine = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::atan2(sine, cosine);
}

}  // namespace dual_lidar_calibration
