#include "dual_lidar_calibration/offline_calibrator.hpp"

#include "dual_lidar_calibration/bag_reader.hpp"
#include "dual_lidar_calibration/deskewer.hpp"
#include "dual_lidar_calibration/frame_calibrator.hpp"
#include "dual_lidar_calibration/frame_synchronizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <utility>
#include <vector>

namespace dual_lidar_calibration {

namespace {

Eigen::Vector3d estimateGyroBias(
  const std::vector<ImuSample> & samples, const double stationary_threshold)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0U;
  for (const auto & sample : samples) {
    if (sample.angular_velocity.norm() <= stationary_threshold) {
      sum += sample.angular_velocity;
      ++count;
    }
  }
  if (count < 20U) {
    return Eigen::Vector3d::Zero();
  }
  return sum / static_cast<double>(count);
}

Eigen::Vector3d estimateGravityDirection(
  const std::vector<ImuSample> & samples, const double gravity_tolerance)
{
  constexpr double kGravity = 9.80665;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0U;
  for (const auto & sample : samples) {
    const double norm = sample.linear_acceleration.norm();
    if (std::abs(norm - kGravity) <= gravity_tolerance && norm > 1.0) {
      sum += sample.linear_acceleration / norm;
      ++count;
    }
  }
  if (count < 20U || sum.norm() < 1.0e-6) {
    return Eigen::Vector3d::Zero();
  }
  return sum.normalized();
}

ImuSample interpolateImu(const std::vector<ImuSample> & samples, const std::int64_t stamp_ns)
{
  const auto upper = std::lower_bound(samples.begin(), samples.end(), stamp_ns,
    [](const ImuSample & sample, const std::int64_t stamp) { return sample.stamp_ns < stamp; });
  if (upper == samples.begin()) {
    return *upper;
  }
  if (upper == samples.end()) {
    return samples.back();
  }
  const auto lower = std::prev(upper);
  const double span = static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  const double ratio = span > 0.0 ? static_cast<double>(stamp_ns - lower->stamp_ns) / span : 0.0;
  ImuSample result;
  result.stamp_ns = stamp_ns;
  result.angular_velocity =
    (1.0 - ratio) * lower->angular_velocity + ratio * upper->angular_velocity;
  result.linear_acceleration =
    (1.0 - ratio) * lower->linear_acceleration + ratio * upper->linear_acceleration;
  return result;
}

std::vector<AngularVelocityPair> buildImuVectorPairs(const BagData & data,
  const CalibrationConfig & config,
  const Eigen::Vector3d & main_bias,
  const Eigen::Vector3d & secondary_bias)
{
  std::vector<AngularVelocityPair> pairs;
  if (data.main_imu.empty() || data.secondary_imu.empty()) {
    return pairs;
  }
  const std::int64_t begin_ns =
    std::max(data.main_imu.front().stamp_ns, data.secondary_imu.front().stamp_ns);
  const std::int64_t end_ns =
    std::min(data.main_imu.back().stamp_ns, data.secondary_imu.back().stamp_ns);
  constexpr std::size_t kImuSampleStride = 5U;
  constexpr double kGravity = 9.80665;
  for (std::size_t i = 0; i < data.main_imu.size(); i += kImuSampleStride) {
    const ImuSample & main_sample = data.main_imu[i];
    if (main_sample.stamp_ns < begin_ns || main_sample.stamp_ns > end_ns) {
      continue;
    }
    const ImuSample secondary_sample = interpolateImu(data.secondary_imu, main_sample.stamp_ns);
    const Eigen::Vector3d main_omega = config.main_lidar_to_imu_rotation.transpose() *
                                       (main_sample.angular_velocity - main_bias);
    const Eigen::Vector3d secondary_omega = config.secondary_lidar_to_imu_rotation.transpose() *
                                            (secondary_sample.angular_velocity - secondary_bias);
    if (main_omega.norm() >= config.min_angular_speed &&
        secondary_omega.norm() >= config.min_angular_speed) {
      pairs.push_back(AngularVelocityPair{main_omega, secondary_omega, 1.0});
    }

    const double main_acceleration_norm = main_sample.linear_acceleration.norm();
    const double secondary_acceleration_norm = secondary_sample.linear_acceleration.norm();
    if (std::abs(main_acceleration_norm - kGravity) <= config.gravity_tolerance &&
        std::abs(secondary_acceleration_norm - kGravity) <= config.gravity_tolerance &&
        main_acceleration_norm > 1.0 && secondary_acceleration_norm > 1.0) {
      const Eigen::Vector3d main_acceleration = config.main_lidar_to_imu_rotation.transpose() *
                                                main_sample.linear_acceleration.normalized();
      const Eigen::Vector3d secondary_acceleration =
        config.secondary_lidar_to_imu_rotation.transpose() *
        secondary_sample.linear_acceleration.normalized();
      pairs.push_back(AngularVelocityPair{main_acceleration, secondary_acceleration, 0.20});
    }
  }
  return pairs;
}

std::vector<TimedFrame> frameStamps(const std::vector<TimedCloud> & clouds)
{
  std::vector<TimedFrame> frames;
  frames.reserve(clouds.size());
  for (std::size_t i = 0; i < clouds.size(); ++i) {
    frames.push_back(TimedFrame{clouds[i].reference_stamp_ns, i});
  }
  return frames;
}

}  // namespace

CalibrationRunResult runOfflineCalibration(
  const std::string & bag_path,
  const CalibrationConfig & config,
  const std::string & static_bag_path)
{
  CalibrationRunResult result;
  const BagData data = readCalibrationBag(bag_path, config);
  BagData static_data;
  const BagData * stationary_data = &data;
  if (!static_bag_path.empty()) {
    static_data = readCalibrationBag(static_bag_path, config, false);
    stationary_data = &static_data;
  }
  result.main_gyro_bias =
    estimateGyroBias(stationary_data->main_imu, config.stationary_gyro_threshold);
  result.secondary_gyro_bias =
    estimateGyroBias(stationary_data->secondary_imu, config.stationary_gyro_threshold);

  const auto imu_pairs = buildImuVectorPairs(
    data, config, result.main_gyro_bias, result.secondary_gyro_bias);
  result.imu_rotation = estimateImuRotation(imu_pairs, config.min_angular_speed);
  if (!result.imu_rotation.observable) {
    if (static_bag_path.empty()) {
      result.message = "IMU rotation is not observable; provide --static-bag or collect motion "
                       "around at least two axes";
      return result;
    }
    const Eigen::Vector3d main_gravity = config.main_lidar_to_imu_rotation.transpose() *
                                         estimateGravityDirection(
                                           static_data.main_imu, config.gravity_tolerance);
    const Eigen::Vector3d secondary_gravity =
      config.secondary_lidar_to_imu_rotation.transpose() *
      estimateGravityDirection(static_data.secondary_imu, config.gravity_tolerance);
    if (main_gravity.norm() < 0.5 || secondary_gravity.norm() < 0.5) {
      result.message = "Static bag does not contain enough valid gravity samples";
      return result;
    }
    result.imu_rotation.secondary_to_main = planeConstrainedRotation(
      config.initial_secondary_to_main.rotation(), main_gravity, secondary_gravity);
    result.used_static_plane_constraint = true;
  }

  const auto pairs = pairNearestFrames(
    frameStamps(data.main_clouds), frameStamps(data.secondary_clouds), config.sync_tolerance_ns);
  result.synchronized_pairs = pairs.size();
  if (pairs.size() < config.minimum_accepted_frames) {
    result.message = "Too few synchronized LiDAR frame pairs";
    return result;
  }

  Eigen::Isometry3d registration_initial = config.initial_secondary_to_main;
  registration_initial.linear() = result.imu_rotation.secondary_to_main;
  const std::size_t process_count = std::min(pairs.size(), config.max_frame_pairs);
  result.frame_results.reserve(process_count);
  const auto calibration_start = std::chrono::steady_clock::now();
  auto print_progress = [&](const std::size_t completed,
                          const FrameCalibrationResult * frame,
                          const char * rejection_reason) {
    const double elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - calibration_start).count();
    const double remaining_seconds = completed > 0U ?
                                       elapsed_seconds *
                                         static_cast<double>(process_count - completed) /
                                         static_cast<double>(completed) :
                                       0.0;
    std::ostringstream progress;
    progress << std::fixed << std::setprecision(1) << "[dual_lidar_calibration] GICP "
             << completed << '/' << process_count << " elapsed=" << elapsed_seconds
             << "s eta=" << remaining_seconds << 's';
    if (frame != nullptr) {
      progress << std::setprecision(3) << " rmse=" << frame->rmse
               << " overlap=" << frame->overlap_ratio
               << " accepted=" << (frame->accepted ? 1 : 0)
               << " reason="
               << (frame->rejection_reason.empty() ? "accepted" : frame->rejection_reason);
    } else {
      progress << " accepted=0 reason=" << rejection_reason;
    }
    std::cout << progress.str() << '\n';
  };
  std::cout << "[dual_lidar_calibration] Starting GICP for " << process_count
            << " synchronized frame pairs\n";
  for (std::size_t pair_index = 0; pair_index < process_count; ++pair_index) {
    const auto & pair = pairs[pair_index];
    const TimedCloud & main_cloud = data.main_clouds[pair.main_index];
    const TimedCloud & secondary_cloud = data.secondary_clouds[pair.secondary_index];
    const std::int64_t common_reference_ns = main_cloud.reference_stamp_ns +
                                             (secondary_cloud.reference_stamp_ns -
                                               main_cloud.reference_stamp_ns) /
                                               2;
    const std::int64_t main_coverage_begin =
      std::min(main_cloud.begin_stamp_ns, common_reference_ns);
    const std::int64_t main_coverage_end =
      std::max(main_cloud.end_stamp_ns, common_reference_ns);
    const std::int64_t secondary_coverage_begin =
      std::min(secondary_cloud.begin_stamp_ns, common_reference_ns);
    const std::int64_t secondary_coverage_end =
      std::max(secondary_cloud.end_stamp_ns, common_reference_ns);
    if (!imuCoversInterval(data.main_imu, main_coverage_begin, main_coverage_end) ||
        !imuCoversInterval(data.secondary_imu, secondary_coverage_begin, secondary_coverage_end)) {
      ++result.imu_coverage_rejections;
      print_progress(pair_index + 1U, nullptr, "imu_coverage_missing");
      continue;
    }

    const auto main_points = deskewCloudToReference(main_cloud,
      common_reference_ns,
      data.main_imu,
      result.main_gyro_bias,
      config.main_lidar_to_imu_rotation);
    const auto secondary_points = deskewCloudToReference(secondary_cloud,
      common_reference_ns,
      data.secondary_imu,
      result.secondary_gyro_bias,
      config.secondary_lidar_to_imu_rotation);
    FrameCalibrationResult frame = calibrateFrame(pair_index,
      main_cloud.reference_stamp_ns,
      secondary_cloud.reference_stamp_ns,
      main_points,
      secondary_points,
      registration_initial,
      result.imu_rotation.secondary_to_main,
      config);
    if (frame.accepted && result.preview_main.empty()) {
      result.preview_main = main_points;
      result.preview_secondary = secondary_points;
    }
    print_progress(pair_index + 1U, &frame, "");
    result.frame_results.push_back(std::move(frame));
  }

  result.aggregate = aggregateExtrinsics(result.frame_results,
    result.imu_rotation.secondary_to_main,
    config.translation_outlier_threshold,
    config.rotation_outlier_threshold_rad);
  if (!result.aggregate.success || result.aggregate.used_frames < config.minimum_accepted_frames) {
    result.message = "Too few consistent GICP frame estimates";
    return result;
  }
  if (result.aggregate.translation_stddev > config.max_translation_stddev ||
      result.aggregate.rotation_stddev_rad > config.max_rotation_stddev_rad) {
    result.message = "Extrinsic dispersion exceeds configured quality limits";
    return result;
  }
  result.success = true;
  result.message = "Calibration passed all quality gates";
  return result;
}

}  // namespace dual_lidar_calibration
