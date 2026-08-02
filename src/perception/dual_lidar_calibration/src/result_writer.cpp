#include "dual_lidar_calibration/result_writer.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dual_lidar_calibration {

namespace {

void atomicWrite(const std::filesystem::path & path, const std::string & contents)
{
  const std::filesystem::path temporary = path.string() + ".tmp";
  {
    std::ofstream stream(temporary);
    if (!stream) {
      throw std::runtime_error("Cannot open output file: " + temporary.string());
    }
    stream << contents;
    if (!stream) {
      throw std::runtime_error("Failed while writing output file: " + temporary.string());
    }
  }
  std::filesystem::rename(temporary, path);
}

YAML::Node vectorNode(const Eigen::Vector3d & value)
{
  YAML::Node node;
  node.push_back(value.x());
  node.push_back(value.y());
  node.push_back(value.z());
  node.SetStyle(YAML::EmitterStyle::Flow);
  return node;
}

YAML::Node matrixNode(const Eigen::Matrix4d & matrix)
{
  YAML::Node rows;
  for (int row = 0; row < 4; ++row) {
    YAML::Node values;
    for (int column = 0; column < 4; ++column) {
      values.push_back(matrix(row, column));
    }
    values.SetStyle(YAML::EmitterStyle::Flow);
    rows.push_back(values);
  }
  return rows;
}

std::string frameResultsCsv(const CalibrationRunResult & result)
{
  std::ostringstream stream;
  stream << std::setprecision(12);
  stream << "pair_index,main_stamp_ns,secondary_stamp_ns,converged,accepted,rejection_reason,"
            "rmse,overlap_ratio,inliers,x,y,z,roll,pitch,yaw\n";
  for (const auto & frame : result.frame_results) {
    const Eigen::Vector3d rpy = rpyFromRotation(frame.secondary_to_main.rotation());
    const Eigen::Vector3d translation = frame.secondary_to_main.translation();
    stream << frame.pair_index << ',' << frame.main_stamp_ns << ',' << frame.secondary_stamp_ns << ','
           << (frame.converged ? 1 : 0) << ',' << (frame.accepted ? 1 : 0) << ','
           << frame.rejection_reason << ',' << frame.rmse << ',' << frame.overlap_ratio << ','
           << frame.inliers << ',' << translation.x() << ',' << translation.y() << ','
           << translation.z() << ',' << rpy.x() << ',' << rpy.y() << ',' << rpy.z() << '\n';
  }
  return stream.str();
}

void writePreview(const std::filesystem::path & path, const CalibrationRunResult & result)
{
  if (result.preview_main.empty() || result.preview_secondary.empty() || !result.aggregate.success) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(result.preview_main.size() + result.preview_secondary.size());
  for (const auto & point : result.preview_main) {
    pcl::PointXYZRGB output;
    output.x = static_cast<float>(point.x());
    output.y = static_cast<float>(point.y());
    output.z = static_cast<float>(point.z());
    output.r = 255U;
    output.g = 64U;
    output.b = 64U;
    cloud.push_back(output);
  }
  for (const auto & point : result.preview_secondary) {
    const Eigen::Vector3d transformed = result.aggregate.secondary_to_main * point;
    pcl::PointXYZRGB output;
    output.x = static_cast<float>(transformed.x());
    output.y = static_cast<float>(transformed.y());
    output.z = static_cast<float>(transformed.z());
    output.r = 64U;
    output.g = 255U;
    output.b = 255U;
    cloud.push_back(output);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = false;
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("Failed to write aligned preview PCD: " + path.string());
  }
}

}  // namespace

void writeCalibrationResults(const std::string & output_directory,
  const CalibrationConfig & config,
  const CalibrationRunResult & result)
{
  const std::filesystem::path output_path(output_directory);
  std::filesystem::create_directories(output_path);

  YAML::Node summary;
  summary["success"] = result.success;
  summary["quality"] = result.success ? "PASS" : "FAIL";
  summary["message"] = result.message;
  summary["synchronized_pairs"] = result.synchronized_pairs;
  summary["processed_pairs"] = result.frame_results.size();
  summary["accepted_frames"] = result.aggregate.used_frames;
  summary["imu_coverage_rejections"] = result.imu_coverage_rejections;
  summary["main_gyro_bias"] = vectorNode(result.main_gyro_bias);
  summary["secondary_gyro_bias"] = vectorNode(result.secondary_gyro_bias);
  summary["imu_rotation_observable"] = result.imu_rotation.observable;
  summary["rotation_constraint_mode"] =
    result.used_static_plane_constraint ? "static_gravity_plane" : "full_imu_rotation";
  summary["imu_rotation_rms"] = result.imu_rotation.rms_residual;
  summary["imu_used_samples"] = result.imu_rotation.used_samples;
  summary["imu_rotation_rpy_rad"] =
    vectorNode(rpyFromRotation(result.imu_rotation.secondary_to_main));
  summary["imu_information_singular_values"] = vectorNode(result.imu_rotation.singular_values);
  summary["translation_stddev_m"] = result.aggregate.translation_stddev;
  summary["rotation_stddev_rad"] = result.aggregate.rotation_stddev_rad;
  YAML::Emitter summary_emitter;
  summary_emitter << summary;
  atomicWrite(output_path / "summary.yaml", summary_emitter.c_str());
  atomicWrite(output_path / "frame_results.csv", frameResultsCsv(result));

  if (!result.success) {
    return;
  }

  const Eigen::Vector3d translation = result.aggregate.secondary_to_main.translation();
  const Eigen::Vector3d rpy = rpyFromRotation(result.aggregate.secondary_to_main.rotation());
  YAML::Node extrinsic;
  YAML::Node merge;
  merge.push_back(translation.x());
  merge.push_back(translation.y());
  merge.push_back(translation.z());
  merge.push_back(rpy.x());
  merge.push_back(rpy.y());
  merge.push_back(rpy.z());
  merge.SetStyle(YAML::EmitterStyle::Flow);
  extrinsic["merge_extrinsic_back_to_front"] = merge;
  extrinsic["convention"] = "p_main = R_secondary_to_main * p_secondary + t_secondary_to_main";
  extrinsic["translation_m"] = vectorNode(translation);
  extrinsic["rotation_rpy_rad"] = vectorNode(rpy);
  extrinsic["rotation_constraint_mode"] =
    result.used_static_plane_constraint ? "static_gravity_plane" : "full_imu_rotation";
  extrinsic["matrix"] = matrixNode(result.aggregate.secondary_to_main.matrix());
  extrinsic["source_topics"]["main_lidar"] = config.main_lidar_topic;
  extrinsic["source_topics"]["secondary_lidar"] = config.secondary_lidar_topic;
  YAML::Emitter extrinsic_emitter;
  extrinsic_emitter << extrinsic;
  atomicWrite(output_path / "extrinsic.yaml", extrinsic_emitter.c_str());

  std::ostringstream matrix_stream;
  matrix_stream << std::setprecision(12) << result.aggregate.secondary_to_main.matrix() << '\n';
  atomicWrite(output_path / "extrinsic_matrix.txt", matrix_stream.str());
  writePreview(output_path / "aligned_preview.pcd", result);
}

}  // namespace dual_lidar_calibration
