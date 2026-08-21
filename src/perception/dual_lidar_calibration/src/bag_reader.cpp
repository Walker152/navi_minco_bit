#include "dual_lidar_calibration/bag_reader.hpp"

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace dual_lidar_calibration {

namespace {

constexpr const char * kLivoxType = "livox_ros_driver2/msg/CustomMsg";
constexpr const char * kImuType = "sensor_msgs/msg/Imu";
constexpr double kStandardGravity = 9.80665;

struct VoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

template<typename MessageT>
MessageT deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  MessageT message;
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

std::int64_t headerStampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

TimedCloud convertCloud(
  const livox_ros_driver2::msg::CustomMsg & message, const CalibrationConfig & config)
{
  TimedCloud cloud;
  const std::int64_t header_stamp = headerStampNs(message.header.stamp);
  const std::int64_t timebase = message.timebase > 0U ?
                                  static_cast<std::int64_t>(message.timebase) : header_stamp;
  if (timebase <= 0) {
    throw std::runtime_error("Livox CustomMsg contains neither a valid timebase nor header stamp");
  }

  const double min_range_sq = config.min_range * config.min_range;
  const double max_range_sq = config.max_range * config.max_range;
  std::uint32_t min_offset = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_offset = 0U;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
  occupied_voxels.reserve(message.points.size() / config.point_stride + 1U);
  cloud.points.reserve(message.points.size() / config.point_stride + 1U);
  for (std::size_t i = 0; i < message.points.size(); i += config.point_stride) {
    const auto & point = message.points[i];
    const Eigen::Vector3d position(point.x, point.y, point.z);
    const double squared_range = position.squaredNorm();
    if (!position.allFinite() || squared_range < min_range_sq || squared_range > max_range_sq) {
      continue;
    }
    min_offset = std::min(min_offset, point.offset_time);
    max_offset = std::max(max_offset, point.offset_time);
    const VoxelKey voxel{static_cast<std::int64_t>(std::floor(position.x() / config.voxel_size)),
      static_cast<std::int64_t>(std::floor(position.y() / config.voxel_size)),
      static_cast<std::int64_t>(std::floor(position.z() / config.voxel_size))};
    if (!occupied_voxels.insert(voxel).second) {
      continue;
    }
    cloud.points.push_back(
      TimedPoint{position, timebase + static_cast<std::int64_t>(point.offset_time)});
  }
  if (cloud.points.empty()) {
    return cloud;
  }
  cloud.begin_stamp_ns = timebase + static_cast<std::int64_t>(min_offset);
  cloud.end_stamp_ns = timebase + static_cast<std::int64_t>(max_offset);
  cloud.reference_stamp_ns =
    cloud.begin_stamp_ns + (cloud.end_stamp_ns - cloud.begin_stamp_ns) / 2;
  return cloud;
}

ImuSample convertImu(const sensor_msgs::msg::Imu & message)
{
  ImuSample sample;
  sample.stamp_ns = headerStampNs(message.header.stamp);
  sample.angular_velocity = Eigen::Vector3d(message.angular_velocity.x,
    message.angular_velocity.y,
    message.angular_velocity.z);
  sample.linear_acceleration = livoxAccelerationToSi(Eigen::Vector3d(message.linear_acceleration.x,
    message.linear_acceleration.y,
    message.linear_acceleration.z));
  return sample;
}

void requireTopic(const std::map<std::string, std::string> & topics,
  const std::string & topic,
  const char * expected_type)
{
  const auto found = topics.find(topic);
  if (found == topics.end()) {
    throw std::runtime_error("Required topic is missing from bag: " + topic);
  }
  if (found->second != expected_type) {
    throw std::runtime_error(
      "Topic '" + topic + "' has type '" + found->second + "', expected '" + expected_type + "'");
  }
}

}  // namespace

Eigen::Vector3d livoxAccelerationToSi(const Eigen::Vector3d & acceleration_g)
{
  return kStandardGravity * acceleration_g;
}

BagData readCalibrationBag(
  const std::string & bag_path, const CalibrationConfig & config, const bool include_lidar)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  std::map<std::string, std::string> topic_types;
  for (const auto & metadata : reader.get_all_topics_and_types()) {
    topic_types[metadata.name] = metadata.type;
  }
  if (include_lidar) {
    requireTopic(topic_types, config.main_lidar_topic, kLivoxType);
    requireTopic(topic_types, config.secondary_lidar_topic, kLivoxType);
  }
  requireTopic(topic_types, config.main_imu_topic, kImuType);
  requireTopic(topic_types, config.secondary_imu_topic, kImuType);

  BagData data;
  std::size_t main_lidar_count = 0U;
  std::size_t secondary_lidar_count = 0U;
  const std::size_t maximum_sampled_frames =
    config.max_frame_pairs + config.minimum_accepted_frames;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name == config.main_imu_topic) {
      const ImuSample sample = convertImu(deserialize<sensor_msgs::msg::Imu>(bag_message));
      if (sample.stamp_ns > 0 && sample.angular_velocity.allFinite() &&
          sample.linear_acceleration.allFinite()) {
        data.main_imu.push_back(sample);
      }
      continue;
    }
    if (bag_message->topic_name == config.secondary_imu_topic) {
      const ImuSample sample = convertImu(deserialize<sensor_msgs::msg::Imu>(bag_message));
      if (sample.stamp_ns > 0 && sample.angular_velocity.allFinite() &&
          sample.linear_acceleration.allFinite()) {
        data.secondary_imu.push_back(sample);
      }
      continue;
    }
    if (include_lidar && bag_message->topic_name == config.main_lidar_topic) {
      const bool selected = main_lidar_count++ % config.frame_stride == 0U;
      if (selected && data.main_clouds.size() < maximum_sampled_frames) {
        TimedCloud cloud =
          convertCloud(deserialize<livox_ros_driver2::msg::CustomMsg>(bag_message), config);
        if (!cloud.points.empty()) {
          data.main_clouds.push_back(std::move(cloud));
        }
      }
      continue;
    }
    if (include_lidar && bag_message->topic_name == config.secondary_lidar_topic) {
      const bool selected = secondary_lidar_count++ % config.frame_stride == 0U;
      if (selected && data.secondary_clouds.size() < maximum_sampled_frames) {
        TimedCloud cloud =
          convertCloud(deserialize<livox_ros_driver2::msg::CustomMsg>(bag_message), config);
        if (!cloud.points.empty()) {
          data.secondary_clouds.push_back(std::move(cloud));
        }
      }
    }
  }

  auto imu_order = [](const ImuSample & lhs, const ImuSample & rhs) {
    return lhs.stamp_ns < rhs.stamp_ns;
  };
  auto cloud_order = [](const TimedCloud & lhs, const TimedCloud & rhs) {
    return lhs.reference_stamp_ns < rhs.reference_stamp_ns;
  };
  std::sort(data.main_imu.begin(), data.main_imu.end(), imu_order);
  std::sort(data.secondary_imu.begin(), data.secondary_imu.end(), imu_order);
  std::sort(data.main_clouds.begin(), data.main_clouds.end(), cloud_order);
  std::sort(data.secondary_clouds.begin(), data.secondary_clouds.end(), cloud_order);
  if (data.main_imu.size() < 3U || data.secondary_imu.size() < 3U) {
    throw std::runtime_error("Bag does not contain enough valid IMU data for calibration");
  }
  if (include_lidar && (data.main_clouds.empty() || data.secondary_clouds.empty())) {
    throw std::runtime_error("Bag does not contain enough valid LiDAR/IMU data for calibration");
  }
  return data;
}

}  // namespace dual_lidar_calibration
