//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "internal_lidar_merger.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "sensor_msgs/msg/point_field.hpp"

namespace livox_ros {

bool IsInternalLidarMergeTransferFormatSupported(int transfer_format)
{
  return transfer_format == 0 || transfer_format == 1;
}

InternalLidarMerger::InternalLidarMerger()
{
  RecomputeTransform();
}

InternalLidarMerger::InternalLidarMerger(const InternalLidarMergeConfig & config)
{
  Configure(config);
}

void InternalLidarMerger::Configure(const InternalLidarMergeConfig & config)
{
  config_ = config;
  RecomputeTransform();
}

bool InternalLidarMerger::CanMerge(uint64_t front_base_time, uint64_t back_base_time) const
{
  return TimeDiff(front_base_time, back_base_time) <= config_.max_interval_ns;
}

bool InternalLidarMerger::ShouldDropFront(uint64_t front_base_time, uint64_t back_base_time) const
{
  return front_base_time <= back_base_time;
}

PointXyzlt InternalLidarMerger::TransformBackPoint(const PointXyzlt & point) const
{
  PointXyzlt transformed = point;
  transformed.x = r00_ * point.x + r01_ * point.y + r02_ * point.z + tx_;
  transformed.y = r10_ * point.x + r11_ * point.y + r12_ * point.z + ty_;
  transformed.z = r20_ * point.x + r21_ * point.y + r22_ * point.z + tz_;
  return transformed;
}

std::unique_ptr<livox_ros_driver2::msg::CustomMsg> InternalLidarMerger::BuildMergedCustomMsg(
  const StoragePacket & front, const StoragePacket & back) const
{
  const uint64_t timebase = std::min(front.base_time, back.base_time);
  auto msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  msg->header.frame_id = config_.frame_id;
  msg->header.stamp = ToRosTime(timebase);
  msg->timebase = timebase;
  msg->point_num = front.points_num + back.points_num;
  msg->lidar_id = static_cast<uint8_t>(config_.front_handle);
  msg->points.reserve(msg->point_num);

  for (const auto & point : front.points) {
    livox_ros_driver2::msg::CustomPoint out{};
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    out.reflectivity = static_cast<uint8_t>(point.intensity);
    out.tag = point.tag;
    out.line = point.line;
    out.offset_time = RelativeOffset(point.offset_time, timebase);
    msg->points.push_back(out);
  }

  for (const auto & point : back.points) {
    const PointXyzlt transformed = TransformBackPoint(point);
    livox_ros_driver2::msg::CustomPoint out{};
    out.x = transformed.x;
    out.y = transformed.y;
    out.z = transformed.z;
    out.reflectivity = static_cast<uint8_t>(transformed.intensity);
    out.tag = transformed.tag;
    out.line = transformed.line;
    out.offset_time = RelativeOffset(transformed.offset_time, timebase);
    msg->points.push_back(out);
  }

  msg->point_num = static_cast<uint32_t>(msg->points.size());
  return msg;
}

std::unique_ptr<sensor_msgs::msg::PointCloud2> InternalLidarMerger::BuildMergedPointCloud2(
  const StoragePacket & front, const StoragePacket & back) const
{
  const uint64_t timebase = std::min(front.base_time, back.base_time);
  const uint32_t point_num = front.points_num + back.points_num;
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = config_.frame_id;
  cloud->header.stamp = ToRosTime(timebase);
  cloud->height = 1;
  cloud->width = point_num;
  cloud->is_bigendian = false;
  cloud->is_dense = true;
  cloud->point_step = sizeof(LivoxPointXyzrtlt);
  cloud->row_step = cloud->width * cloud->point_step;
  FillPointCloud2Fields(*cloud);
  cloud->data.resize(static_cast<size_t>(point_num) * sizeof(LivoxPointXyzrtlt));

  auto * dst = reinterpret_cast<LivoxPointXyzrtlt *>(cloud->data.data());
  size_t out_index = 0;
  for (const auto & point : front.points) {
    dst[out_index].x = point.x;
    dst[out_index].y = point.y;
    dst[out_index].z = point.z;
    dst[out_index].reflectivity = point.intensity;
    dst[out_index].tag = point.tag;
    dst[out_index].line = point.line;
    dst[out_index].timestamp = static_cast<double>(RelativeOffset(point.offset_time, timebase));
    ++out_index;
  }

  for (const auto & point : back.points) {
    const PointXyzlt transformed = TransformBackPoint(point);
    dst[out_index].x = transformed.x;
    dst[out_index].y = transformed.y;
    dst[out_index].z = transformed.z;
    dst[out_index].reflectivity = transformed.intensity;
    dst[out_index].tag = transformed.tag;
    dst[out_index].line = transformed.line;
    dst[out_index].timestamp = static_cast<double>(RelativeOffset(transformed.offset_time, timebase));
    ++out_index;
  }

  return cloud;
}

uint64_t InternalLidarMerger::TimeDiff(uint64_t lhs, uint64_t rhs)
{
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

uint32_t InternalLidarMerger::RelativeOffset(uint64_t point_time, uint64_t base_time)
{
  if (point_time <= base_time) {
    return 0;
  }
  const uint64_t offset = point_time - base_time;
  return static_cast<uint32_t>(std::min<uint64_t>(offset, std::numeric_limits<uint32_t>::max()));
}

builtin_interfaces::msg::Time InternalLidarMerger::ToRosTime(uint64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(timestamp_ns / kNsPerSecond);
  stamp.nanosec = static_cast<uint32_t>(timestamp_ns % kNsPerSecond);
  return stamp;
}

void InternalLidarMerger::FillPointCloud2Fields(sensor_msgs::msg::PointCloud2 & cloud)
{
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64;
}

void InternalLidarMerger::RecomputeTransform()
{
  tx_ = static_cast<float>(config_.extrinsic_back_to_front[0]);
  ty_ = static_cast<float>(config_.extrinsic_back_to_front[1]);
  tz_ = static_cast<float>(config_.extrinsic_back_to_front[2]);

  const double roll = config_.extrinsic_back_to_front[3];
  const double pitch = config_.extrinsic_back_to_front[4];
  const double yaw = config_.extrinsic_back_to_front[5];
  const double cos_roll = std::cos(roll);
  const double cos_pitch = std::cos(pitch);
  const double cos_yaw = std::cos(yaw);
  const double sin_roll = std::sin(roll);
  const double sin_pitch = std::sin(pitch);
  const double sin_yaw = std::sin(yaw);

  r00_ = static_cast<float>(cos_pitch * cos_yaw);
  r01_ = static_cast<float>(sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw);
  r02_ = static_cast<float>(cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw);

  r10_ = static_cast<float>(cos_pitch * sin_yaw);
  r11_ = static_cast<float>(sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw);
  r12_ = static_cast<float>(cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw);

  r20_ = static_cast<float>(-sin_pitch);
  r21_ = static_cast<float>(sin_roll * cos_pitch);
  r22_ = static_cast<float>(cos_roll * cos_pitch);
}

}  // namespace livox_ros
