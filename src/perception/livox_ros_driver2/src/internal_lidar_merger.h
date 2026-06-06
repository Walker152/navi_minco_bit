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

#ifndef LIVOX_ROS_DRIVER2_INTERNAL_LIDAR_MERGER_H_
#define LIVOX_ROS_DRIVER2_INTERNAL_LIDAR_MERGER_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "comm/comm.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace livox_ros
{

bool IsInternalLidarMergeTransferFormatSupported(int transfer_format);

struct InternalLidarMergeConfig
{
  bool enabled = false;
  uint32_t front_handle = 0;
  uint32_t back_handle = 0;
  std::string output_topic = "livox/lidar";
  std::string frame_id = "livox_frame";
  uint64_t max_interval_ns = 5000000;
  std::array<double, 6> extrinsic_back_to_front{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

class InternalLidarMerger
{
public:
  InternalLidarMerger();
  explicit InternalLidarMerger(const InternalLidarMergeConfig & config);

  void Configure(const InternalLidarMergeConfig & config);
  const InternalLidarMergeConfig & GetConfig() const { return config_; }

  bool CanMerge(uint64_t front_base_time, uint64_t back_base_time) const;
  bool ShouldDropFront(uint64_t front_base_time, uint64_t back_base_time) const;

  PointXyzlt TransformBackPoint(const PointXyzlt & point) const;

  std::unique_ptr<livox_ros_driver2::msg::CustomMsg> BuildMergedCustomMsg(
    const StoragePacket & front,
    const StoragePacket & back) const;

  std::unique_ptr<sensor_msgs::msg::PointCloud2> BuildMergedPointCloud2(
    const StoragePacket & front,
    const StoragePacket & back) const;

private:
  static uint64_t TimeDiff(uint64_t lhs, uint64_t rhs);
  static uint32_t RelativeOffset(uint64_t point_time, uint64_t base_time);
  static builtin_interfaces::msg::Time ToRosTime(uint64_t timestamp_ns);
  static void FillPointCloud2Fields(sensor_msgs::msg::PointCloud2 & cloud);

  void RecomputeTransform();

  InternalLidarMergeConfig config_;
  float r00_ = 1.0F;
  float r01_ = 0.0F;
  float r02_ = 0.0F;
  float r10_ = 0.0F;
  float r11_ = 1.0F;
  float r12_ = 0.0F;
  float r20_ = 0.0F;
  float r21_ = 0.0F;
  float r22_ = 1.0F;
  float tx_ = 0.0F;
  float ty_ = 0.0F;
  float tz_ = 0.0F;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_INTERNAL_LIDAR_MERGER_H_
