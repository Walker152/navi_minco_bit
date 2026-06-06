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

#include <cmath>
#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include "comm/comm.h"
#include "internal_lidar_merger.h"
#include "lddc.h"

namespace livox_ros
{
namespace
{

constexpr uint64_t kMs = 1000000ULL;

PointXyzlt MakePoint(float x, float y, float z, float intensity, uint8_t tag, uint8_t line, uint64_t offset_time)
{
  PointXyzlt point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.tag = tag;
  point.line = line;
  point.offset_time = offset_time;
  return point;
}

StoragePacket MakePacket(uint32_t handle, uint64_t base_time, std::initializer_list<PointXyzlt> points)
{
  StoragePacket packet{};
  packet.lidar_type = kLivoxLidarType;
  packet.handle = handle;
  packet.base_time = base_time;
  packet.points.assign(points.begin(), points.end());
  packet.points_num = static_cast<uint32_t>(packet.points.size());
  return packet;
}

InternalLidarMergeConfig MakeConfig()
{
  InternalLidarMergeConfig config{};
  config.enabled = true;
  config.front_handle = IpStringToNum("192.168.1.135");
  config.back_handle = IpStringToNum("192.168.1.122");
  config.output_topic = "livox/lidar";
  config.frame_id = "livox_frame";
  config.max_interval_ns = 5 * kMs;
  config.extrinsic_back_to_front = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
  return config;
}

}  // namespace

TEST(InternalLidarMergerTest, TransformsBackPointsIntoFrontFrame)
{
  InternalLidarMerger merger(MakeConfig());
  const PointXyzlt back = MakePoint(2.0F, 3.0F, 4.0F, 9.0F, 0x10, 2, 1100);

  const PointXyzlt transformed = merger.TransformBackPoint(back);

  EXPECT_FLOAT_EQ(transformed.x, 3.0F);
  EXPECT_FLOAT_EQ(transformed.y, 5.0F);
  EXPECT_FLOAT_EQ(transformed.z, 7.0F);
  EXPECT_FLOAT_EQ(transformed.intensity, 9.0F);
  EXPECT_EQ(transformed.tag, 0x10);
  EXPECT_EQ(transformed.line, 2);
  EXPECT_EQ(transformed.offset_time, 1100U);
}

TEST(InternalLidarMergerTest, AppliesPrecomputedRollPitchYawTransform)
{
  auto config = MakeConfig();
  config.extrinsic_back_to_front = {0.0, 0.0, 0.0, PI / 2.0, 0.0, 0.0};
  InternalLidarMerger merger(config);
  const PointXyzlt back = MakePoint(0.0F, 1.0F, 0.0F, 1.0F, 0x01, 0, 100);

  const PointXyzlt transformed = merger.TransformBackPoint(back);

  EXPECT_NEAR(transformed.x, 0.0F, 1e-6F);
  EXPECT_NEAR(transformed.y, 0.0F, 1e-6F);
  EXPECT_NEAR(transformed.z, 1.0F, 1e-6F);
}

TEST(InternalLidarMergerTest, BuildsMergedCustomMsgWithSharedTimebaseAndRelativeOffsets)
{
  const auto config = MakeConfig();
  InternalLidarMerger merger(config);
  const auto front = MakePacket(config.front_handle, 1000, {
    MakePoint(1.0F, 2.0F, 3.0F, 5.0F, 0x10, 1, 1010),
  });
  const auto back = MakePacket(config.back_handle, 1200, {
    MakePoint(2.0F, 3.0F, 4.0F, 7.0F, 0x20, 3, 1250),
  });

  auto msg = merger.BuildMergedCustomMsg(front, back);

  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "livox_frame");
  EXPECT_EQ(msg->header.stamp.sec, 0);
  EXPECT_EQ(msg->header.stamp.nanosec, 1000U);
  EXPECT_EQ(msg->timebase, 1000U);
  EXPECT_EQ(msg->point_num, 2U);
  EXPECT_EQ(msg->lidar_id, static_cast<uint8_t>(config.front_handle));
  ASSERT_EQ(msg->points.size(), 2U);

  EXPECT_FLOAT_EQ(msg->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(msg->points[0].y, 2.0F);
  EXPECT_FLOAT_EQ(msg->points[0].z, 3.0F);
  EXPECT_EQ(msg->points[0].reflectivity, 5U);
  EXPECT_EQ(msg->points[0].tag, 0x10);
  EXPECT_EQ(msg->points[0].line, 1U);
  EXPECT_EQ(msg->points[0].offset_time, 10U);

  EXPECT_FLOAT_EQ(msg->points[1].x, 3.0F);
  EXPECT_FLOAT_EQ(msg->points[1].y, 5.0F);
  EXPECT_FLOAT_EQ(msg->points[1].z, 7.0F);
  EXPECT_EQ(msg->points[1].reflectivity, 7U);
  EXPECT_EQ(msg->points[1].tag, 0x20);
  EXPECT_EQ(msg->points[1].line, 3U);
  EXPECT_EQ(msg->points[1].offset_time, 250U);
}

TEST(InternalLidarMergerTest, BuildsMergedPointCloud2WithDriverFields)
{
  const auto config = MakeConfig();
  InternalLidarMerger merger(config);
  const auto front = MakePacket(config.front_handle, 10 * kMs, {
    MakePoint(1.0F, 2.0F, 3.0F, 5.0F, 0x10, 1, 10 * kMs + 100),
  });
  const auto back = MakePacket(config.back_handle, 10 * kMs + 1000, {
    MakePoint(2.0F, 3.0F, 4.0F, 7.0F, 0x20, 3, 10 * kMs + 1300),
  });

  auto cloud = merger.BuildMergedPointCloud2(front, back);

  ASSERT_NE(cloud, nullptr);
  EXPECT_EQ(cloud->header.frame_id, "livox_frame");
  EXPECT_EQ(cloud->header.stamp.sec, 0);
  EXPECT_EQ(cloud->header.stamp.nanosec, 10 * kMs);
  EXPECT_EQ(cloud->height, 1U);
  EXPECT_EQ(cloud->width, 2U);
  EXPECT_EQ(cloud->fields.size(), 7U);
  EXPECT_EQ(cloud->fields[0].name, "x");
  EXPECT_EQ(cloud->fields[6].name, "timestamp");
  EXPECT_EQ(cloud->point_step, sizeof(LivoxPointXyzrtlt));
  EXPECT_EQ(cloud->data.size(), 2U * sizeof(LivoxPointXyzrtlt));

  LivoxPointXyzrtlt points[2]{};
  std::memcpy(points, cloud->data.data(), cloud->data.size());
  EXPECT_FLOAT_EQ(points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(points[0].y, 2.0F);
  EXPECT_FLOAT_EQ(points[0].z, 3.0F);
  EXPECT_FLOAT_EQ(points[0].reflectivity, 5.0F);
  EXPECT_EQ(points[0].tag, 0x10);
  EXPECT_EQ(points[0].line, 1);
  EXPECT_DOUBLE_EQ(points[0].timestamp, 100.0);

  EXPECT_FLOAT_EQ(points[1].x, 3.0F);
  EXPECT_FLOAT_EQ(points[1].y, 5.0F);
  EXPECT_FLOAT_EQ(points[1].z, 7.0F);
  EXPECT_FLOAT_EQ(points[1].reflectivity, 7.0F);
  EXPECT_EQ(points[1].tag, 0x20);
  EXPECT_EQ(points[1].line, 3);
  EXPECT_DOUBLE_EQ(points[1].timestamp, 1300.0);
}

TEST(InternalLidarMergerTest, RejectsFramesOutsideSyncWindowAndChoosesOlderDropSide)
{
  InternalLidarMerger merger(MakeConfig());

  EXPECT_TRUE(merger.CanMerge(1000, 1000 + 5 * kMs));
  EXPECT_FALSE(merger.CanMerge(1000, 1000 + 5 * kMs + 1));
  EXPECT_TRUE(merger.ShouldDropFront(1000, 1000 + 5 * kMs + 1));
  EXPECT_FALSE(merger.ShouldDropFront(1000 + 5 * kMs + 1, 1000));
}

TEST(InternalLidarMergerTest, SupportsOnlyRos2PointCloud2AndCustomMsgTransferFormats)
{
  EXPECT_TRUE(IsInternalLidarMergeTransferFormatSupported(kPointCloud2Msg));
  EXPECT_TRUE(IsInternalLidarMergeTransferFormatSupported(kLivoxCustomMsg));
  EXPECT_FALSE(IsInternalLidarMergeTransferFormatSupported(kPclPxyziMsg));
}

}  // namespace livox_ros
