#include <gtest/gtest.h>

#include "ivox/ivox3d.h"

namespace {

using TestIVox = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, pcl::PointXYZINormal>;

TestIVox MakeIVox()
{
  TestIVox::Options options;
  options.resolution_ = 2.0F;
  options.point_resolution_ = 0.5F;
  options.nearby_type_ = TestIVox::NearbyType::CENTER;
  return TestIVox(options);
}

pcl::PointXYZINormal MakePoint(float x, float y, float z)
{
  pcl::PointXYZINormal point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(IVoxMapVoxelDedup, RepeatedPointsDoNotGrowOneMapVoxel)
{
  auto ivox = MakeIVox();
  TestIVox::PointVector points;
  for (int i = 0; i < 1000; ++i) {
    points.emplace_back(MakePoint(0.1F + 0.0001F * static_cast<float>(i % 100), 0.1F, 0.1F));
  }

  ivox.AddPoints(points);

  const auto stats = ivox.StatGridPoints();
  ASSERT_GE(stats.size(), 3U);
  EXPECT_FLOAT_EQ(stats[2], 1.0F);
}

TEST(IVoxMapVoxelDedup, TwoMeterGridHasAtMostSixtyFourHalfMeterVoxels)
{
  auto ivox = MakeIVox();
  TestIVox::PointVector points;
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      for (int z = 0; z < 4; ++z) {
        points.emplace_back(MakePoint(0.25F + 0.5F * static_cast<float>(x),
          0.25F + 0.5F * static_cast<float>(y),
          0.25F + 0.5F * static_cast<float>(z)));
      }
    }
  }
  ivox.AddPoints(points);
  ivox.AddPoints(points);

  const auto stats = ivox.StatGridPoints();
  ASSERT_GE(stats.size(), 3U);
  EXPECT_FLOAT_EQ(stats[2], 64.0F);
}

TEST(IVoxMapVoxelDedup, CloserPointReplacesExistingRepresentative)
{
  auto ivox = MakeIVox();
  ivox.AddPoints(TestIVox::PointVector{MakePoint(0.49F, 0.49F, 0.49F)});
  ivox.AddPoints(TestIVox::PointVector{MakePoint(0.25F, 0.25F, 0.25F)});

  TestIVox::PointVector nearest;
  ASSERT_TRUE(ivox.GetClosestPoint(MakePoint(0.25F, 0.25F, 0.25F), nearest, 1));
  ASSERT_EQ(nearest.size(), 1U);
  EXPECT_FLOAT_EQ(nearest.front().x, 0.25F);
  EXPECT_FLOAT_EQ(nearest.front().y, 0.25F);
  EXPECT_FLOAT_EQ(nearest.front().z, 0.25F);
}

}  // namespace
