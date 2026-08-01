#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "dual_lidar_calibration/frame_synchronizer.hpp"

namespace dual_lidar_calibration {
namespace {

TEST(FrameSynchronizer, PairsNearestFramesOnceWithinTolerance)
{
  const std::vector<TimedFrame> main_frames{
    {100'000'000LL, 0U},
    {200'000'000LL, 1U},
    {300'000'000LL, 2U},
  };
  const std::vector<TimedFrame> secondary_frames{
    {96'000'000LL, 0U},
    {204'000'000LL, 1U},
    {320'000'000LL, 2U},
  };

  const auto pairs = pairNearestFrames(main_frames, secondary_frames, 5'000'000LL);

  ASSERT_EQ(pairs.size(), 2U);
  EXPECT_EQ(pairs[0].main_index, 0U);
  EXPECT_EQ(pairs[0].secondary_index, 0U);
  EXPECT_EQ(pairs[0].delta_ns, -4'000'000LL);
  EXPECT_EQ(pairs[1].main_index, 1U);
  EXPECT_EQ(pairs[1].secondary_index, 1U);
  EXPECT_EQ(pairs[1].delta_ns, 4'000'000LL);
}

}  // namespace
}  // namespace dual_lidar_calibration
