#include "time_batching.h"

#include <cstddef>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

namespace
{

std::vector<int> batches(const std::vector<double> & timestamps_ms, double time_bin_ms)
{
  return point_lio::build_time_batches(
    timestamps_ms.size(), time_bin_ms,
    [&timestamps_ms](std::size_t index) { return timestamps_ms[index]; });
}

}  // namespace

TEST(TimeBatching, EmptyInputProducesNoBatch)
{
  EXPECT_EQ(batches({}, 0.25), std::vector<int>());
}

TEST(TimeBatching, ZeroBinPreservesPerTimestampBehavior)
{
  EXPECT_EQ(
    batches({0.00, 0.00, 0.10, 0.10, 0.20}, 0.0),
    (std::vector<int>{2, 2, 1}));
}

TEST(TimeBatching, PositiveBinCombinesAdjacentTimestamps)
{
  EXPECT_EQ(
    batches({0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32}, 0.25),
    (std::vector<int>{7, 2}));
  EXPECT_EQ(
    batches({0.00, 0.00, 0.00, 0.30}, 0.25),
    (std::vector<int>{3, 1}));
}

TEST(TimeBatching, EveryPointIsRetained)
{
  const std::vector<double> timestamps_ms{0.00, 0.04, 0.08, 0.12, 0.28, 0.32};
  const auto result = batches(timestamps_ms, 0.25);
  EXPECT_EQ(
    std::accumulate(result.begin(), result.end(), 0),
    static_cast<int>(timestamps_ms.size()));
}
