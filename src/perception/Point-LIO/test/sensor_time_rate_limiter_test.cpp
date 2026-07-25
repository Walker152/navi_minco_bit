#include "sensor_time_rate_limiter.h"

#include <gtest/gtest.h>

#include <limits>

TEST(SensorTimeRateLimiter, PublishesOnlyNewStatesAtConfiguredSensorRate)
{
  point_lio::SensorTimeRateLimiter limiter;

  EXPECT_TRUE(limiter.should_publish(100.000, 200.0));
  EXPECT_FALSE(limiter.should_publish(100.004, 200.0));
  EXPECT_TRUE(limiter.should_publish(100.005, 200.0));
  EXPECT_FALSE(limiter.should_publish(100.005, 200.0));
  EXPECT_TRUE(limiter.should_publish(100.010, 200.0));
}

TEST(SensorTimeRateLimiter, ResetsAfterSensorTimeRollback)
{
  point_lio::SensorTimeRateLimiter limiter;

  EXPECT_TRUE(limiter.should_publish(100.000, 200.0));
  EXPECT_TRUE(limiter.should_publish(10.000, 200.0));
  EXPECT_FALSE(limiter.should_publish(10.004, 200.0));
  EXPECT_TRUE(limiter.should_publish(10.005, 200.0));
}

TEST(SensorTimeRateLimiter, RejectsInvalidInput)
{
  point_lio::SensorTimeRateLimiter limiter;

  EXPECT_FALSE(limiter.should_publish(
    std::numeric_limits<double>::quiet_NaN(), 200.0));
  EXPECT_FALSE(limiter.should_publish(100.000, 0.0));
  EXPECT_FALSE(limiter.should_publish(100.000, -1.0));
}
