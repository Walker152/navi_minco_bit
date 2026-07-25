#pragma once

#include <cmath>
#include <limits>

namespace point_lio
{

class SensorTimeRateLimiter
{
public:
  bool should_publish(double sensor_time, double frequency_hz)
  {
    if (!std::isfinite(sensor_time) || !std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
      return false;
    }

    if (!std::isfinite(last_publish_sensor_time_) ||
        sensor_time < last_publish_sensor_time_) {
      last_publish_sensor_time_ = sensor_time;
      return true;
    }

    constexpr double kTimeEpsilon = 1.0e-9;
    const double publish_period = 1.0 / frequency_hz;
    if (sensor_time - last_publish_sensor_time_ + kTimeEpsilon < publish_period) {
      return false;
    }

    last_publish_sensor_time_ = sensor_time;
    return true;
  }

private:
  double last_publish_sensor_time_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace point_lio
