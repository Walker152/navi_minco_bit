#pragma once

#include <cstddef>
#include <vector>

namespace point_lio
{

/**
 * @brief Group time-ordered points into EKF update batches.
 *
 * A non-positive time_bin_ms preserves the original behavior: only points with
 * exactly the same timestamp share an update. A positive value groups adjacent
 * points while their timestamp span remains smaller than the configured bin.
 * The returned values are point counts, so every input point is retained.
 */
template <typename TimestampAt>
std::vector<int> build_time_batches(
  std::size_t point_count, double time_bin_ms, TimestampAt timestamp_at)
{
  std::vector<int> batch_sizes;
  if (point_count == 0) {
    return batch_sizes;
  }

  batch_sizes.reserve(point_count);
  std::size_t batch_begin = 0;
  double batch_begin_stamp = timestamp_at(0);
  double previous_stamp = batch_begin_stamp;

  for (std::size_t index = 1; index < point_count; ++index) {
    const double current_stamp = timestamp_at(index);
    const bool start_new_batch =
      time_bin_ms > 0.0
        ? current_stamp - batch_begin_stamp >= time_bin_ms
        : current_stamp > previous_stamp;

    if (start_new_batch) {
      batch_sizes.emplace_back(static_cast<int>(index - batch_begin));
      batch_begin = index;
      batch_begin_stamp = current_stamp;
    }
    previous_stamp = current_stamp;
  }

  batch_sizes.emplace_back(static_cast<int>(point_count - batch_begin));
  return batch_sizes;
}

}  // namespace point_lio
