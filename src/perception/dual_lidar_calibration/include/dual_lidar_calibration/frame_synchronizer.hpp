#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dual_lidar_calibration {

struct TimedFrame
{
  std::int64_t stamp_ns{0};
  std::size_t source_index{0U};
};

struct FramePair
{
  std::size_t main_index{0U};
  std::size_t secondary_index{0U};
  std::int64_t delta_ns{0};
};

// Inputs must be sorted by stamp_ns. Each input frame is used at most once.
// delta_ns is secondary_stamp - main_stamp.
std::vector<FramePair> pairNearestFrames(const std::vector<TimedFrame> & main_frames,
  const std::vector<TimedFrame> & secondary_frames,
  std::int64_t tolerance_ns);

}  // namespace dual_lidar_calibration
