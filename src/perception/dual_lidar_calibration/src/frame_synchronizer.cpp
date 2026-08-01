#include "dual_lidar_calibration/frame_synchronizer.hpp"

#include <cstdint>
#include <limits>

namespace dual_lidar_calibration {

std::vector<FramePair> pairNearestFrames(const std::vector<TimedFrame> & main_frames,
  const std::vector<TimedFrame> & secondary_frames,
  const std::int64_t tolerance_ns)
{
  std::vector<FramePair> pairs;
  if (tolerance_ns < 0) {
    return pairs;
  }

  std::size_t secondary_pos = 0U;
  for (std::size_t main_pos = 0U;
       main_pos < main_frames.size() && secondary_pos < secondary_frames.size(); ++main_pos) {
    while (secondary_pos < secondary_frames.size() &&
           secondary_frames[secondary_pos].stamp_ns - main_frames[main_pos].stamp_ns <
             -tolerance_ns) {
      ++secondary_pos;
    }
    if (secondary_pos >= secondary_frames.size()) {
      break;
    }

    std::size_t best_pos = secondary_frames.size();
    std::int64_t best_abs_delta = std::numeric_limits<std::int64_t>::max();
    for (std::size_t candidate = secondary_pos; candidate < secondary_frames.size(); ++candidate) {
      const std::int64_t delta =
        secondary_frames[candidate].stamp_ns - main_frames[main_pos].stamp_ns;
      if (delta > tolerance_ns) {
        break;
      }
      const std::int64_t absolute_delta = delta < 0 ? -delta : delta;
      if (absolute_delta < best_abs_delta) {
        best_abs_delta = absolute_delta;
        best_pos = candidate;
      }
    }
    if (best_pos == secondary_frames.size()) {
      continue;
    }

    pairs.push_back(FramePair{main_frames[main_pos].source_index,
      secondary_frames[best_pos].source_index,
      secondary_frames[best_pos].stamp_ns - main_frames[main_pos].stamp_ns});
    secondary_pos = best_pos + 1U;
  }
  return pairs;
}

}  // namespace dual_lidar_calibration
