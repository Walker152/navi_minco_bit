#pragma once

#include <cstdint>
#include <vector>

namespace small_rog_map {

class ESDFUtils
{
public:
  // Input: occupancy_01 (0=obstacle, 1=free). Output: squared distance (in cells^2).
  static void computeEDT2D(
    int width, int height, const std::vector<uint8_t> & occupancy_01, std::vector<double> & dist_sq_out);

private:
  static void computeEDT1D(const std::vector<double> & f, std::vector<double> & d);
};

}  // namespace small_rog_map
