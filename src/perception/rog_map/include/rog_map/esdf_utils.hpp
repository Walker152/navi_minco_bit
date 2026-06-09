#pragma once

#include <cstdint>
#include <vector>

namespace rog_map {

class ESDFUtils
{
public:
  // Input mask: 0 = target/obstacle seed, 1 = free/non-target.
  // Output is squared distance in grid cells.
  static void computeEDT2D(
    int width, int height, const std::vector<uint8_t> & mask, std::vector<double> & dist_sq_out);

private:
  static void computeEDT1D(const std::vector<double> & f, std::vector<double> & d);
};

}  // namespace rog_map
