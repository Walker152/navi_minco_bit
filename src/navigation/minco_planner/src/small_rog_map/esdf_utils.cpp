#include "small_rog_map/esdf_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace small_rog_map
{

namespace
{
constexpr double kInf = 1.0e20;

inline bool isValidSize(int width, int height)
{
  return width > 0 && height > 0;
}

inline void computeEDT1DNoAlloc(
  const std::vector<double> & f,
  std::vector<double> & d,
  std::vector<int> & v,
  std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  std::fill(d.begin(), d.end(), kInf);

  if (n <= 0) {
    return;
  }

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int p = v[static_cast<size_t>(k)];
      const double fq = f[static_cast<size_t>(q)];
      const double fp = f[static_cast<size_t>(p)];
      // If both are INF, s becomes NaN; skip q in that case.
      if (fq >= kInf && fp >= kInf) {
        s = kInf;
      } else {
        s = ((fq + static_cast<double>(q * q)) - (fp + static_cast<double>(p * p))) /
          (2.0 * static_cast<double>(q - p));
      }

      if (k <= 0 || s > z[static_cast<size_t>(k)]) {
        break;
      }
      --k;
    }

    ++k;
    v[static_cast<size_t>(k)] = q;
    z[static_cast<size_t>(k)] = s;
    z[static_cast<size_t>(k) + 1] = kInf;
  }

  int kk = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<size_t>(kk) + 1] < static_cast<double>(q)) {
      ++kk;
    }
    const int p = v[static_cast<size_t>(kk)];
    const double val = static_cast<double>((q - p) * (q - p)) + f[static_cast<size_t>(p)];
    d[static_cast<size_t>(q)] = val;
  }
}
}  // namespace

void ESDFUtils::computeEDT1D(const std::vector<double> & f, std::vector<double> & d)
{
  const int n = static_cast<int>(f.size());
  d.assign(static_cast<size_t>(n), kInf);

  if (n <= 0) {
    return;
  }

  std::vector<int> v(static_cast<size_t>(n));
  std::vector<double> z(static_cast<size_t>(n) + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int p = v[static_cast<size_t>(k)];
      const double fq = f[static_cast<size_t>(q)];
      const double fp = f[static_cast<size_t>(p)];
      // If both are INF, s becomes NaN; skip q in that case.
      if (fq >= kInf && fp >= kInf) {
        s = kInf;
      } else {
        s = ((fq + static_cast<double>(q * q)) - (fp + static_cast<double>(p * p))) /
          (2.0 * static_cast<double>(q - p));
      }

      if (k <= 0 || s > z[static_cast<size_t>(k)]) {
        break;
      }
      --k;
    }

    ++k;
    v[static_cast<size_t>(k)] = q;
    z[static_cast<size_t>(k)] = s;
    z[static_cast<size_t>(k) + 1] = kInf;
  }

  int kk = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<size_t>(kk) + 1] < static_cast<double>(q)) {
      ++kk;
    }
    const int p = v[static_cast<size_t>(kk)];
    const double val = static_cast<double>((q - p) * (q - p)) + f[static_cast<size_t>(p)];
    d[static_cast<size_t>(q)] = val;
  }
}

void ESDFUtils::computeEDT2D(
  int width,
  int height,
  const std::vector<uint8_t> & occupancy_01,
  std::vector<double> & dist_sq_out)
{
  if (!isValidSize(width, height)) {
    throw std::invalid_argument("ESDFUtils::computeEDT2D: invalid width/height");
  }
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (occupancy_01.size() != expected) {
    throw std::invalid_argument("ESDFUtils::computeEDT2D: occupancy size mismatch");
  }

  // First pass: rows
  std::vector<double> tmp(expected, kInf);

  #pragma omp parallel
  {
    std::vector<double> f_row(static_cast<size_t>(width));
    std::vector<double> d_row(static_cast<size_t>(width));
    std::vector<int> v_row(static_cast<size_t>(width));
    std::vector<double> z_row(static_cast<size_t>(width) + 1);

    #pragma omp for
    for (int y = 0; y < height; ++y) {
      const size_t row_base = static_cast<size_t>(y) * static_cast<size_t>(width);
      for (int x = 0; x < width; ++x) {
        const uint8_t occ = occupancy_01[row_base + static_cast<size_t>(x)];
        f_row[static_cast<size_t>(x)] = (occ == 0U) ? 0.0 : kInf;
      }
      computeEDT1DNoAlloc(f_row, d_row, v_row, z_row);
      for (int x = 0; x < width; ++x) {
        tmp[row_base + static_cast<size_t>(x)] = d_row[static_cast<size_t>(x)];
      }
    }
  }

  // Second pass: cols
  dist_sq_out.assign(expected, kInf);

  #pragma omp parallel
  {
    std::vector<double> f_col(static_cast<size_t>(height));
    std::vector<double> d_col(static_cast<size_t>(height));
    std::vector<int> v_col(static_cast<size_t>(height));
    std::vector<double> z_col(static_cast<size_t>(height) + 1);

    #pragma omp for
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        f_col[static_cast<size_t>(y)] = tmp[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)];
      }
      computeEDT1DNoAlloc(f_col, d_col, v_col, z_col);
      for (int y = 0; y < height; ++y) {
        dist_sq_out[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] = d_col[static_cast<size_t>(y)];
      }
    }
  }
}

}  // namespace small_rog_map
