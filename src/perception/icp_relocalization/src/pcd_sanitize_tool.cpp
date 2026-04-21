#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

double percentile(std::vector<double> values, double p)
{
  if (values.empty()) {
    return 0.0;
  }
  p = std::clamp(p, 0.0, 100.0);
  const double idx_f = (p / 100.0) * static_cast<double>(values.size() - 1);
  const auto idx_lo = static_cast<std::size_t>(std::floor(idx_f));
  const auto idx_hi = static_cast<std::size_t>(std::ceil(idx_f));

  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx_lo), values.end());
  const double v_lo = values[idx_lo];
  if (idx_hi == idx_lo) {
    return v_lo;
  }

  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx_hi), values.end());
  const double v_hi = values[idx_hi];
  const double alpha = idx_f - static_cast<double>(idx_lo);
  return v_lo + (v_hi - v_lo) * alpha;
}

struct Bounds
{
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  double max_r = 0.0;
};

Bounds computeRobustBounds(const PointCloud::Ptr & cloud)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(cloud->size());
  ys.reserve(cloud->size());
  zs.reserve(cloud->size());

  for (const auto & p : cloud->points) {
    xs.push_back(p.x);
    ys.push_back(p.y);
    zs.push_back(p.z);
  }

  const double qx_lo = percentile(xs, 0.01);
  const double qx_hi = percentile(xs, 99.99);
  const double qy_lo = percentile(ys, 0.01);
  const double qy_hi = percentile(ys, 99.99);
  const double qz_lo = percentile(zs, 0.01);
  const double qz_hi = percentile(zs, 99.99);

  const double span_x = std::max(0.0, qx_hi - qx_lo);
  const double span_y = std::max(0.0, qy_hi - qy_lo);
  const double span_z = std::max(0.0, qz_hi - qz_lo);

  const double margin_x = std::max(0.2, 0.03 * span_x);
  const double margin_y = std::max(0.2, 0.03 * span_y);
  const double margin_z = std::max(0.2, 0.05 * span_z);

  const double cx = 0.5 * (qx_lo + qx_hi);
  const double cy = 0.5 * (qy_lo + qy_hi);
  std::vector<double> rs;
  rs.reserve(cloud->size());
  for (const auto & p : cloud->points) {
    rs.push_back(std::hypot(static_cast<double>(p.x) - cx, static_cast<double>(p.y) - cy));
  }

  const double r_hi = percentile(rs, 99.99);

  Bounds b;
  b.min_x = qx_lo - margin_x;
  b.max_x = qx_hi + margin_x;
  b.min_y = qy_lo - margin_y;
  b.max_y = qy_hi + margin_y;
  b.min_z = qz_lo - margin_z;
  b.max_z = qz_hi + margin_z;
  b.max_r = r_hi + std::max(0.5, 0.02 * r_hi);
  return b;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: pcd_sanitize_tool <input.pcd> <output.pcd> [--scale <factor>]\n";
    return 1;
  }

  const std::string input_path = argv[1];
  const std::string output_path = argv[2];
  double scale = 1.0;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scale") {
      if (i + 1 >= argc) {
        std::cerr << "[pcd_sanitize_tool] Missing value for --scale\n";
        return 1;
      }
      try {
        scale = std::stod(argv[++i]);
      } catch (const std::exception &) {
        std::cerr << "[pcd_sanitize_tool] Invalid --scale value\n";
        return 1;
      }
    }
  }
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    std::cerr << "[pcd_sanitize_tool] --scale must be a positive finite number\n";
    return 1;
  }

  PointCloud::Ptr cloud_raw(new PointCloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *cloud_raw) != 0) {
    std::cerr << "[pcd_sanitize_tool] Failed to load: " << input_path << "\n";
    return 2;
  }

  pcl::Indices indices;
  PointCloud::Ptr cloud_finite(new PointCloud());
  pcl::removeNaNFromPointCloud(*cloud_raw, *cloud_finite, indices);

  if (!cloud_finite || cloud_finite->empty()) {
    std::cerr << "[pcd_sanitize_tool] No valid points after NaN removal.\n";
    return 3;
  }

  if (std::abs(scale - 1.0) > 1.0e-12) {
    for (auto & p : cloud_finite->points) {
      p.x = static_cast<float>(static_cast<double>(p.x) * scale);
      p.y = static_cast<float>(static_cast<double>(p.y) * scale);
      p.z = static_cast<float>(static_cast<double>(p.z) * scale);
    }
  }

  const auto bounds = computeRobustBounds(cloud_finite);

  PointCloud::Ptr cloud_clean(new PointCloud());
  cloud_clean->reserve(cloud_finite->size());
  const double cx = 0.5 * (bounds.min_x + bounds.max_x);
  const double cy = 0.5 * (bounds.min_y + bounds.max_y);
  for (const auto & p : cloud_finite->points) {
    const bool in_box = p.x >= bounds.min_x && p.x <= bounds.max_x && p.y >= bounds.min_y &&
                        p.y <= bounds.max_y && p.z >= bounds.min_z && p.z <= bounds.max_z;
    if (!in_box) {
      continue;
    }
    const double r = std::hypot(static_cast<double>(p.x) - cx, static_cast<double>(p.y) - cy);
    if (r <= bounds.max_r) {
      cloud_clean->points.push_back(p);
    }
  }

  cloud_clean->width = static_cast<std::uint32_t>(cloud_clean->points.size());
  cloud_clean->height = 1;
  cloud_clean->is_dense = true;

  if (cloud_clean->empty()) {
    std::cerr << "[pcd_sanitize_tool] Cleaned cloud is empty, aborting save.\n";
    return 4;
  }

  if (pcl::io::savePCDFileBinaryCompressed(output_path, *cloud_clean) != 0) {
    std::cerr << "[pcd_sanitize_tool] Failed to save: " << output_path << "\n";
    return 5;
  }

  std::cout << "[pcd_sanitize_tool] Input points : " << cloud_raw->size() << "\n";
  std::cout << "[pcd_sanitize_tool] Finite points: " << cloud_finite->size() << "\n";
  std::cout << "[pcd_sanitize_tool] Scale factor : " << scale << "\n";
  std::cout << "[pcd_sanitize_tool] Output points: " << cloud_clean->size() << "\n";
  std::cout << "[pcd_sanitize_tool] Bounds x:[" << bounds.min_x << ", " << bounds.max_x << "] y:["
            << bounds.min_y << ", " << bounds.max_y << "] z:[" << bounds.min_z << ", " << bounds.max_z
            << "] r<= " << bounds.max_r << "\n";
  std::cout << "[pcd_sanitize_tool] Saved to " << output_path << "\n";

  return 0;
}
