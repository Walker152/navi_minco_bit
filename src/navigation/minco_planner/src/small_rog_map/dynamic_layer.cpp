#include "small_rog_map/dynamic_layer.hpp"

#include "small_rog_map/esdf_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <string>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace small_rog_map
{

namespace
{
struct PointXYZ
{
  float x;
  float y;
  float z;
};
}  // namespace

DynamicLayer::DynamicLayer() = default;

void DynamicLayer::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::string & topic,
  double resolution,
  double local_size_m,
  double dilation_radius_m)
{
  node_ = node;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resolution_ = resolution;
    local_size_m_ = local_size_m;
    dilation_radius_m_ = dilation_radius_m;
  }

  auto node_ptr = node_.lock();
  if (!node_ptr) {
    cloud_sub_.reset();
    return;
  }

  cloud_sub_ = node_ptr->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic,
    rclcpp::SensorDataQoS(),
    std::bind(&DynamicLayer::cloudCallback, this, std::placeholders::_1));
}

void DynamicLayer::setRobotPosition(double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robot_pos_ = Eigen::Vector2d(x, y);
}

void DynamicLayer::setGeometry(int w, int h, double res, const Eigen::Vector2d & origin)
{
  if (w <= 0 || h <= 0 || res <= 0.0) {
    return;
  }

  const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h);
  std::lock_guard<std::mutex> lock(mutex_);
  width_ = w;
  height_ = h;
  resolution_ = res;
  origin_ = origin;
  dist_m_.assign(expected, kFarDistance);
}

void DynamicLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  double res = 0.0;
  double local_size_m = 0.0;
  double dilation_radius_m = 0.0;
  Eigen::Vector2d robot_pos(0.0, 0.0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    res = resolution_;
    local_size_m = local_size_m_;
    dilation_radius_m = dilation_radius_m_;
    robot_pos = robot_pos_;
  }

  if (res <= 0.0 || local_size_m <= 0.0) {
    return;
  }

  const int w = static_cast<int>(std::ceil(local_size_m / res));
  if (w <= 0) {
    return;
  }

  const Eigen::Vector2d origin = robot_pos - Eigen::Vector2d(local_size_m / 2.0, local_size_m / 2.0);

  // Update the ESDF from the point cloud in a robot-centered local sliding window.
  updateFromPointCloud(*msg, w, w, res, origin, dilation_radius_m);
}

bool DynamicLayer::isValid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return width_ > 1 && height_ > 1 && resolution_ > 0.0 && !dist_m_.empty();
}

int DynamicLayer::width() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return width_;
}

int DynamicLayer::height() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return height_;
}

double DynamicLayer::resolution() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return resolution_;
}

Eigen::Vector2d DynamicLayer::origin() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return origin_;
}

bool DynamicLayer::isInside(const Eigen::Vector2d & pos_xy) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (width_ <= 1 || height_ <= 1 || resolution_ <= 0.0) {
    return false;
  }
  const double px = (pos_xy.x() - origin_.x()) / resolution_;
  const double py = (pos_xy.y() - origin_.y()) / resolution_;
  return px >= 0.0 && py >= 0.0 && px < static_cast<double>(width_ - 1) && py < static_cast<double>(height_ - 1);
}

void DynamicLayer::buildDilationOffsets(int radius_cells, std::vector<Eigen::Vector2i> & offsets) const
{
  offsets.clear();
  if (radius_cells <= 0) {
    offsets.emplace_back(0, 0);
    return;
  }
  const int r2 = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy <= r2) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
}

void DynamicLayer::updateFromPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  double dilation_radius_m)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    throw std::invalid_argument("DynamicLayer::updateFromPointCloud: invalid grid metadata");
  }

  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  // 1. Build a 2D occupancy mask
  // 0 = obstacle, 1 = free
  // [Theory] A 2D ESDF can be computed by an Euclidean Distance Transform (EDT) on this mask.
  std::vector<uint8_t> occ01(expected, 1U);

  // 2. Mark obstacle cells from sparse point cloud hits
  // Find x/y fields by name
  if (!cloud.is_bigendian && cloud.point_step > 0U && !cloud.data.empty()) {
    auto find_field = [&cloud](const char * name) -> const sensor_msgs::msg::PointField * {
      for (const auto & f : cloud.fields) {
        if (f.name == name) {
          return &f;
        }
      }
      return nullptr;
    };

    const auto * fx = find_field("x");
    const auto * fy = find_field("y");
    const bool fields_ok =
      fx && fy && fx->datatype == sensor_msgs::msg::PointField::FLOAT32 && fy->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      fx->count == 1U && fy->count == 1U &&
      (static_cast<size_t>(fx->offset) + sizeof(float) <= static_cast<size_t>(cloud.point_step)) &&
      (static_cast<size_t>(fy->offset) + sizeof(float) <= static_cast<size_t>(cloud.point_step));

    if (fields_ok) {
      const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
      const size_t stride = static_cast<size_t>(cloud.point_step);
      const size_t max_points_by_bytes = cloud.data.size() / stride;
      const size_t n = std::min(point_count, max_points_by_bytes);

      const bool is_standard_xyz_layout =
        fx->offset == static_cast<uint32_t>(offsetof(PointXYZ, x)) &&
        fy->offset == static_cast<uint32_t>(offsetof(PointXYZ, y)) &&
        stride >= sizeof(PointXYZ);

      if (is_standard_xyz_layout) {
        const uint8_t * p = cloud.data.data();
        for (size_t i = 0; i < n; ++i, p += stride) {
          const auto * pt = reinterpret_cast<const PointXYZ *>(p);
          const float x = pt->x;
          const float y = pt->y;
          if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
          }

          const int ix = static_cast<int>(std::floor((static_cast<double>(x) - origin.x()) / resolution));
          const int iy = static_cast<int>(std::floor((static_cast<double>(y) - origin.y()) / resolution));
          if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
            continue;
          }
          const size_t idx = static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix);
          occ01[idx] = 0U;
        }
      } else {
        for (size_t i = 0; i < n; ++i) {
          const uint8_t * p = &cloud.data[i * stride];
          float x = 0.0f;
          float y = 0.0f;
          std::memcpy(&x, p + fx->offset, sizeof(float));
          std::memcpy(&y, p + fy->offset, sizeof(float));
          if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
          }

          const int ix = static_cast<int>(std::floor((static_cast<double>(x) - origin.x()) / resolution));
          const int iy = static_cast<int>(std::floor((static_cast<double>(y) - origin.y()) / resolution));
          if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
            continue;
          }
          const size_t idx = static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix);
          occ01[idx] = 0U;
        }
      }
    }
  }

  // 3. Run EDT directly on the original occupancy mask.
  std::vector<double> dist_sq_cells;
  ESDFUtils::computeEDT2D(width, height, occ01, dist_sq_cells);

  std::vector<double> dist_m(expected, kFarDistance);
#ifdef _OPENMP
  #pragma omp parallel for
#endif
  for (size_t i = 0; i < expected; ++i) {
    if (occ01[i] == 0U) {
      dist_m[i] = kESDFStrength;
      continue;
    }
    const double d2 = dist_sq_cells[i];
    if (d2 >= 1.0e19) {
      dist_m[i] = kFarDistance;
      continue;
    }
    const double d_raw = std::sqrt(d2) * resolution;
    const double d_dilated = d_raw - dilation_radius_m;
    if (!std::isfinite(d_dilated)) {
      dist_m[i] = kFarDistance;
      continue;
    }
    if (d_dilated <= 0.0) {
      dist_m[i] = kESDFStrength;
      continue;
    }
    dist_m[i] = d_dilated;
    if (dist_m[i] > kFarDistance) {
      dist_m[i] = kFarDistance;
    }
  }

  // 4. Commit the new distance field
  {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    dist_m_.swap(dist_m);
  }
}

void DynamicLayer::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 1. Check validity
  if (width_ <= 1 || height_ <= 1 || resolution_ <= 0.0 || dist_m_.empty()) {
    dist = kFarDistance;
    grad.setZero();
    return;
  }

  // 2. Convert world coords to grid coords
  const double px = (pos.x() - origin_.x()) / resolution_;
  const double py = (pos.y() - origin_.y()) / resolution_;

  const int ix = static_cast<int>(std::floor(px));
  const int iy = static_cast<int>(std::floor(py));

  if (ix < 0 || iy < 0 || ix >= width_ - 1 || iy >= height_ - 1) {
    dist = kFarDistance;
    grad.setZero();
    return;
  }

  const double fx = px - static_cast<double>(ix);
  const double fy = py - static_cast<double>(iy);

  const size_t w = static_cast<size_t>(width_);
  const size_t idx00 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix);
  const size_t idx10 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix + 1);
  const size_t idx01 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix);
  const size_t idx11 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix + 1);

  const double d00 = dist_m_[idx00];
  const double d10 = dist_m_[idx10];
  const double d01 = dist_m_[idx01];
  const double d11 = dist_m_[idx11];

  // If any neighbor is marked as a hard obstacle (-inf), treat this query as inside obstacle.
  // This prevents bilinear interpolation from producing NaNs.
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    dist = kESDFStrength;
    grad.setZero();
    return;
  }

  // 3. Bilinear interpolation for distance
  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;

  const double dd_dx_pix = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
  const double dd_dy_pix = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);

  // 4. Finite-difference gradient (meters)
  grad.x() = dd_dx_pix / resolution_;
  grad.y() = dd_dy_pix / resolution_;
  grad.z() = 0.0;
}

}  // namespace small_rog_map
