#include <rog_map/field_layer.hpp>

#include <rog_map/esdf_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

void DynamicLayer::update(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const std::vector<uint8_t> & mask,
  double inflation_radius,
  double max_distance,
  double min_distance,
  bool clamp_distance,
  InterpolationMode interpolation,
  FieldBuildStats * stats)
{
  rebuild(width,
    height,
    resolution,
    origin,
    mask,
    inflation_radius,
    max_distance,
    min_distance,
    clamp_distance,
    interpolation,
    stats);
}

void DynamicLayer::rebuild(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const std::vector<uint8_t> & mask,
  double inflation_radius,
  double max_distance,
  double min_distance,
  bool clamp_distance,
  InterpolationMode interpolation,
  FieldBuildStats * stats)
{
  FieldBuildStats local_stats;
  const auto total_start = std::chrono::steady_clock::now();
  // Dynamic field/二维 ESDF 由 ROGMap 提供的最终二维 mask 生成。
  // mask=0 作为障碍源，最终距离会执行 raw_distance - inflation_radius。
  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    throw std::invalid_argument("DynamicLayer::rebuild: invalid grid metadata");
  }

  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (mask.size() != expected) {
    throw std::invalid_argument("DynamicLayer::rebuild: mask size mismatch");
  }

  std::vector<double> dist_sq_pos;
  const auto edt_pos_start = std::chrono::steady_clock::now();
  ESDFUtils::computeEDT2D(width, height, mask, dist_sq_pos);
  local_stats.edt_positive_time_ms = elapsedMs(edt_pos_start);

  const auto inv_start = std::chrono::steady_clock::now();
  std::vector<uint8_t> inv_mask(expected, 0U);
  for (size_t i = 0; i < expected; ++i) {
    inv_mask[i] = (mask[i] == 0U) ? 1U : 0U;
  }
  local_stats.inverse_mask_time_ms = elapsedMs(inv_start);

  std::vector<double> dist_sq_neg;
  const auto edt_neg_start = std::chrono::steady_clock::now();
  ESDFUtils::computeEDT2D(width, height, inv_mask, dist_sq_neg);
  local_stats.edt_negative_time_ms = elapsedMs(edt_neg_start);

  const auto fill_start = std::chrono::steady_clock::now();
  const double max_dist = std::max(0.1, max_distance);
  const double min_dist = std::min(0.0, min_distance);
  std::vector<double> dist_m(expected, max_dist);
  for (size_t i = 0; i < expected; ++i) {
    double raw = 0.0;
    if (mask[i] == 1U) {
      raw = (dist_sq_pos[i] >= 1.0e19) ? kFarDistance : std::sqrt(dist_sq_pos[i]) * resolution;
    } else {
      raw = (dist_sq_neg[i] >= 1.0e19) ? -kFarDistance : -std::sqrt(dist_sq_neg[i]) * resolution;
    }
    const double inflated = raw - inflation_radius;
    dist_m[i] = std::isfinite(inflated) ? inflated : ((mask[i] == 1U) ? kFarDistance : -kFarDistance);
    if (clamp_distance) {
      dist_m[i] = std::clamp(dist_m[i], min_dist, max_dist);
    }
  }
  local_stats.distance_fill_time_ms = elapsedMs(fill_start);

  const auto commit_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_ = origin;
  max_distance_ = max_dist;
  min_distance_ = min_dist;
  clamp_distance_ = clamp_distance;
  interpolation_ = interpolation;
  dist_m_.swap(dist_m);
  local_stats.commit_time_ms = elapsedMs(commit_start);
  local_stats.total_time_ms = elapsedMs(total_start);
  if (stats) {
    *stats = local_stats;
  }
}

bool DynamicLayer::isValid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t expected =
    static_cast<size_t>(std::max(0, width_)) * static_cast<size_t>(std::max(0, height_));
  return width_ > 1 && height_ > 1 && resolution_ > 0.0 && !dist_m_.empty() && dist_m_.size() == expected;
}

bool DynamicLayer::matchesGeometry(
  int width, int height, double resolution, const Eigen::Vector2d & origin) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t expected =
    static_cast<size_t>(std::max(0, width)) * static_cast<size_t>(std::max(0, height));
  return width_ == width && height_ == height && dist_m_.size() == expected &&
         std::abs(resolution_ - resolution) <= 1.0e-9 && (origin_ - origin).norm() <= 1.0e-9;
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

std::vector<double> DynamicLayer::distances() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dist_m_;
}

double DynamicLayer::maxDistance() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_distance_;
}

double DynamicLayer::minDistance() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return min_distance_;
}

bool DynamicLayer::clampDistanceEnabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return clamp_distance_;
}

InterpolationMode DynamicLayer::interpolationMode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return interpolation_;
}

void DynamicLayer::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (width_ <= 1 || height_ <= 1 || resolution_ <= 0.0 || dist_m_.empty()) {
    dist = max_distance_;
    grad.setZero();
    return;
  }

  const double px = (pos.x() - origin_.x()) / resolution_;
  const double py = (pos.y() - origin_.y()) / resolution_;
  const int ix = static_cast<int>(std::floor(px));
  const int iy = static_cast<int>(std::floor(py));
  if (ix < 0 || iy < 0 || ix >= width_ - 1 || iy >= height_ - 1) {
    dist = max_distance_;
    grad.setZero();
    return;
  }

  const double fx = px - static_cast<double>(ix);
  const double fy = py - static_cast<double>(iy);
  const size_t w = static_cast<size_t>(width_);

  if (interpolation_ == InterpolationMode::QUADRATIC && ix >= 1 && iy >= 1 && ix + 1 < width_ &&
      iy + 1 < height_) {
    const auto sample = [this, w](int x, int y) {
      return dist_m_[static_cast<size_t>(y) * w + static_cast<size_t>(x)];
    };
    const double wx[3] = {0.5 * fx * (fx - 1.0), 1.0 - fx * fx, 0.5 * fx * (fx + 1.0)};
    const double wy[3] = {0.5 * fy * (fy - 1.0), 1.0 - fy * fy, 0.5 * fy * (fy + 1.0)};
    const double dwx[3] = {fx - 0.5, -2.0 * fx, fx + 0.5};
    const double dwy[3] = {fy - 0.5, -2.0 * fy, fy + 0.5};

    dist = 0.0;
    double dd_dx_pix = 0.0;
    double dd_dy_pix = 0.0;
    for (int dy = 0; dy < 3; ++dy) {
      for (int dx = 0; dx < 3; ++dx) {
        const double d = sample(ix + dx - 1, iy + dy - 1);
        if (!std::isfinite(d)) {
          dist = max_distance_;
          grad.setZero();
          return;
        }
        dist += wx[dx] * wy[dy] * d;
        dd_dx_pix += dwx[dx] * wy[dy] * d;
        dd_dy_pix += wx[dx] * dwy[dy] * d;
      }
    }
    if (!std::isfinite(dist)) {
      dist = max_distance_;
      grad.setZero();
      return;
    }
    grad.x() = dd_dx_pix / resolution_;
    grad.y() = dd_dy_pix / resolution_;
    grad.z() = 0.0;
    if (!grad.allFinite()) {
      grad.setZero();
    }
    if (clamp_distance_) {
      const double unclamped = dist;
      dist = std::clamp(dist, min_distance_, max_distance_);
      if (unclamped != dist) {
        grad.setZero();
      }
    }
    return;
  }

  const size_t idx00 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix);
  const size_t idx10 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix + 1);
  const size_t idx01 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix);
  const size_t idx11 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix + 1);

  const double d00 = dist_m_[idx00];
  const double d10 = dist_m_[idx10];
  const double d01 = dist_m_[idx01];
  const double d11 = dist_m_[idx11];
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    dist = max_distance_;
    grad.setZero();
    return;
  }

  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;
  if (!std::isfinite(dist)) {
    dist = max_distance_;
    grad.setZero();
    return;
  }

  const double dd_dx_pix = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
  const double dd_dy_pix = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);
  grad.x() = dd_dx_pix / resolution_;
  grad.y() = dd_dy_pix / resolution_;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  if (clamp_distance_) {
    const double unclamped = dist;
    dist = std::clamp(dist, min_distance_, max_distance_);
    if (unclamped != dist) {
      grad.setZero();
    }
  }
}

}  // namespace rog_map
