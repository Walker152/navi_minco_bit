#include "small_rog_map/static_layer.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <limits>

namespace small_rog_map
{

StaticLayer::StaticLayer() = default;

bool StaticLayer::isValid() const
{
  return width_ > 1 && height_ > 1 && !dist_m_.empty() && resolution_ > 0.0;
}

int StaticLayer::width() const { return width_; }
int StaticLayer::height() const { return height_; }
double StaticLayer::resolution() const { return resolution_; }
const Eigen::Vector2d & StaticLayer::origin() const { return origin_; }
const std::vector<double> & StaticLayer::distanceBuffer() const { return dist_m_; }

bool StaticLayer::loadFromPCD(const std::string & pcd_path, double resolution)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1) {
    return false;
  }
  if (!cloud || cloud->empty()) {
    return false;
  }
  if (resolution <= 0.0) {
    return false;
  }

  resolution_ = resolution;

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto & pt : cloud->points) {
    min_x = std::min(min_x, static_cast<double>(pt.x));
    max_x = std::max(max_x, static_cast<double>(pt.x));
    min_y = std::min(min_y, static_cast<double>(pt.y));
    max_y = std::max(max_y, static_cast<double>(pt.y));
  }

  origin_ = Eigen::Vector2d(min_x, min_y);

  width_ = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
  height_ = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;

  if (width_ <= 1 || height_ <= 1) {
    dist_m_.clear();
    width_ = 0;
    height_ = 0;
    return false;
  }

  dist_m_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), kFarDistance);

  for (const auto & pt : cloud->points) {
    const int ix = static_cast<int>(std::lround((static_cast<double>(pt.x) - min_x) / resolution_));
    const int iy = static_cast<int>(std::lround((static_cast<double>(pt.y) - min_y) / resolution_));
    if (ix >= 0 && ix < width_ && iy >= 0 && iy < height_) {
      dist_m_[static_cast<size_t>(iy) * static_cast<size_t>(width_) + static_cast<size_t>(ix)] =
        static_cast<double>(pt.intensity);
    }
  }

  return true;
}

void StaticLayer::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  if (!isValid()) {
    dist = kFarDistance;
    grad.setZero();
    return;
  }

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

  // 避免 0 * (-inf) 在双线性插值中产生 NaN，导致障碍附近被误判为远距离。
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    dist = -std::numeric_limits<double>::infinity();
    grad.setZero();
    return;
  }

  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;

  const double dd_dx_pix = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
  const double dd_dy_pix = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);

  grad.x() = dd_dx_pix / resolution_;
  grad.y() = dd_dy_pix / resolution_;
  grad.z() = 0.0;
}

}  // namespace small_rog_map
