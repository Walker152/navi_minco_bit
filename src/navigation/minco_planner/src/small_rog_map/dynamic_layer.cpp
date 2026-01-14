#include "small_rog_map/dynamic_layer.hpp"

#include "small_rog_map/esdf_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace small_rog_map
{

DynamicLayer::DynamicLayer() = default;

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

void DynamicLayer::updateFromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double dilation_radius_m,
  bool treat_unknown_as_obstacle)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double res = static_cast<double>(grid.info.resolution);

  if (w <= 0 || h <= 0 || res <= 0.0) {
    throw std::invalid_argument("DynamicLayer::updateFromOccupancyGrid: invalid grid metadata");
  }

  const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (grid.data.size() != expected) {
    throw std::invalid_argument("DynamicLayer::updateFromOccupancyGrid: data size mismatch");
  }

  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;

  // Build obstacle mask (0=obstacle, 1=free)
  std::vector<uint8_t> occ01(expected, 1U);
  std::vector<Eigen::Vector2i> obstacle_cells;
  obstacle_cells.reserve(expected / 32U);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
      const int8_t v = grid.data[idx];

      bool is_obstacle = (v >= 99);
      if (!is_obstacle && treat_unknown_as_obstacle && v < 0) {
        is_obstacle = true;
      }

      if (is_obstacle) {
        occ01[idx] = 0U;
        obstacle_cells.emplace_back(x, y);
      }
    }
  }

  // Dilation
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(dilation_radius_m / res)));
  if (radius_cells > 0 && !obstacle_cells.empty()) {
    std::vector<Eigen::Vector2i> offsets;
    buildDilationOffsets(radius_cells, offsets);

    std::vector<uint8_t> occ01_dilated = occ01;
    for (const auto & c : obstacle_cells) {
      for (const auto & off : offsets) {
        const int nx = c.x() + off.x();
        const int ny = c.y() + off.y();
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        const size_t nidx = static_cast<size_t>(ny) * static_cast<size_t>(w) + static_cast<size_t>(nx);
        occ01_dilated[nidx] = 0U;
      }
    }
    occ01.swap(occ01_dilated);
  }

  // EDT (squared distance in cells^2)
  std::vector<double> dist_sq_cells;
  ESDFUtils::computeEDT2D(w, h, occ01, dist_sq_cells);

  // Convert to meters
  std::vector<double> dist_m(expected, kFarDistance);
  for (size_t i = 0; i < expected; ++i) {
    const double d2 = dist_sq_cells[i];
    if (d2 >= 1.0e19) {
      dist_m[i] = kFarDistance;
      continue;
    }
    dist_m[i] = std::sqrt(d2) * res;
    if (!std::isfinite(dist_m[i]) || dist_m[i] > kFarDistance) {
      dist_m[i] = kFarDistance;
    }
  }

  // Commit under lock
  {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = w;
    height_ = h;
    resolution_ = res;
    origin_ = Eigen::Vector2d(ox, oy);
    dist_m_.swap(dist_m);
  }
}

void DynamicLayer::updateFromCostmap2D(
  nav2_costmap_2d::Costmap2D * costmap,
  double dilation_radius_m,
  bool treat_unknown_as_obstacle)
{
  if (costmap == nullptr) {
    throw std::invalid_argument("DynamicLayer::updateFromCostmap2D: costmap is null");
  }

  const int w = static_cast<int>(costmap->getSizeInCellsX());
  const int h = static_cast<int>(costmap->getSizeInCellsY());
  const double res = static_cast<double>(costmap->getResolution());
  const double ox = static_cast<double>(costmap->getOriginX());
  const double oy = static_cast<double>(costmap->getOriginY());

  if (w <= 0 || h <= 0 || res <= 0.0) {
    throw std::invalid_argument("DynamicLayer::updateFromCostmap2D: invalid costmap metadata");
  }

  const unsigned char * char_map = costmap->getCharMap();
  if (char_map == nullptr) {
    throw std::runtime_error("DynamicLayer::updateFromCostmap2D: getCharMap returned null");
  }

  const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h);

  // Build obstacle mask (0=obstacle, 1=free)
  std::vector<uint8_t> occ01(expected, 1U);
  std::vector<Eigen::Vector2i> obstacle_cells;
  obstacle_cells.reserve(expected / 32U);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
      const unsigned char c = char_map[idx];

      bool is_obstacle =
        (c == nav2_costmap_2d::LETHAL_OBSTACLE) ||
        (c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

      if (!is_obstacle && c == nav2_costmap_2d::NO_INFORMATION && treat_unknown_as_obstacle) {
        is_obstacle = true;
      }

      if (is_obstacle) {
        occ01[idx] = 0U;
        obstacle_cells.emplace_back(x, y);
      }
    }
  }

  // Dilation
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(dilation_radius_m / res)));
  if (radius_cells > 0 && !obstacle_cells.empty()) {
    std::vector<Eigen::Vector2i> offsets;
    buildDilationOffsets(radius_cells, offsets);

    std::vector<uint8_t> occ01_dilated = occ01;
    for (const auto & c : obstacle_cells) {
      for (const auto & off : offsets) {
        const int nx = c.x() + off.x();
        const int ny = c.y() + off.y();
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
          continue;
        }
        const size_t nidx = static_cast<size_t>(ny) * static_cast<size_t>(w) + static_cast<size_t>(nx);
        occ01_dilated[nidx] = 0U;
      }
    }
    occ01.swap(occ01_dilated);
  }

  // EDT (squared distance in cells^2)
  std::vector<double> dist_sq_cells;
  ESDFUtils::computeEDT2D(w, h, occ01, dist_sq_cells);

  // Convert to meters
  std::vector<double> dist_m(expected, kFarDistance);
  for (size_t i = 0; i < expected; ++i) {
    const double d2 = dist_sq_cells[i];
    if (d2 >= 1.0e19) {
      dist_m[i] = kFarDistance;
      continue;
    }
    dist_m[i] = std::sqrt(d2) * res;
    if (!std::isfinite(dist_m[i]) || dist_m[i] > kFarDistance) {
      dist_m[i] = kFarDistance;
    }
  }

  // Commit under lock
  {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = w;
    height_ = h;
    resolution_ = res;
    origin_ = Eigen::Vector2d(ox, oy);
    dist_m_.swap(dist_m);
  }
}

void DynamicLayer::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (width_ <= 1 || height_ <= 1 || resolution_ <= 0.0 || dist_m_.empty()) {
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
