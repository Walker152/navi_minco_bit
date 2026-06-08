#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <cmath>

namespace rog_map {

namespace {

bool finiteOrNan(double value)
{
  return std::isfinite(value) || std::isnan(value);
}

CellType classifyCell(
  const ColumnStats & stats, CellData & cell, const ProjectionLayerConfig & config, double resolution)
{
  const int min_observed = std::max(1, config.min_observed_voxels);
  cell.confidence = static_cast<float>(
    std::clamp(static_cast<double>(stats.observed) / static_cast<double>(min_observed), 0.0, 1.0));

  if (stats.occupied > 0 && std::isfinite(stats.min_z) && std::isfinite(stats.max_z)) {
    cell.min_z = static_cast<float>(stats.min_z);
    cell.max_z = static_cast<float>(stats.max_z);
    cell.height = static_cast<float>(std::max(0.0, stats.max_z - stats.min_z));
    cell.ratio = static_cast<float>((static_cast<double>(stats.occupied + 1) * resolution) /
                                    std::max(static_cast<double>(cell.height), resolution));
  }
  cell.ground_z = static_cast<float>(stats.ground_z);
  cell.ceiling_z = static_cast<float>(stats.ceiling_z);

  if (stats.observed < min_observed) {
    return CellType::UNKNOWN;
  }
  if (stats.occupied == 0) {
    return CellType::FREE;
  }

  if (config.terrain_enable) {
    const double body_z_min = std::min(config.robot_body_z_min, config.robot_body_z_max);
    const double body_z_max = std::max(config.robot_body_z_min, config.robot_body_z_max);
    const double body_clearance_top = body_z_max + config.overhead_clearance_margin;
    const bool body_band_blocked = stats.occupied_in_body_band > 0;
    const bool has_ground = std::isfinite(stats.ground_z);
    const bool ceiling_blocks = config.clearance_check_enable && std::isfinite(stats.ceiling_z) &&
                                stats.ceiling_z < body_z_max + config.min_clearance_height;
    const bool slope_ok = static_cast<double>(cell.slope_deg) <= config.max_slope_deg;
    const bool step_ok = static_cast<double>(cell.step_height) <= config.max_step_height;
    const bool surface_ok =
      !has_ground || (static_cast<double>(cell.height) <= std::max(config.surface_thickness, resolution) ||
                       stats.occupied_below_body > 0);
    const bool vertical_wall = static_cast<double>(cell.height) > config.tunnel_wall_min_height &&
                               static_cast<double>(cell.ratio) > config.min_ratio && body_band_blocked;

    if (!body_band_blocked && !ceiling_blocks && slope_ok && step_ok && surface_ok) {
      cell.traversable = 1U;
      return CellType::PASSABLE;
    }

    if (body_band_blocked) {
      const double body_band_height =
        std::isfinite(stats.body_band_min_z) && std::isfinite(stats.body_band_max_z)
          ? stats.body_band_max_z - stats.body_band_min_z
          : static_cast<double>(cell.height);
      const bool crossable_body_band = body_band_height <= std::max(config.surface_thickness, resolution) &&
                                       static_cast<double>(cell.step_height) <= config.max_step_height &&
                                       !vertical_wall;
      if (crossable_body_band && !ceiling_blocks && slope_ok) {
        cell.traversable = 1U;
        return CellType::PASSABLE;
      }
    }

    if (vertical_wall || ceiling_blocks || body_band_blocked) {
      cell.traversable = 0U;
      return CellType::OCCUPIED;
    }

    if (stats.occupied_above_body > 0 && !ceiling_blocks &&
        (!std::isfinite(stats.ceiling_z) || stats.ceiling_z >= body_clearance_top)) {
      cell.traversable = 1U;
      return CellType::FREE;
    }
  }

  if (cell.height <= config.low_obstacle_height) {
    return CellType::PASSABLE;
  }
  if (cell.height > config.obstacle_height && cell.ratio > config.min_ratio) {
    return CellType::OCCUPIED;
  }
  return CellType::OCCUPIED;
}

}  // namespace

void ProjectionLayer::update(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner)
{
  updateFull(width, height, resolution, origin, config, scanner);
}

void ProjectionLayer::updateFull(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 || !scanner) {
    width_ = 0;
    height_ = 0;
    resolution_ = 0.0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    return;
  }

  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
  const bool geometry_changed = width_ != width || height_ != height ||
                                std::abs(resolution_ - resolution) > 1.0e-9 ||
                                (origin_ - origin).norm() > 1.0e-9 || cells_.size() != expected;

  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_ = origin;

  std::vector<CellData> previous;
  if (!geometry_changed) {
    previous = cells_;
  }

  cells_.assign(expected, CellData{});
  values_.assign(expected, 255U);
  mask_.assign(expected, config.unknown_as_occupied ? 0U : 1U);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
      updateOneCell(idx, x, y, config, scanner, previous);
    }
  }

  updateTerrainNeighborhood(config, nullptr);
  applyHoleFill(config, nullptr);
  for (size_t idx = 0; idx < cells_.size(); ++idx) {
    applyValueAndMask(cells_[idx], config);
    values_[idx] = cells_[idx].value;
    mask_[idx] = cells_[idx].mask;
  }
}

void ProjectionLayer::updateDirty(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<int> & dirty_columns,
  bool force_full_refresh)
{
  const bool geometry_changed = !matchesGeometry(width, height, resolution, origin);
  if (force_full_refresh || geometry_changed || cells_.empty()) {
    updateFull(width, height, resolution, origin, config, scanner);
    return;
  }
  if (dirty_columns.empty()) {
    return;
  }

  const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::vector<uint8_t> update_mask(expected, 0U);
  const int dirty_radius = std::max(1, config.hole_fill_radius + 1);
  std::vector<int> expanded;
  expanded.reserve(
    dirty_columns.size() * static_cast<size_t>((2 * dirty_radius + 1) * (2 * dirty_radius + 1)));
  for (const int column_id : dirty_columns) {
    if (column_id < 0 || column_id >= static_cast<int>(expected)) {
      continue;
    }
    const int cx = column_id % width_;
    const int cy = column_id / width_;
    for (int dy = -dirty_radius; dy <= dirty_radius; ++dy) {
      for (int dx = -dirty_radius; dx <= dirty_radius; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
          continue;
        }
        const int nidx = ny * width_ + nx;
        if (!update_mask[static_cast<size_t>(nidx)]) {
          update_mask[static_cast<size_t>(nidx)] = 1U;
          expanded.push_back(nidx);
        }
      }
    }
  }

  const auto previous = cells_;
  for (const int idx_int : expanded) {
    const size_t idx = static_cast<size_t>(idx_int);
    updateOneCell(idx, idx_int % width_, idx_int / width_, config, scanner, previous);
  }
  updateTerrainNeighborhood(config, &update_mask);
  applyHoleFill(config, &update_mask);
  for (const int idx_int : expanded) {
    const size_t idx = static_cast<size_t>(idx_int);
    applyValueAndMask(cells_[idx], config);
    values_[idx] = cells_[idx].value;
    mask_[idx] = cells_[idx].mask;
  }
}

bool ProjectionLayer::matchesGeometry(
  int width, int height, double resolution, const Eigen::Vector2d & origin) const
{
  return width_ == width && height_ == height &&
         cells_.size() == static_cast<size_t>(width) * static_cast<size_t>(height) &&
         std::abs(resolution_ - resolution) <= 1.0e-9 && (origin_ - origin).norm() <= 1.0e-9;
}

void ProjectionLayer::applyValueAndMask(CellData & cell, const ProjectionLayerConfig & config)
{
  switch (cell.type) {
  case CellType::UNKNOWN:
    cell.value = config.unknown_as_occupied ? 254U : 255U;
    cell.mask = config.unknown_as_occupied ? 0U : 1U;
    break;
  case CellType::FREE:
    cell.value = 0U;
    cell.mask = 1U;
    break;
  case CellType::PASSABLE:
    cell.value = config.passable_as_free ? 0U : config.passable_cost;
    cell.mask = 1U;
    break;
  case CellType::OCCUPIED:
    cell.value = 254U;
    cell.mask = 0U;
    break;
  }
}

void ProjectionLayer::updateOneCell(size_t idx,
  int x,
  int y,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<CellData> & previous)
{
  (void)x;
  (void)y;
  const ColumnStats stats = scanner(x, y);

  CellData cell;
  cell.last_hit_time = static_cast<float>(stats.last_hit_time);
  cell.last_update_time = static_cast<float>(stats.last_update_time);
  const CellType raw_type = classifyCell(stats, cell, config, resolution_);
  cell.raw_type = raw_type;
  if (!previous.empty() && idx < previous.size()) {
    cell.type = previous[idx].type;
    cell.pending_type = previous[idx].pending_type;
    cell.pending_count = previous[idx].pending_count;
    cell.stable_count = previous[idx].stable_count;
  } else {
    cell.type = raw_type;
    cell.pending_type = raw_type;
  }
  cells_[idx] = cell;
  cells_[idx].type = applyHysteresis(idx, raw_type, config);
  applyValueAndMask(cells_[idx], config);
  values_[idx] = cells_[idx].value;
  mask_[idx] = cells_[idx].mask;
}

CellType ProjectionLayer::applyHysteresis(
  size_t idx, CellType raw_type, const ProjectionLayerConfig & config)
{
  CellData & cell = cells_[idx];
  if (!config.hysteresis_en || config.hysteresis_count <= 0 || cell.type == CellType::UNKNOWN ||
      raw_type == CellType::OCCUPIED) {
    cell.type = raw_type;
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    cell.stable_count =
      raw_type == cell.type ? static_cast<uint8_t>(std::min<int>(255, cell.stable_count + 1)) : 0U;
    return cell.type;
  }

  if (raw_type == cell.type) {
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    cell.stable_count = static_cast<uint8_t>(std::min<int>(255, cell.stable_count + 1));
    return cell.type;
  }

  if (cell.type == CellType::OCCUPIED && raw_type != CellType::OCCUPIED) {
    if (cell.pending_type != raw_type) {
      cell.pending_type = raw_type;
      cell.pending_count = 1U;
    } else {
      cell.pending_count = static_cast<uint8_t>(std::min<int>(255, cell.pending_count + 1));
    }
    if (cell.pending_count >= static_cast<uint8_t>(config.hysteresis_count)) {
      cell.type = raw_type;
      cell.stable_count = 0U;
      cell.pending_count = 0U;
    }
    return cell.type;
  }

  cell.type = raw_type;
  cell.pending_type = raw_type;
  cell.pending_count = 0U;
  cell.stable_count = 0U;
  return cell.type;
}

void ProjectionLayer::updateTerrainNeighborhood(
  const ProjectionLayerConfig & config, const std::vector<uint8_t> * update_mask)
{
  if (!config.terrain_enable || width_ <= 0 || height_ <= 0 || cells_.empty()) {
    return;
  }
  const auto before = cells_;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
      if (update_mask && (idx >= update_mask->size() || (*update_mask)[idx] == 0U)) {
        continue;
      }
      auto & cell = cells_[idx];
      if (!finiteOrNan(cell.ground_z)) {
        cell.ground_z = std::numeric_limits<float>::quiet_NaN();
      }
      double max_step = 0.0;
      double max_slope = 0.0;
      if (std::isfinite(cell.ground_z)) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
              continue;
            }
            const size_t nidx =
              static_cast<size_t>(ny) * static_cast<size_t>(width_) + static_cast<size_t>(nx);
            const auto & neighbor = before[nidx];
            if (!std::isfinite(neighbor.ground_z)) {
              continue;
            }
            const double dz =
              std::abs(static_cast<double>(cell.ground_z) - static_cast<double>(neighbor.ground_z));
            const double run = resolution_ * std::sqrt(static_cast<double>(dx * dx + dy * dy));
            max_step = std::max(max_step, dz);
            if (run > 1.0e-9) {
              max_slope = std::max(max_slope, std::atan2(dz, run) * 180.0 / M_PI);
            }
          }
        }
      }
      cell.step_height = static_cast<float>(max_step);
      cell.slope_deg = static_cast<float>(max_slope);
      if (cell.type == CellType::PASSABLE &&
          (max_step > config.max_step_height || max_slope > config.max_slope_deg)) {
        cell.type = CellType::OCCUPIED;
        cell.traversable = 0U;
      }
    }
  }
}

void ProjectionLayer::applyHoleFill(
  const ProjectionLayerConfig & config, const std::vector<uint8_t> * update_mask)
{
  if (!config.hole_fill_en || config.hole_fill_radius <= 0 ||
      config.hole_fill_min_occupied_neighbors <= 0 || width_ <= 0 || height_ <= 0 || cells_.empty()) {
    return;
  }

  const auto before = cells_;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
      if (update_mask && (idx >= update_mask->size() || (*update_mask)[idx] == 0U)) {
        continue;
      }
      if (before[idx].type != CellType::UNKNOWN && before[idx].type != CellType::FREE) {
        continue;
      }

      int occupied_neighbors = 0;
      for (int dy = -config.hole_fill_radius; dy <= config.hole_fill_radius; ++dy) {
        for (int dx = -config.hole_fill_radius; dx <= config.hole_fill_radius; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            continue;
          }
          const size_t nidx =
            static_cast<size_t>(ny) * static_cast<size_t>(width_) + static_cast<size_t>(nx);
          if (before[nidx].type == CellType::OCCUPIED) {
            ++occupied_neighbors;
          }
        }
      }
      if (occupied_neighbors >= config.hole_fill_min_occupied_neighbors) {
        cells_[idx].type = CellType::OCCUPIED;
      }
    }
  }
}

}  // namespace rog_map
