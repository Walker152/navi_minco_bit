#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

CellType classifyCell(
  const ColumnStats & stats, CellData & cell, const ProjectionLayerConfig & config, double resolution)
{
  const int min_observed = std::max(1, config.min_observed_voxels);
  cell.confidence = static_cast<float>(
    std::clamp(static_cast<double>(stats.observed_count) / static_cast<double>(min_observed), 0.0, 1.0));

  if (stats.observed_count < min_observed) {
    cell.raw_reason = ProjectionClassReason::INSUFFICIENT_OBSERVATION;
    cell.traversable = 0U;
    return CellType::UNKNOWN;
  }

  if (stats.occupied_count == 0) {
    cell.raw_reason = ProjectionClassReason::EMPTY_COLUMN;
    cell.traversable = 1U;
    return CellType::FREE;
  }

  const bool valid_occupied_span = stats.occupied_z_index_min <= stats.occupied_z_index_max &&
                                   std::isfinite(stats.occupied_z_min_abs) &&
                                   std::isfinite(stats.occupied_z_max_abs);
  if (!valid_occupied_span) {
    cell.raw_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  const int span_steps = stats.occupied_z_index_max - stats.occupied_z_index_min;
  const double height_delta = static_cast<double>(span_steps) * resolution;
  cell.occupied_z_min_abs = static_cast<float>(stats.occupied_z_min_abs);
  cell.occupied_z_max_abs = static_cast<float>(stats.occupied_z_max_abs);
  cell.height_delta = static_cast<float>(height_delta);

  if (height_delta <= config.surface_height_delta_max) {
    cell.vertical_occupancy_ratio = 1.0F;
    cell.raw_reason = ProjectionClassReason::THIN_SURFACE;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  const double vertical_occupancy_ratio =
    std::clamp((static_cast<double>(stats.occupied_count - 1) * resolution) / height_delta, 0.0, 1.0);
  cell.vertical_occupancy_ratio = static_cast<float>(vertical_occupancy_ratio);

  if (height_delta >= config.wall_height_delta_min &&
      vertical_occupancy_ratio >= config.wall_occupancy_ratio_min) {
    cell.raw_reason = ProjectionClassReason::SOLID_VERTICAL_WALL;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  if (height_delta >= config.tunnel_height_delta_min && height_delta <= config.tunnel_height_delta_max &&
      vertical_occupancy_ratio <= config.tunnel_occupancy_ratio_max) {
    cell.raw_reason = ProjectionClassReason::HOLLOW_TUNNEL;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  cell.raw_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
  cell.traversable = 0U;
  return CellType::OCCUPIED;
}

}  // namespace

void ProjectionLayer::update(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  ProjectionUpdateStats * stats)
{
  updateFull(width, height, resolution, origin, config, scanner, stats);
}

void ProjectionLayer::updateFull(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  ProjectionUpdateStats * stats)
{
  const auto full_start = std::chrono::steady_clock::now();
  if (width <= 0 || height <= 0 || resolution <= 0.0 || !scanner) {
    width_ = 0;
    height_ = 0;
    resolution_ = 0.0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    if (stats) {
      stats->update_full_time_ms = elapsedMs(full_start);
    }
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

  const auto hole_start = std::chrono::steady_clock::now();
  applyHoleFill(config, nullptr);
  if (stats) {
    stats->hole_fill_time_ms += elapsedMs(hole_start);
  }

  const auto value_start = std::chrono::steady_clock::now();
  for (size_t idx = 0; idx < cells_.size(); ++idx) {
    applyValueAndMask(cells_[idx], config);
    values_[idx] = cells_[idx].value;
    mask_[idx] = cells_[idx].mask;
  }
  if (stats) {
    stats->value_mask_time_ms += elapsedMs(value_start);
    stats->update_full_time_ms += elapsedMs(full_start);
    collectClassificationStats(cells_, stats);
  }
}

void ProjectionLayer::updateDirty(int width,
  int height,
  double resolution,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<int> & dirty_columns,
  bool force_full_refresh,
  ProjectionUpdateStats * stats)
{
  const auto dirty_start = std::chrono::steady_clock::now();
  const bool geometry_changed = !matchesGeometry(width, height, resolution, origin);
  if (force_full_refresh || geometry_changed || cells_.empty()) {
    updateFull(width, height, resolution, origin, config, scanner, stats);
    return;
  }
  if (dirty_columns.empty()) {
    if (stats) {
      stats->update_dirty_time_ms += elapsedMs(dirty_start);
      collectClassificationStats(cells_, stats);
    }
    return;
  }

  const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::vector<uint8_t> update_mask(expected, 0U);
  const int dirty_radius = config.hole_fill_en ? std::max(0, config.hole_fill_radius) : 0;
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

  const auto hole_start = std::chrono::steady_clock::now();
  applyHoleFill(config, &update_mask);
  if (stats) {
    stats->hole_fill_time_ms += elapsedMs(hole_start);
  }

  const auto value_start = std::chrono::steady_clock::now();
  for (const int idx_int : expanded) {
    const size_t idx = static_cast<size_t>(idx_int);
    applyValueAndMask(cells_[idx], config);
    values_[idx] = cells_[idx].value;
    mask_[idx] = cells_[idx].mask;
  }
  if (stats) {
    stats->value_mask_time_ms += elapsedMs(value_start);
    stats->update_dirty_time_ms += elapsedMs(dirty_start);
    collectClassificationStats(cells_, stats);
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
  // value is a cost/debug layer; mask is the 2D ESDF source, where 0 is obstacle and 1 is free.
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

void ProjectionLayer::collectClassificationStats(
  const std::vector<CellData> & cells, ProjectionUpdateStats * stats)
{
  if (!stats) {
    return;
  }
  stats->thin_surface_count = 0.0;
  stats->vertical_wall_count = 0.0;
  stats->hollow_tunnel_count = 0.0;
  stats->ambiguous_occupied_count = 0.0;
  stats->empty_column_count = 0.0;
  stats->insufficient_observation_count = 0.0;
  for (const auto & cell : cells) {
    switch (cell.raw_reason) {
    case ProjectionClassReason::INSUFFICIENT_OBSERVATION:
      stats->insufficient_observation_count += 1.0;
      break;
    case ProjectionClassReason::EMPTY_COLUMN:
      stats->empty_column_count += 1.0;
      break;
    case ProjectionClassReason::THIN_SURFACE:
      stats->thin_surface_count += 1.0;
      break;
    case ProjectionClassReason::SOLID_VERTICAL_WALL:
      stats->vertical_wall_count += 1.0;
      break;
    case ProjectionClassReason::HOLLOW_TUNNEL:
      stats->hollow_tunnel_count += 1.0;
      break;
    case ProjectionClassReason::AMBIGUOUS_OCCUPIED:
      stats->ambiguous_occupied_count += 1.0;
      break;
    }
  }
}

void ProjectionLayer::updateOneCell(size_t idx,
  int x,
  int y,
  const ProjectionLayerConfig & config,
  const ColumnScanner & scanner,
  const std::vector<CellData> & previous)
{
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
  // OCCUPIED enters immediately; clearing to FREE/PASSABLE waits for repeated confirmation.
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

void ProjectionLayer::applyHoleFill(
  const ProjectionLayerConfig & config, const std::vector<uint8_t> * update_mask)
{
  // Only UNKNOWN/FREE holes are filled. PASSABLE, including HOLLOW_TUNNEL, keeps its raw classification.
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
        cells_[idx].hole_filled = true;
        cells_[idx].traversable = 0U;
      }
    }
  }
}

}  // namespace rog_map
