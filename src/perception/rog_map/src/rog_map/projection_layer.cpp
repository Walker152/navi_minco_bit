#include <rog_map/projection_layer.hpp>

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

ProjectionSlideResult ProjectionLayer::syncSlidingWindow(int width,
  int height,
  double resolution,
  const Eigen::Vector2i & min_id,
  const Eigen::Vector2d & origin,
  const ProjectionLayerConfig & config)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 || width % 2 == 0 || height % 2 == 0) {
    throw std::invalid_argument(
      "ProjectionLayer sliding window requires positive odd dimensions and resolution");
  }

  ProjectionSlideResult result;
  current_config_ = config;
  const Vec3i half_size(width / 2, height / 2, 0);
  const Vec3i center_id(min_id.x() + half_size.x(), min_id.y() + half_size.y(), 0);
  Vec3f center_pos;

  if (!initialized_) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    cell_buffer_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), CellData{});
    cells_.assign(cell_buffer_.size(), CellData{});
    values_.assign(cell_buffer_.size(), 255U);
    mask_.assign(cell_buffer_.size(), config.unknown_as_occupied ? 0U : 1U);
    initSlidingMap(half_size, resolution, true, 0.0, Vec3f::Zero());
    globalIndexToPos(center_id, center_pos);
    updateLocalMapOriginAndBound(center_pos, center_id);
    initialized_ = true;
    resetLocalMap();
    result.full_refresh_required = true;
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
    rebuildViews(config);
    return result;
  }

  if (width_ != width || height_ != height || std::abs(resolution_ - resolution) > 1.0e-9 ||
      (sc_.map_size_i - Vec3i(width, height, 1)).cwiseAbs().maxCoeff() != 0 ||
      cell_buffer_.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
    throw std::invalid_argument(
      "ProjectionLayer sliding storage geometry cannot change after initialization");
  }

  const Vec3i shift = center_id - local_map_origin_i_;
  result.window_moved = shift.x() != 0 || shift.y() != 0;
  result.full_refresh_required = std::abs(shift.x()) >= width_ || std::abs(shift.y()) >= height_;
  slide_dirty_hash_ids_.clear();
  globalIndexToPos(center_id, center_pos);
  SlidingMap::mapSliding(center_pos);
  origin_ = origin;

  std::vector<uint8_t> dirty_flags(cell_buffer_.size(), 0U);
  for (const int hash_id : slide_dirty_hash_ids_) {
    if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
      continue;
    }
    Vec3i global_id;
    hashIdToGlobalIndex(hash_id, global_id);
    const int x = global_id.x() - local_map_bound_min_i_.x();
    const int y = global_id.y() - local_map_bound_min_i_.y();
    if (x < 0 || y < 0 || x >= width_ || y >= height_) {
      continue;
    }
    const int view_id = y * width_ + x;
    if (!dirty_flags[static_cast<size_t>(view_id)]) {
      dirty_flags[static_cast<size_t>(view_id)] = 1U;
      result.dirty_columns.push_back(view_id);
    }
  }
  if (result.full_refresh_required && result.dirty_columns.size() != cell_buffer_.size()) {
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
  }
  rebuildViews(config);
  return result;
}

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

  if (!initialized_) {
    const Eigen::Vector2i min_id(static_cast<int>(std::llround(origin.x() / resolution)),
      static_cast<int>(std::llround(origin.y() / resolution)));
    syncSlidingWindow(width, height, resolution, min_id, origin, config);
  }
  if (!matchesGeometry(width, height, resolution, origin)) {
    throw std::invalid_argument("ProjectionLayer::updateFull called with unsynchronized geometry");
  }
  current_config_ = config;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      updateOneCell(x, y, config, scanner);
    }
  }

  rebuildViews(config);

  const auto hole_start = std::chrono::steady_clock::now();
  applyHoleFill(config, nullptr);
  if (stats) {
    stats->hole_fill_time_ms += elapsedMs(hole_start);
  }

  const auto value_start = std::chrono::steady_clock::now();
  rebuildViews(config);
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

  current_config_ = config;
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

  for (const int idx_int : expanded) {
    updateOneCell(idx_int % width_, idx_int / width_, config, scanner);
  }

  rebuildViews(config);

  const auto hole_start = std::chrono::steady_clock::now();
  applyHoleFill(config, &update_mask);
  if (stats) {
    stats->hole_fill_time_ms += elapsedMs(hole_start);
  }

  const auto value_start = std::chrono::steady_clock::now();
  rebuildViews(config);
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

void ProjectionLayer::updateOneCell(
  int x, int y, const ProjectionLayerConfig & config, const ColumnScanner & scanner)
{
  const ColumnStats stats = scanner(x, y);
  const int hash_id = hashIndexFromLocal(x, y);
  const CellData previous = cell_buffer_[static_cast<size_t>(hash_id)];

  CellData cell;
  cell.last_hit_time = static_cast<float>(stats.last_hit_time);
  cell.last_update_time = static_cast<float>(stats.last_update_time);
  const CellType raw_type = classifyCell(stats, cell, config, resolution_);
  cell.raw_type = raw_type;
  cell.type = previous.type;
  cell.pending_type = previous.pending_type;
  cell.pending_count = previous.pending_count;
  cell.stable_count = previous.stable_count;
  cell.type = applyHysteresis(cell, raw_type, config);
  applyValueAndMask(cell, config);
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
}

CellType ProjectionLayer::applyHysteresis(
  CellData & cell, CellType raw_type, const ProjectionLayerConfig & config)
{
  // OCCUPIED enters immediately; clearing to FREE/PASSABLE waits for repeated confirmation.
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
        const int hash_id = hashIndexFromLocal(x, y);
        CellData & cell = cell_buffer_[static_cast<size_t>(hash_id)];
        cell.type = CellType::OCCUPIED;
        cell.hole_filled = true;
        cell.traversable = 0U;
        applyValueAndMask(cell, config);
      }
    }
  }
}

void ProjectionLayer::rebuildViews(const ProjectionLayerConfig & config)
{
  const size_t expected =
    static_cast<size_t>(std::max(0, width_)) * static_cast<size_t>(std::max(0, height_));
  cells_.resize(expected);
  values_.resize(expected);
  mask_.resize(expected);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t view_id = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
      CellData & stored = cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      applyValueAndMask(stored, config);
      cells_[view_id] = stored;
      values_[view_id] = stored.value;
      mask_[view_id] = stored.mask;
    }
  }
}

int ProjectionLayer::hashIndexFromLocal(int x, int y) const
{
  const Vec3i global_id(local_map_bound_min_i_.x() + x, local_map_bound_min_i_.y() + y, 0);
  return SlidingMap::getHashIndexFromGlobalIndex(global_id);
}

void ProjectionLayer::resetLocalMap()
{
  slide_dirty_hash_ids_.clear();
  for (size_t i = 0; i < cell_buffer_.size(); ++i) {
    CellData cell;
    applyValueAndMask(cell, current_config_);
    cell_buffer_[i] = cell;
    slide_dirty_hash_ids_.push_back(static_cast<int>(i));
  }
}

void ProjectionLayer::resetCell(const int & hash_id)
{
  if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
    return;
  }
  CellData cell;
  applyValueAndMask(cell, current_config_);
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
  slide_dirty_hash_ids_.push_back(hash_id);
}

}  // namespace rog_map
