#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <cmath>

namespace rog_map {

void ProjectionLayer::update(
    int width,
    int height,
    double resolution,
    const Eigen::Vector2d &origin,
    const ProjectionLayerConfig &config,
    const ColumnScanner &scanner) {
    if (width <= 0 || height <= 0 || resolution <= 0.0 || !scanner) {
        width_ = 0;
        height_ = 0;
        resolution_ = 0.0;
        cells_.clear();
        values_.clear();
        mask_.clear();
        return;
    }

    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;

    const size_t expected = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    cells_.assign(expected, CellData{});
    values_.assign(expected, 255U);
    mask_.assign(expected, config.unknown_as_occupied ? 0U : 1U);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width_) + static_cast<size_t>(x);
            const ColumnStats stats = scanner(x, y);

            CellData cell;
            cell.last_hit_time = static_cast<float>(stats.last_hit_time);
            cell.last_update_time = static_cast<float>(stats.last_update_time);

            if (stats.occupied > 0 && std::isfinite(stats.min_z) && std::isfinite(stats.max_z)) {
                cell.min_z = static_cast<float>(stats.min_z);
                cell.max_z = static_cast<float>(stats.max_z);
                cell.height = static_cast<float>(std::max(0.0, stats.max_z - stats.min_z));
                cell.ratio = static_cast<float>(
                    (static_cast<double>(stats.occupied + 1) * resolution_) /
                    std::max(static_cast<double>(cell.height), resolution_));
            }

            if (stats.observed < config.min_observed_voxels) {
                cell.type = CellType::UNKNOWN;
                cell.value = config.unknown_as_occupied ? 254U : 255U;
                cell.mask = config.unknown_as_occupied ? 0U : 1U;
            } else if (stats.occupied == 0) {
                cell.type = CellType::FREE;
                cell.value = 0U;
                cell.mask = 1U;
            } else if (cell.height <= config.low_obstacle_height) {
                cell.type = CellType::PASSABLE;
                cell.value = config.passable_cost;
                cell.mask = 1U;
            } else if (cell.height > config.obstacle_height && cell.ratio > config.min_ratio) {
                cell.type = CellType::OCCUPIED;
                cell.value = 254U;
                cell.mask = 0U;
            } else {
                cell.type = CellType::OCCUPIED;
                cell.value = 254U;
                cell.mask = 0U;
            }

            cells_[idx] = cell;
            values_[idx] = cell.value;
            mask_[idx] = cell.mask;
        }
    }
}

}  // namespace rog_map
