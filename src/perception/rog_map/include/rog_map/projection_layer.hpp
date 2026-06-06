#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>

namespace rog_map {

enum class CellType : uint8_t {
    UNKNOWN = 0,
    FREE = 1,
    PASSABLE = 2,
    OCCUPIED = 3
};

struct CellData {
    CellType type{CellType::UNKNOWN};
    float min_z{0.0f};
    float max_z{0.0f};
    float height{0.0f};
    float ratio{0.0f};
    float last_hit_time{0.0f};
    float last_update_time{0.0f};
    uint8_t value{255};
    uint8_t mask{0};
};

struct ProjectionLayerConfig {
    bool unknown_as_occupied{true};
    double low_obstacle_height{0.07};
    double obstacle_height{0.14};
    double min_ratio{0.35};
    int min_observed_voxels{2};
    uint8_t passable_cost{50};
};

struct ColumnStats {
    int observed{0};
    int occupied{0};
    double min_z{std::numeric_limits<double>::infinity()};
    double max_z{-std::numeric_limits<double>::infinity()};
    double last_hit_time{0.0};
    double last_update_time{0.0};
};

class ProjectionLayer {
public:
    using ColumnScanner = std::function<ColumnStats(int gx, int gy)>;

    void update(
        int width,
        int height,
        double resolution,
        const Eigen::Vector2d &origin,
        const ProjectionLayerConfig &config,
        const ColumnScanner &scanner);

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    const Eigen::Vector2d &origin() const { return origin_; }

    const std::vector<CellData> &cells() const { return cells_; }
    const std::vector<uint8_t> &values() const { return values_; }
    const std::vector<uint8_t> &mask() const { return mask_; }
    bool empty() const { return values_.empty(); }

private:
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    Eigen::Vector2d origin_{0.0, 0.0};
    std::vector<CellData> cells_;
    std::vector<uint8_t> values_;
    std::vector<uint8_t> mask_;
};

}  // namespace rog_map
