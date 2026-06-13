#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>

namespace rog_map {

enum class CellType : uint8_t
{
  UNKNOWN = 0,
  FREE = 1,
  PASSABLE = 2,
  OCCUPIED = 3
};

enum class ProjectionClassReason : uint8_t
{
  INSUFFICIENT_OBSERVATION = 0,
  EMPTY_COLUMN,
  THIN_SURFACE,
  SOLID_VERTICAL_WALL,
  HOLLOW_TUNNEL,
  AMBIGUOUS_OCCUPIED
};

struct CellData
{
  CellType type{CellType::UNKNOWN};
  CellType raw_type{CellType::UNKNOWN};
  CellType pending_type{CellType::UNKNOWN};
  uint8_t value{255};
  uint8_t mask{1};
  uint8_t pending_count{0};
  uint8_t stable_count{0};
  float confidence{0.0f};
  float occupied_z_min_abs{std::numeric_limits<float>::quiet_NaN()};
  float occupied_z_max_abs{std::numeric_limits<float>::quiet_NaN()};
  float height_delta{0.0f};
  float vertical_occupancy_ratio{0.0f};
  ProjectionClassReason raw_reason{ProjectionClassReason::INSUFFICIENT_OBSERVATION};
  uint8_t traversable{0};
  bool hole_filled{false};
  float last_hit_time{0.0f};
  float last_update_time{0.0f};
};

struct ProjectionLayerConfig
{
  bool unknown_as_occupied{false};
  int min_observed_voxels{2};
  double surface_height_delta_max{0.10};
  double wall_height_delta_min{0.20};
  double wall_occupancy_ratio_min{0.80};
  double tunnel_height_delta_min{0.24};
  double tunnel_height_delta_max{0.40};
  double tunnel_occupancy_ratio_max{0.55};
  uint8_t passable_cost{50};
  bool passable_as_free{true};
  bool hysteresis_en{true};
  int hysteresis_count{2};
  bool hole_fill_en{true};
  int hole_fill_radius{1};
  int hole_fill_min_occupied_neighbors{5};
};

struct ColumnStats
{
  int observed_count{0};
  int occupied_count{0};
  int occupied_z_index_min{std::numeric_limits<int>::max()};
  int occupied_z_index_max{std::numeric_limits<int>::min()};
  double occupied_z_min_abs{std::numeric_limits<double>::infinity()};
  double occupied_z_max_abs{-std::numeric_limits<double>::infinity()};
  double last_hit_time{0.0};
  double last_update_time{0.0};
};

struct ProjectionUpdateStats
{
  double update_full_time_ms{0.0};
  double update_dirty_time_ms{0.0};
  double hole_fill_time_ms{0.0};
  double value_mask_time_ms{0.0};
  double thin_surface_count{0.0};
  double vertical_wall_count{0.0};
  double hollow_tunnel_count{0.0};
  double ambiguous_occupied_count{0.0};
  double empty_column_count{0.0};
  double insufficient_observation_count{0.0};
};

class ProjectionLayer
{
public:
  using ColumnScanner = std::function<ColumnStats(int gx, int gy)>;

  void update(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    ProjectionUpdateStats * stats = nullptr);

  void updateFull(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    ProjectionUpdateStats * stats = nullptr);

  void updateDirty(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    const std::vector<int> & dirty_columns,
    bool force_full_refresh,
    ProjectionUpdateStats * stats = nullptr);

  bool matchesGeometry(int width, int height, double resolution, const Eigen::Vector2d & origin) const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const Eigen::Vector2d & origin() const { return origin_; }

  const std::vector<CellData> & cells() const { return cells_; }
  const std::vector<uint8_t> & values() const { return values_; }
  const std::vector<uint8_t> & mask() const { return mask_; }
  bool empty() const { return values_.empty(); }

private:
  static void applyValueAndMask(CellData & cell, const ProjectionLayerConfig & config);
  static void collectClassificationStats(
    const std::vector<CellData> & cells, ProjectionUpdateStats * stats);
  CellType applyHysteresis(size_t idx, CellType raw_type, const ProjectionLayerConfig & config);
  void updateOneCell(size_t idx,
    int x,
    int y,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    const std::vector<CellData> & previous);
  void applyHoleFill(const ProjectionLayerConfig & config, const std::vector<uint8_t> * update_mask);

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};
  std::vector<CellData> cells_;
  std::vector<uint8_t> values_;
  std::vector<uint8_t> mask_;
};

}  // namespace rog_map
