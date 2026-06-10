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
  float min_z{std::numeric_limits<float>::quiet_NaN()};
  float max_z{std::numeric_limits<float>::quiet_NaN()};
  float height{0.0f};
  float ratio{0.0f};
  float ground_z{std::numeric_limits<float>::quiet_NaN()};
  float ceiling_z{std::numeric_limits<float>::infinity()};
  float slope_deg{0.0f};
  float step_height{0.0f};
  uint8_t traversable{0};
  float last_hit_time{0.0f};
  float last_update_time{0.0f};
};

struct ProjectionLayerConfig
{
  bool unknown_as_occupied{true};
  double low_obstacle_height{0.07};
  double obstacle_height{0.14};
  double min_ratio{0.35};
  int min_observed_voxels{2};
  uint8_t passable_cost{50};
  bool hysteresis_en{true};
  int hysteresis_count{2};
  bool hole_fill_en{true};
  int hole_fill_radius{1};
  int hole_fill_min_occupied_neighbors{5};
  bool terrain_enable{false};
  double robot_body_z_min{0.02};
  double robot_body_z_max{0.30};
  double surface_thickness{0.08};
  double max_step_height{0.10};
  double max_slope_deg{18.0};
  bool clearance_check_enable{false};
  double min_clearance_height{0.30};
  double tunnel_wall_min_height{0.18};
  bool passable_as_free{false};
};

struct ColumnStats
{
  int observed{0};
  int occupied{0};
  int occupied_in_body_band{0};
  int occupied_below_body{0};
  int occupied_above_body{0};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
  double ground_z{std::numeric_limits<double>::quiet_NaN()};
  double ceiling_z{std::numeric_limits<double>::infinity()};
  double body_band_min_z{std::numeric_limits<double>::infinity()};
  double body_band_max_z{-std::numeric_limits<double>::infinity()};
  double last_hit_time{0.0};
  double last_update_time{0.0};
};

struct ProjectionUpdateStats
{
  double update_full_time_ms{0.0};
  double update_dirty_time_ms{0.0};
  double terrain_time_ms{0.0};
  double hole_fill_time_ms{0.0};
  double value_mask_time_ms{0.0};
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
  CellType applyHysteresis(size_t idx, CellType raw_type, const ProjectionLayerConfig & config);
  void updateOneCell(size_t idx,
    int x,
    int y,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    const std::vector<CellData> & previous);
  void updateTerrainNeighborhood(
    const ProjectionLayerConfig & config, const std::vector<uint8_t> * update_mask);
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
