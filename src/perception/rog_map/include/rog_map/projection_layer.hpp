#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>
#include <rog_map/rog_map_core/sliding_map.h>

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
  CellType base_type{CellType::UNKNOWN};
  CellType pending_type{CellType::UNKNOWN};
  uint8_t value{255};
  uint8_t mask{1};
  uint8_t pending_count{0};
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
  double occupied_clear_deadline{0.0};
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
  double obstacle_hold_time{0.0};
  bool mask_filter_en{true};
  int fill_occ_min{5};
  int denoise_occ_max{0};
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
  double mask_filter_time_ms{0.0};
  double value_mask_time_ms{0.0};
  double thin_surface_count{0.0};
  double vertical_wall_count{0.0};
  double hollow_tunnel_count{0.0};
  double ambiguous_occupied_count{0.0};
  double empty_column_count{0.0};
  double insufficient_observation_count{0.0};
};

struct ProjectionSlideResult
{
  bool window_moved{false};
  bool full_refresh_required{false};
  std::vector<int> dirty_columns;
};

class ProjectionLayer : protected SlidingMap
{
public:
  using ColumnScanner = std::function<ColumnStats(int gx, int gy)>;

  ProjectionSlideResult syncSlidingWindow(int width,
    int height,
    double resolution,
    const Eigen::Vector2i & min_id,
    const Eigen::Vector2d & origin,
    const ProjectionLayerConfig & config);

  void update(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    double now,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    ProjectionUpdateStats * stats = nullptr);

  void updateFull(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    double now,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    ProjectionUpdateStats * stats = nullptr);

  void updateDirty(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    double now,
    const ProjectionLayerConfig & config,
    const ColumnScanner & scanner,
    const std::vector<int> & dirty_columns,
    bool force_full_refresh,
    ProjectionUpdateStats * stats = nullptr);

  bool advanceObstacleClearance(double now, const ProjectionLayerConfig & config);

  bool matchesGeometry(int width, int height, double resolution, const Eigen::Vector2d & origin) const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const Eigen::Vector2d & origin() const { return origin_; }

  const std::vector<CellData> & cells() const { return cells_; }
  const std::vector<uint8_t> & values() const { return values_; }
  const std::vector<uint8_t> & mask() const { return mask_; }
  bool empty() const { return values_.empty(); }
  size_t storageCapacity() const { return cell_buffer_.size(); }

private:
  static void applyValueAndMask(CellData & cell, const ProjectionLayerConfig & config);
  static void collectClassificationStats(
    const std::vector<CellData> & cells, ProjectionUpdateStats * stats);
  CellType applyHysteresis(CellData & cell, CellType raw_type, const ProjectionLayerConfig & config);
  void updateOneCell(
    int x, int y, double now, const ProjectionLayerConfig & config, const ColumnScanner & scanner);
  void filterMask(
    const ProjectionLayerConfig & config,
    const std::vector<int> * base_dirty_indices,
    double * view_time_ms = nullptr);
  void fillMask(
    const ProjectionLayerConfig & config, const std::vector<int> & candidate_indices);
  void denoiseMask(
    const ProjectionLayerConfig & config, const std::vector<int> & candidate_indices);
  void updateView(int view_id, const ProjectionLayerConfig & config);
  void rebuildViews(const ProjectionLayerConfig & config);
  int hashIndexFromLocal(int x, int y) const;
  void resetLocalMap() override;
  void resetCell(const int & hash_id) override;

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};
  std::vector<CellData> cells_;
  std::vector<uint8_t> values_;
  std::vector<uint8_t> mask_;
  std::vector<CellData> cell_buffer_;
  std::vector<int> slide_dirty_hash_ids_;
  ProjectionLayerConfig current_config_;
  bool initialized_{false};
  bool view_rebuild_required_{false};
};

}  // namespace rog_map
