#include <rog_map/projection_layer.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace rog_map {
namespace {

constexpr double kResolution = 0.05;

ProjectionLayerConfig testConfig()
{
  ProjectionLayerConfig config;
  config.unknown_as_occupied = false;
  config.min_observed_voxels = 2;
  config.surface_height_delta_max = 0.10;
  config.wall_height_delta_min = 0.20;
  config.wall_occupancy_ratio_min = 0.80;
  config.tunnel_height_delta_min = 0.24;
  config.tunnel_height_delta_max = 0.40;
  config.tunnel_occupancy_ratio_max = 0.55;
  config.passable_cost = 50;
  config.passable_as_free = true;
  config.hysteresis_en = false;
  config.hysteresis_count = 2;
  config.hole_fill_en = false;
  config.hole_fill_radius = 1;
  config.hole_fill_min_occupied_neighbors = 5;
  return config;
}

ColumnStats makeStats(const std::vector<int> & occupied_indices, int observed_count = 8)
{
  ColumnStats stats;
  stats.observed_count = observed_count;
  stats.occupied_count = static_cast<int>(occupied_indices.size());
  for (const int index : occupied_indices) {
    stats.occupied_z_index_min = std::min(stats.occupied_z_index_min, index);
    stats.occupied_z_index_max = std::max(stats.occupied_z_index_max, index);
    const double occupied_z_abs = static_cast<double>(index) * kResolution;
    stats.occupied_z_min_abs = std::min(stats.occupied_z_min_abs, occupied_z_abs);
    stats.occupied_z_max_abs = std::max(stats.occupied_z_max_abs, occupied_z_abs);
  }
  stats.last_hit_time = stats.occupied_count > 0 ? 1.0 : 0.0;
  stats.last_update_time = 1.0;
  return stats;
}

CellData classifyOne(const ColumnStats & stats, const ProjectionLayerConfig & config = testConfig())
{
  ProjectionLayer layer;
  layer.updateFull(
    1,
    1,
    kResolution,
    Eigen::Vector2d::Zero(),
    config,
    [&stats](int, int) {
      return stats;
    },
    nullptr);
  EXPECT_EQ(layer.cells().size(), 1U);
  return layer.cells().front();
}

TEST(ProjectionHeightRatio, InsufficientObservationIsUnknown)
{
  const auto cell = classifyOne(makeStats({}, 1));

  EXPECT_EQ(cell.type, CellType::UNKNOWN);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::INSUFFICIENT_OBSERVATION);
  EXPECT_EQ(cell.traversable, 0U);
}

TEST(ProjectionHeightRatio, ObservedEmptyColumnIsFree)
{
  const auto cell = classifyOne(makeStats({}, 2));

  EXPECT_EQ(cell.type, CellType::FREE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(cell.traversable, 1U);
}

TEST(ProjectionHeightRatio, SingleLayerGroundIsThinSurface)
{
  const auto cell = classifyOne(makeStats({0}));

  EXPECT_EQ(cell.type, CellType::PASSABLE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::THIN_SURFACE);
  EXPECT_FLOAT_EQ(cell.height_delta, 0.0F);
  EXPECT_FLOAT_EQ(cell.vertical_occupancy_ratio, 1.0F);
}

TEST(ProjectionHeightRatio, RoughSlopeWithinSurfaceDeltaIsThinSurface)
{
  const auto cell = classifyOne(makeStats({0, 1, 2}));

  EXPECT_EQ(cell.type, CellType::PASSABLE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::THIN_SURFACE);
  EXPECT_NEAR(cell.height_delta, 0.10F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, ContinuousVerticalWallIsOccupied)
{
  const auto cell = classifyOne(makeStats({0, 1, 2, 3, 4, 5, 6}));

  EXPECT_EQ(cell.type, CellType::OCCUPIED);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::SOLID_VERTICAL_WALL);
  EXPECT_NEAR(cell.height_delta, 0.30F, 1.0e-6F);
  EXPECT_NEAR(cell.vertical_occupancy_ratio, 1.0F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, ThreeHundredMillimeterTunnelIsPassable)
{
  const auto cell = classifyOne(makeStats({0, 6}));

  EXPECT_EQ(cell.type, CellType::PASSABLE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::HOLLOW_TUNNEL);
  EXPECT_NEAR(cell.height_delta, 0.30F, 1.0e-6F);
  EXPECT_NEAR(cell.vertical_occupancy_ratio, 1.0F / 6.0F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, TunnelWithTwoLayerSurfacesIsPassable)
{
  const auto cell = classifyOne(makeStats({0, 1, 5, 6}));

  EXPECT_EQ(cell.type, CellType::PASSABLE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::HOLLOW_TUNNEL);
  EXPECT_NEAR(cell.vertical_occupancy_ratio, 0.5F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, RatioBetweenTunnelAndWallThresholdsIsAmbiguousOccupied)
{
  const auto cell = classifyOne(makeStats({0, 1, 3, 5, 6}));

  EXPECT_EQ(cell.type, CellType::OCCUPIED);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::AMBIGUOUS_OCCUPIED);
  EXPECT_NEAR(cell.height_delta, 0.30F, 1.0e-6F);
  EXPECT_NEAR(cell.vertical_occupancy_ratio, 4.0F / 6.0F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, HollowStructureBelowTunnelHeightIsAmbiguousOccupied)
{
  const auto cell = classifyOne(makeStats({0, 3}));

  EXPECT_EQ(cell.type, CellType::OCCUPIED);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::AMBIGUOUS_OCCUPIED);
  EXPECT_NEAR(cell.height_delta, 0.15F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, HollowStructureAboveTunnelHeightIsAmbiguousOccupied)
{
  const auto cell = classifyOne(makeStats({0, 10}));

  EXPECT_EQ(cell.type, CellType::OCCUPIED);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::AMBIGUOUS_OCCUPIED);
  EXPECT_NEAR(cell.height_delta, 0.50F, 1.0e-6F);
}

TEST(ProjectionHeightRatio, RatioIsFiniteAndClampedToUnitInterval)
{
  const auto wall = classifyOne(makeStats({0, 1, 2, 3, 4, 5, 6}));
  const auto tunnel = classifyOne(makeStats({0, 6}));

  EXPECT_TRUE(std::isfinite(wall.vertical_occupancy_ratio));
  EXPECT_TRUE(std::isfinite(tunnel.vertical_occupancy_ratio));
  EXPECT_GE(wall.vertical_occupancy_ratio, 0.0F);
  EXPECT_LE(wall.vertical_occupancy_ratio, 1.0F);
  EXPECT_GE(tunnel.vertical_occupancy_ratio, 0.0F);
  EXPECT_LE(tunnel.vertical_occupancy_ratio, 1.0F);
}

TEST(ProjectionHeightRatio, ThinSurfacePrecedesWallEvenWhenRatioIsOne)
{
  const auto cell = classifyOne(makeStats({4}));

  EXPECT_EQ(cell.type, CellType::PASSABLE);
  EXPECT_EQ(cell.raw_reason, ProjectionClassReason::THIN_SURFACE);
  EXPECT_FLOAT_EQ(cell.vertical_occupancy_ratio, 1.0F);
}

TEST(ProjectionHeightRatio, HysteresisImmediatelyEntersOccupiedAndDelaysClearing)
{
  ProjectionLayerConfig config = testConfig();
  config.hysteresis_en = true;
  config.hysteresis_count = 2;

  ProjectionLayer layer;
  ColumnStats current = makeStats({0});
  const auto scanner = [&current](int, int) {
    return current;
  };

  layer.updateFull(1, 1, kResolution, Eigen::Vector2d::Zero(), config, scanner, nullptr);
  EXPECT_EQ(layer.cells().front().type, CellType::PASSABLE);

  current = makeStats({0, 1, 2, 3, 4, 5, 6});
  layer.updateFull(1, 1, kResolution, Eigen::Vector2d::Zero(), config, scanner, nullptr);
  EXPECT_EQ(layer.cells().front().type, CellType::OCCUPIED);

  current = makeStats({0});
  layer.updateFull(1, 1, kResolution, Eigen::Vector2d::Zero(), config, scanner, nullptr);
  EXPECT_EQ(layer.cells().front().type, CellType::OCCUPIED);

  layer.updateFull(1, 1, kResolution, Eigen::Vector2d::Zero(), config, scanner, nullptr);
  EXPECT_EQ(layer.cells().front().type, CellType::PASSABLE);
}

TEST(ProjectionHeightRatio, HoleFillFillsUnknownOrFreeButNotHollowTunnelPassable)
{
  ProjectionLayerConfig config = testConfig();
  config.hole_fill_en = true;
  config.hole_fill_radius = 1;
  config.hole_fill_min_occupied_neighbors = 5;

  std::map<int, ColumnStats> columns;
  for (int i = 0; i < 9; ++i) {
    columns[i] = makeStats({0, 1, 2, 3, 4, 5, 6});
  }
  columns[4] = makeStats({}, 2);

  ProjectionLayer layer;
  layer.updateFull(
    3,
    3,
    kResolution,
    Eigen::Vector2d::Zero(),
    config,
    [&columns](int x, int y) {
      return columns[y * 3 + x];
    },
    nullptr);
  EXPECT_EQ(layer.cells()[4].raw_reason, ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(layer.cells()[4].type, CellType::OCCUPIED);
  EXPECT_TRUE(layer.cells()[4].hole_filled);

  columns[4] = makeStats({0, 6});
  layer.updateFull(
    3,
    3,
    kResolution,
    Eigen::Vector2d::Zero(),
    config,
    [&columns](int x, int y) {
      return columns[y * 3 + x];
    },
    nullptr);
  EXPECT_EQ(layer.cells()[4].raw_reason, ProjectionClassReason::HOLLOW_TUNNEL);
  EXPECT_EQ(layer.cells()[4].type, CellType::PASSABLE);
  EXPECT_FALSE(layer.cells()[4].hole_filled);
}

}  // namespace
}  // namespace rog_map
