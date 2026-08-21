#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rog_map {

struct PriorMapTransform2D
{
  double tx{0.0};
  double ty{0.0};
  double yaw{0.0};
};

struct PriorMapData
{
  bool loaded{false};
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.25};
  std::vector<uint8_t> occupied;

  bool transform_ready{false};
  PriorMapTransform2D fixed_transform{};
  double fixed_transform_cos{1.0};
  double fixed_transform_sin{0.0};
  double fast_origin_yaw_cos{1.0};
  double fast_origin_yaw_sin{0.0};

  bool projection_cache_ready{false};
  int cached_min_x{0};
  int cached_min_y{0};
  int cached_width{0};
  int cached_height{0};
  double cached_resolution{0.0};
  std::vector<uint8_t> cached_mask;
  std::vector<uint8_t> scratch_mask;
};

PriorMapData loadPriorMap(const std::string & yaml_path, const std::string & pgm_path);

bool priorMapOccupied(const PriorMapData & prior_map, double map_x, double map_y);

void transformPriorMapPoint(
  const PriorMapTransform2D & transform, double rog_x, double rog_y, double & map_x, double & map_y);

bool initializePriorMapTransformOnce(
  PriorMapData & prior_map, const PriorMapTransform2D & transform);

bool refreshPriorMapProjectionCache(PriorMapData & prior_map,
  int min_global_x,
  int min_global_y,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y);

void fusePriorMapProjection(bool prior_enabled,
  const PriorMapData & prior_map,
  const std::vector<uint8_t> & dynamic_mask,
  const std::vector<uint8_t> & dynamic_values,
  std::vector<uint8_t> & fused_mask,
  std::vector<uint8_t> & fused_values);

}  // namespace rog_map
