#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rog_map {

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
};

struct PriorMapTransform2D
{
  double tx{0.0};
  double ty{0.0};
  double yaw{0.0};
};

PriorMapData loadPriorMap(const std::string & yaml_path, const std::string & pgm_path);

bool priorMapOccupied(const PriorMapData & prior_map, double map_x, double map_y);

void transformPriorMapPoint(
  const PriorMapTransform2D & transform, double rog_x, double rog_y, double & map_x, double & map_y);

void fusePriorMapProjection(const PriorMapData & prior_map,
  const PriorMapTransform2D * transform,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<uint8_t> & dynamic_mask,
  const std::vector<uint8_t> & dynamic_values,
  std::vector<uint8_t> & fused_mask,
  std::vector<uint8_t> & fused_values,
  std::vector<uint8_t> * prior_mask = nullptr);

}  // namespace rog_map
