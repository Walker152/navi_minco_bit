/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <rog_map/prob_map.h>
#ifdef _OPENMP
#include <omp.h>
#endif
using namespace rog_map;
using namespace super_utils;

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

bool vec3iLess(const Vec3i & a, const Vec3i & b)
{
  if (a.x() != b.x()) {
    return a.x() < b.x();
  }
  if (a.y() != b.y()) {
    return a.y() < b.y();
  }
  return a.z() < b.z();
}

void uniqueVec3i(vec_E<Vec3i> & ids)
{
  std::sort(ids.begin(), ids.end(), vec3iLess);
  ids.erase(std::unique(ids.begin(),
              ids.end(),
              [](const Vec3i & a, const Vec3i & b) {
                return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
              }),
    ids.end());
}

}  // namespace

void ProbMap::initProbMap()
{
  static bool init_once{false};
  if (init_once) {
    throw std::runtime_error(" -- [ROGMap] ProbMap can only init once.");
  }
  init_once = true;
  initSlidingMap(cfg_.half_map_size_i,
    cfg_.resolution,
    cfg_.map_sliding_en,
    cfg_.map_sliding_thresh,
    cfg_.fix_map_origin);
  inf_map_ = std::make_shared<InfMap>(cfg_);

  if (cfg_.frontier_extraction_en) {
    fcnt_map_ = std::make_shared<FreeCntMap>(cfg_.half_map_size_i + Vec3i::Constant(2),
      cfg_.resolution,
      cfg_.map_sliding_en,
      cfg_.map_sliding_thresh,
      cfg_.fix_map_origin);
  }

  if (cfg_.esdf_en) {
    esdf_map_ = std::make_shared<ESDFMap>();
    esdf_map_->initESDFMap(cfg_.half_map_size_i,
      cfg_.resolution,
      cfg_.esdf_resolution,
      cfg_.esdf_local_update_box,
      cfg_.map_sliding_en,
      cfg_.map_sliding_thresh,
      cfg_.fix_map_origin,
      cfg_.unk_thresh);
  }

  posToGlobalIndex(cfg_.virtual_ceil_height, sc_.virtual_ceil_height_id_g);
  posToGlobalIndex(cfg_.virtual_ground_height, sc_.virtual_ground_height_id_g);

  cfg_.virtual_ceil_height = sc_.virtual_ceil_height_id_g * cfg_.resolution;
  cfg_.virtual_ground_height = sc_.virtual_ground_height_id_g * cfg_.resolution;

  cout << "[ProbMap] virtual_ceil_height: " << cfg_.virtual_ceil_height << endl;
  cout << "[ProbMap] virtual_ground_height: " << cfg_.virtual_ground_height << endl;

  if (!cfg_.map_sliding_en) {
    std::cout << YELLOW << " -- [ProbMap] Map sliding disabled, set origin to ["
              << cfg_.fix_map_origin.transpose() << "] -- " << RESET << std::endl;
    slideAllMap(cfg_.fix_map_origin);
  }

  int map_size = sc_.map_size_i.prod();

  occupancy_buffer_.resize(map_size, 0);
  last_hit_time_.resize(map_size, 0.0);
  last_update_time_.resize(map_size, 0.0);
  active_flags_.resize(map_size, 0U);
  active_ids_.clear();
  dirty_column_flags_.resize(
    static_cast<size_t>(sc_.map_size_i.x()) * static_cast<size_t>(sc_.map_size_i.y()), 0U);
  dirty_column_ids_.clear();
  full_layer_refresh_required_ = true;
  raycast_data_.raycaster.setResolution(cfg_.resolution);
  raycast_data_.operation_cnt.resize(map_size, 0);
  raycast_data_.hit_cnt.resize(map_size, 0);

  resetLocalMap();

  std::cout << GREEN << " -- [ProbMap] Init successfully -- ." << RESET << std::endl;
  printMapInformation();
}

void ProbMap::setUpdateTime(double now)
{
  current_update_time_ = now;
}

Vec3f ProbMap::getLocalMapOrigin() const
{
  return local_map_origin_d_;
}

Vec3f ProbMap::getLocalMapSize() const
{
  return cfg_.map_size_d;
}

// Query====================================================
bool ProbMap::isOccupied(const Vec3f & pos) const
{
  if (!insideLocalMap(pos)) {
    return false;
  }
  if (pos.z() > cfg_.virtual_ceil_height || pos.z() < cfg_.virtual_ground_height) {
    return true;
  }
  return isOccupied(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isUnknown(const Vec3f & pos) const
{
  if (!insideLocalMap(pos)) {
    return true;
  }
  if (pos.z() > cfg_.virtual_ceil_height || pos.z() < cfg_.virtual_ground_height) {
    return false;
  }

  return isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isKnownFree(const Vec3f & pos) const
{
  if (!insideLocalMap(pos)) {
    return false;
  }
  if (pos.z() > cfg_.virtual_ceil_height || pos.z() < cfg_.virtual_ground_height) {
    return false;
  }
  return isKnownFree(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isFrontier(const Vec3f & pos) const
{
  // 1) Check local map
  if (!insideLocalMap(pos)) {
    return false;
  }

  // 2) Check virtual ceil and ground
  if (pos.z() > cfg_.virtual_ceil_height || pos.z() < cfg_.virtual_ground_height) {
    return false;
  }

  // 3) Check frontier
  if (isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)])) {
    if (fcnt_map_->getFreeCnt(pos)) {
      return true;
    }
  }
  return false;
}

// ======================================================
bool ProbMap::isOccupied(const Vec3i & id_g) const
{
  if (!insideLocalMap(id_g)) {
    return false;
  }
  if (id_g.z() > sc_.virtual_ceil_height_id_g ||
      id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
    return true;
  }
  return isOccupied(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isUnknown(const Vec3i & id_g) const
{
  if (!insideLocalMap(id_g)) {
    return true;
  }
  if (id_g.z() > sc_.virtual_ceil_height_id_g ||
      id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
    return false;
  }
  return isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isKnownFree(const Vec3i & id_g) const
{
  if (!insideLocalMap(id_g)) {
    return false;
  }
  if (id_g.z() > sc_.virtual_ceil_height_id_g ||
      id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
    return true;
  }
  return isKnownFree(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

double ProbMap::cellLastHitTime(const Vec3i & id_g) const
{
  if (!insideLocalMap(id_g)) {
    return 0.0;
  }
  return last_hit_time_[getHashIndexFromGlobalIndex(id_g)];
}

double ProbMap::cellLastUpdateTime(const Vec3i & id_g) const
{
  if (!insideLocalMap(id_g)) {
    return 0.0;
  }
  return last_update_time_[getHashIndexFromGlobalIndex(id_g)];
}

bool ProbMap::isFrontier(const Vec3i & id_g) const
{
  // 1) Check local map
  if (!insideLocalMap(id_g)) {
    return false;
  }
  if (id_g.z() > sc_.virtual_ceil_height_id_g ||
      id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
    return false;
  }

  if (isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)])) {
    if (fcnt_map_->getFreeCnt(id_g) > 0) {
      return true;
    }
  }
  return false;
}

bool ProbMap::isKnownFreeInflate(const Vec3f & pos) const
{
  return inf_map_->isKnownFreeInflate(pos);
}

bool ProbMap::isOccupiedInflate(const Vec3f & pos) const
{
  return inf_map_->isOccupiedInflate(pos);
}

bool ProbMap::isUnknownInflate(const Vec3f & pos) const
{
  return inf_map_->isUnknownInflate(pos);
}

// Query====================================================

void ProbMap::updateOccPointCloud(const PointCloud & input_cloud)
{
  /// Step 1; Raycast and add to update cache.
  const int cloud_in_size = input_cloud.size();
  Vec3f localmap_min = local_map_bound_min_d_;
  Vec3f localmap_max = local_map_bound_max_d_;
  for (int i = 0; i < cloud_in_size; i++) {
    static Vec3f p, ray_pt;
    static Vec3i pt_id_g, pt_id_l;
    if (cfg_.intensity_thresh > 0 && input_cloud[i].intensity < cfg_.intensity_thresh) {
      continue;
    }

    p.x() = input_cloud[i].x;
    p.y() = input_cloud[i].y;
    p.z() = input_cloud[i].z;

    posToGlobalIndex(p, pt_id_g);

    if (p.z() > cfg_.virtual_ceil_height || p.z() < cfg_.virtual_ground_height) {
      continue;
    }
    if (insideLocalMap(pt_id_g)) {
      const int occ_hit_num = ceil(cfg_.l_occ / cfg_.l_hit);
      for (int j = 0; j < occ_hit_num; j++) {
        insertUpdateCandidate(pt_id_g, true);
      }
      localmap_max = localmap_max.cwiseMax(p);
      localmap_min = localmap_min.cwiseMin(p);
    }
  }
  if (cfg_.map_sliding_en) {
    local_map_bound_max_d_ = localmap_max;
    local_map_bound_min_d_ = localmap_min;
  }
  probabilisticMapFromCache();
  map_empty_ = false;
}

void ProbMap::slideAllMap(const rog_map::Vec3f & pos)
{
  mapSliding(pos);
  inf_map_->mapSliding(pos);
  if (cfg_.frontier_extraction_en) {
    fcnt_map_->mapSliding(pos);
  }
  if (cfg_.esdf_en) {
    esdf_map_->mapSliding(pos);
  }
  markAllDirtyColumns();
}

void ProbMap::updateProbMap(
  const PointCloud & cloud, const Pose & sensor_pose, const Vec3f & map_center_pos)
{
  TimeConsuming tc("updateMap", false);
  runtime_stats_ = RuntimeStats{};
  const Vec3f & sensor_pos = sensor_pose.first;
  runtime_stats_.input_point_count = cloud.size();
  if (cfg_.map_sliding_en && !insideLocalMap(map_center_pos) &&
      raycast_data_.batch_update_counter == 0) {
    std::cout << YELLOW << " -- [ROGMapCore] cur_pose out of map range, reset the map." << RESET
              << std::endl;
    std::cout << YELLOW << " -- [ROGMapCore] Sliding to map center at: "
              << map_center_pos.transpose() << RESET << std::endl;
    slideAllMap(map_center_pos);
    return;
  }

  if (sensor_pos.z() > cfg_.virtual_ceil_height) {
    std::cout << YELLOW << " -- [ROGMapCore] Odom above virtual ceil, please check map parameter -- ."
              << RESET << std::endl;
    return;
  } else if (sensor_pos.z() < cfg_.virtual_ground_height) {
    std::cout << YELLOW << " -- [ROGMapCore] Odom below virtual ground, please check map parameter -- ."
              << RESET << std::endl;
    return;
  }

  if (raycast_data_.batch_update_counter == 0 && cfg_.map_sliding_en &&
      (map_empty_ || (map_center_pos - local_map_origin_d_).norm() > cfg_.map_sliding_thresh)) {
    slideAllMap(map_center_pos);
  }

  updateLocalBox(map_center_pos);
  TimeConsuming t_raycast("raycast", false);
  raycastProcess(cloud, sensor_pos);
  runtime_stats_.raycast_time = t_raycast.stop();
  raycast_data_.batch_update_counter++;
  if (raycast_data_.batch_update_counter >= cfg_.batch_update_size) {
    raycast_data_.batch_update_counter = 0;
    runtime_stats_.cache_count = static_cast<double>(raycast_data_.update_cache_id_g.size());
    TimeConsuming t_update("update", false);
    probabilisticMapFromCache();
    runtime_stats_.prob_update_time = t_update.stop();
    map_empty_ = false;
  }
  runtime_stats_.dirty_column_count_from_probmap = static_cast<double>(dirtyColumnIds().size());
  runtime_stats_.active_cell_count = static_cast<double>(active_ids_.size());
  inf_map_->getInflationNumAndTime(runtime_stats_.inflation_count, runtime_stats_.inflation_time);
  runtime_stats_.total_update_time = tc.stop();

  /* Update ESDF map */
  if (cfg_.esdf_en) {
    esdf_map_->updateESDF3D(map_center_pos);
  }

  /* For the first frame, clear all unknown around the robot */
  static bool first = true;
  if (first) {
    first = false;
    for (double dx = -cfg_.raycast_range_min; dx <= cfg_.raycast_range_min; dx += cfg_.resolution) {
      for (double dy = -cfg_.raycast_range_min; dy <= cfg_.raycast_range_min; dy += cfg_.resolution) {
        for (double dz = -cfg_.raycast_range_min; dz <= cfg_.raycast_range_min; dz += cfg_.resolution) {
          Vec3f p(dx, dy, dz);
          if (p.norm() <= cfg_.raycast_range_min) {
            Vec3f pp = sensor_pos + p;
            int hash_id = getHashIndexFromPos(pp);
            missPointUpdate(pp, hash_id, 999);
          }
        }
      }
    }
  }
}

GridType ProbMap::getGridType(Vec3i & id_g) const
{
  if (id_g.z() <= sc_.virtual_ground_height_id_g ||
      id_g.z() >= sc_.virtual_ceil_height_id_g - sc_.safe_margin_i) {
    return super_utils::OCCUPIED;
  }
  if (!insideLocalMap(id_g)) {
    return super_utils::OUT_OF_MAP;
  }
  Vec3i id_l;
  globalIndexToLocalIndex(id_g, id_l);
  int hash_id = getLocalIndexHash(id_l);
  double ret = occupancy_buffer_[hash_id];
  if (isKnownFree(ret)) {
    return GridType::KNOWN_FREE;
  } else if (isOccupied(ret)) {
    return GridType::OCCUPIED;
  } else {
    return GridType::UNKNOWN;
  }
}

GridType ProbMap::getGridType(const Vec3f & pos) const
{
  if (pos.z() <= cfg_.virtual_ground_height || pos.z() >= cfg_.virtual_ceil_height) {
    return OCCUPIED;
  }
  if (!insideLocalMap(pos)) {
    return OUT_OF_MAP;
  }
  Vec3i id_g, id_l;
  posToGlobalIndex(pos, id_g);
  return getGridType(id_g);
}

GridType ProbMap::getInfGridType(const Vec3f & pos) const
{
  // NOTE, we consider, if the pos is not inside prob map, it is also out of inf map.
  if (!insideLocalMap(pos)) {
    return OUT_OF_MAP;
  }
  return inf_map_->getGridType(pos);
}

double ProbMap::getMapValue(const Vec3f & pos) const
{
  if (!insideLocalMap(pos)) {
    return 0;
  }
  return occupancy_buffer_[getHashIndexFromPos(pos)];
}

void ProbMap::boxSearch(
  const Vec3f & _box_min, const Vec3f & _box_max, const GridType & gt, vec_E<Vec3f> & out_points) const
{
  out_points.clear();
  if (map_empty_) {
    std::cout << YELLOW << " -- [ROG] Map is empty, cannot perform box search." << RESET << std::endl;
    return;
  }
  if ((_box_max - _box_min).minCoeff() <= 0) {
    std::cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << std::endl;
    return;
  }
  Vec3f box_min_d = _box_min, box_max_d = _box_max;
  boundBoxByLocalMap(box_min_d, box_max_d);
  if ((box_max_d - box_min_d).minCoeff() <= 0) {
    std::cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << std::endl;
    return;
  }
  Vec3i box_min_id_g, box_max_id_g;
  posToGlobalIndex(box_min_d, box_min_id_g);
  posToGlobalIndex(box_max_d, box_max_id_g);
  Vec3i box_size = box_max_id_g - box_min_id_g;
  if (gt == UNKNOWN) {
    out_points.reserve(box_size.prod());
    for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
      for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
        for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
          Vec3i id_g(i, j, k);
          if (isUnknown(id_g)) {
            Vec3f pos;
            globalIndexToPos(id_g, pos);
            out_points.push_back(pos);
          }
        }
      }
    }
  } else if (gt == OCCUPIED) {
    out_points.reserve(box_size.prod() / 3);
    for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
      for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
        for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
          Vec3i id_g(i, j, k);
          if (isOccupied(id_g)) {
            Vec3f pos;
            globalIndexToPos(id_g, pos);
            out_points.push_back(pos);
          }
        }
      }
    }
  } else if (gt == FRONTIER) {
    out_points.reserve(box_size.prod() / 3);
    for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
      for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
        for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
          Vec3i id_g(i, j, k);
          if (isFrontier(id_g)) {
            Vec3f pos;
            globalIndexToPos(id_g, pos);
            out_points.push_back(pos);
          }
        }
      }
    }
  } else {
    throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
  }
}

void ProbMap::rawOccupiedBoxSearch(
  const Vec3f & _box_min, const Vec3f & _box_max, vec_E<Vec3f> & out_points) const
{
  out_points.clear();
  if (map_empty_) {
    std::cout << YELLOW << " -- [ROG] Map is empty, cannot perform raw occupied box search." << RESET
              << std::endl;
    return;
  }
  if ((_box_max - _box_min).minCoeff() <= 0) {
    std::cout << YELLOW << " -- [ROG] Raw occupied box search failed, box size is zero." << RESET
              << std::endl;
    return;
  }

  Vec3f box_min_d = _box_min, box_max_d = _box_max;
  boundBoxByLocalMap(box_min_d, box_max_d);
  if ((box_max_d - box_min_d).minCoeff() <= 0) {
    std::cout << YELLOW << " -- [ROG] Raw occupied box search failed, box size is zero." << RESET
              << std::endl;
    return;
  }

  Vec3i box_min_id_g, box_max_id_g;
  posToGlobalIndex(box_min_d, box_min_id_g);
  posToGlobalIndex(box_max_d, box_max_id_g);
  const Vec3i box_size = box_max_id_g - box_min_id_g;
  out_points.reserve(box_size.prod() / 3);

  for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); ++i) {
    for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); ++j) {
      for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); ++k) {
        const Vec3i id_g(i, j, k);
        if (!insideLocalMap(id_g)) {
          continue;
        }
        const int hash_id = getHashIndexFromGlobalIndex(id_g);
        if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
          continue;
        }
        if (!isOccupied(occupancy_buffer_[hash_id])) {
          continue;
        }
        Vec3f pos;
        globalIndexToPos(id_g, pos);
        out_points.push_back(pos);
      }
    }
  }
}

void ProbMap::boxSearchInflate(
  const Vec3f & box_min, const Vec3f & box_max, const GridType & gt, vec_E<Vec3f> & out_points) const
{
  inf_map_->boxSearch(box_min, box_max, gt, out_points);
}

void ProbMap::boundBoxByLocalMap(Vec3f & box_min, Vec3f & box_max) const
{
  if ((box_max - box_min).minCoeff() <= 0) {
    box_min = box_max;
    std::cout << YELLOW << "-- [ROG] Bound box is invalid." << RESET << std::endl;
    return;
  }

  box_min = box_min.cwiseMax(local_map_bound_min_d_);
  box_max = box_max.cwiseMin(local_map_bound_max_d_);
  box_max.z() = std::min(box_max.z(), cfg_.virtual_ceil_height);
  box_min.z() = std::max(box_min.z(), cfg_.virtual_ground_height);
}

void ProbMap::resetCell(const int & hash_id)
{
  float & ret = occupancy_buffer_[hash_id];
  if (isOccupied(ret)) {
    /// if current state is occupied
    Vec3f pos;
    hashIdToPos(hash_id, pos);
    inf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
    if (cfg_.esdf_en) {
      esdf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
    }
  } else if (isKnownFree(ret)) {
    /// if current state is free
    Vec3f pos;
    hashIdToPos(hash_id, pos);
    inf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
    if (cfg_.esdf_en) {
      esdf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
    }
    if (cfg_.frontier_extraction_en) {
      Vec3i id_g;
      posToGlobalIndex(pos, id_g);
      if (cfg_.frontier_extraction_en) {
        fcnt_map_->updateFrontierCounter(id_g, false);
      }
    }
  } else {
    // nothing need to do
  }
  ret = 0;
  if (hash_id >= 0 && hash_id < static_cast<int>(last_hit_time_.size())) {
    Vec3f pos;
    hashIdToPos(hash_id, pos);
    Vec3i id_g;
    posToGlobalIndex(pos, id_g);
    markDirtyColumn(id_g);
    last_hit_time_[hash_id] = 0.0f;
    last_update_time_[hash_id] = 0.0f;
    active_flags_[hash_id] = 0U;
  }
}

GridType ProbMap::classifyProb(const float & prob) const
{
  if (isOccupied(prob)) {
    return GridType::OCCUPIED;
  }
  if (isKnownFree(prob)) {
    return GridType::KNOWN_FREE;
  }
  return GridType::UNKNOWN;
}

void ProbMap::updateCellState(const Vec3f & pos, const GridType & from_type, const GridType & to_type)
{
  if (from_type == to_type) {
    return;
  }

  Vec3f center_pos;
  Vec3i id_g;
  posToGlobalIndex(pos, id_g);
  globalIndexToPos(id_g, center_pos);

  inf_map_->updateGridCounter(center_pos, from_type, to_type);
  if (cfg_.esdf_en) {
    esdf_map_->updateGridCounter(center_pos, from_type, to_type);
  }

  if (cfg_.frontier_extraction_en) {
    if (from_type == KNOWN_FREE) {
      fcnt_map_->updateFrontierCounter(id_g, false);
    }
    if (to_type == KNOWN_FREE) {
      fcnt_map_->updateFrontierCounter(id_g, true);
    }
  }
  markDirtyColumn(id_g);
}

void ProbMap::probabilisticMapFromCache()
{
  //    int addr = getHashIndexFromGlobalIndex(Vec3i(41,
  //                                                 -216,
  //                                                 -6));
  //    float ret = occupancy_buffer_[addr];
  //    std::cout << "ret: " << ret << std::endl;
  while (!raycast_data_.update_cache_id_g.empty()) {
    Vec3f pos;
    Vec3i id_g = raycast_data_.update_cache_id_g.front();
    raycast_data_.update_cache_id_g.pop();
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    int hash_id = getLocalIndexHash(id_l);
    globalIndexToPos(id_g, pos);
    if (raycast_data_.hit_cnt[hash_id] > 0) {
      hitPointUpdate(pos, hash_id, raycast_data_.hit_cnt[hash_id]);
    } else {
      missPointUpdate(pos, hash_id, raycast_data_.operation_cnt[hash_id] - raycast_data_.hit_cnt[hash_id]);
    }
    raycast_data_.hit_cnt[hash_id] = 0;
    raycast_data_.operation_cnt[hash_id] = 0;
  }
}

void ProbMap::hitPointUpdate(const Vec3f & pos, const int & hash_id, const int & hit_num)
{
  float & ret = occupancy_buffer_[hash_id];
  GridType from_type = classifyProb(ret);

  ret += cfg_.l_hit * hit_num;
  if (ret > cfg_.l_max) {
    ret = cfg_.l_max;
  }

  const GridType to_type = classifyProb(ret);
  updateCellState(pos, from_type, to_type);
  Vec3i id_g;
  posToGlobalIndex(pos, id_g);
  markDirtyColumn(id_g);

  if (hash_id >= 0 && hash_id < static_cast<int>(last_hit_time_.size())) {
    last_hit_time_[hash_id] = current_update_time_;
    last_update_time_[hash_id] = current_update_time_;
    if (cfg_.decay_active_list_en && hash_id < static_cast<int>(active_flags_.size()) &&
        !active_flags_[hash_id]) {
      active_flags_[hash_id] = 1U;
      active_ids_.push_back(hash_id);
    }
  }
}

void ProbMap::missPointUpdate(const Vec3f & pos, const int & hash_id, const int & hit_num)
{
  float & ret = occupancy_buffer_[hash_id];
  GridType from_type = classifyProb(ret);
  ret += cfg_.l_miss * hit_num;
  if (ret < cfg_.l_min) {
    ret = cfg_.l_min;
  }

  const GridType to_type = classifyProb(ret);
  updateCellState(pos, from_type, to_type);
  Vec3i id_g;
  posToGlobalIndex(pos, id_g);
  markDirtyColumn(id_g);

  if (hash_id >= 0 && hash_id < static_cast<int>(last_update_time_.size())) {
    last_update_time_[hash_id] = static_cast<float>(current_update_time_);
  }
}

bool ProbMap::applyDecay(double now)
{
  runtime_stats_.decayed_count = 0.0;
  if (!cfg_.decay_en) {
    return false;
  }

  const double keep_time = std::max(0.0, cfg_.keep_time);
  const double clear_time = cfg_.decay_clear_time;
  const double inv_decay_span = 1.0 / (clear_time - keep_time);
  const double l_start = static_cast<double>(cfg_.l_max);
  const double l_end = static_cast<double>(std::nextafter(cfg_.l_free, cfg_.l_min));

  bool changed = false;
  int decayed_count = 0;

  auto keepActive = [](const int hash_id, std::vector<int> * next_active) {
    if (next_active != nullptr) {
      next_active->push_back(hash_id);
    }
  };

  auto clearActive = [&](const int hash_id) {
    if (cfg_.decay_active_list_en && hash_id >= 0 &&
        hash_id < static_cast<int>(active_flags_.size())) {
      active_flags_[hash_id] = 0U;
    }
  };

  auto decayOne = [&](const int hash_id, std::vector<int> * next_active) {
    if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) ||
        hash_id >= static_cast<int>(last_hit_time_.size()) ||
        hash_id >= static_cast<int>(last_update_time_.size())) {
      return;
    }

    float & prob = occupancy_buffer_[hash_id];
    const GridType from_type = classifyProb(prob);
    if (from_type != GridType::OCCUPIED) {
      clearActive(hash_id);
      return;
    }

    const double last_hit = static_cast<double>(last_hit_time_[hash_id]);
    if (!(last_hit > 0.0) || now <= last_hit) {
      keepActive(hash_id, next_active);
      return;
    }

    const double age = now - last_hit;
    if (age <= keep_time) {
      keepActive(hash_id, next_active);
      return;
    }

    const double old_log = static_cast<double>(prob);
    double new_log = old_log;
    if (age >= clear_time) {
      new_log = l_end;
    } else {
      const double ratio = std::clamp((age - keep_time) * inv_decay_span, 0.0, 1.0);
      const double target_log = (1.0 - ratio) * l_start + ratio * l_end;
      new_log = std::min(old_log, target_log);
    }
    new_log =
      std::clamp(new_log, static_cast<double>(cfg_.l_min), static_cast<double>(cfg_.l_max));

    if (new_log < old_log - 1.0e-6) {
      prob = static_cast<float>(new_log);
      last_update_time_[hash_id] = now;

      const GridType to_type = classifyProb(prob);
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      updateCellState(pos, from_type, to_type);
      Vec3i id_g;
      posToGlobalIndex(pos, id_g);
      markDirtyColumn(id_g);

      changed = true;
      ++decayed_count;

      if (to_type == GridType::OCCUPIED) {
        keepActive(hash_id, next_active);
      } else {
        clearActive(hash_id);
      }
      return;
    }

    keepActive(hash_id, next_active);
  };

  if (cfg_.decay_active_list_en) {
    std::vector<int> next_active;
    next_active.reserve(active_ids_.size());
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(active_flags_.size()) || !active_flags_[hash_id]) {
        continue;
      }
      decayOne(hash_id, &next_active);
    }
    active_ids_.swap(next_active);
  } else {
    for (int hash_id = 0; hash_id < static_cast<int>(occupancy_buffer_.size()); ++hash_id) {
      if (classifyProb(occupancy_buffer_[hash_id]) == GridType::OCCUPIED) {
        decayOne(hash_id, nullptr);
      }
    }
  }

  runtime_stats_.decayed_count = decayed_count;
  return changed;
}

void ProbMap::raycastProcess(const PointCloud & input_cloud, const Vec3f & cur_odom)
{
#ifdef _OPENMP
  if (cfg_.parallel_raycast_en && cfg_.raycasting_en && cfg_.raycast_num_threads > 1 &&
      input_cloud.size() > static_cast<size_t>(cfg_.raycast_num_threads * 16)) {
    raycastProcessParallel(input_cloud, cur_odom);
    return;
  }
#endif
  raycastProcessSerial(input_cloud, cur_odom);
}

void ProbMap::raycastProcessSerial(const PointCloud & input_cloud, const Vec3f & cur_odom)
{
  runtime_stats_.raycast_input_point_count = static_cast<double>(input_cloud.size());
  // bounding box of updated region
  raycast_data_.cache_box_min = cur_odom;
  raycast_data_.cache_box_max = cur_odom;
  Vec3f raycast_box_min, raycast_box_max;

  {
    std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
    raycast_box_max = raycast_data_.local_update_box_max;
    raycast_box_min = raycast_data_.local_update_box_min;
  }

  /// Step 1; Raycast and add to update cache.
  const int & cloud_in_size = input_cloud.size();
  // new version of raycasting process
  auto raycasting_cloud = vec_Vec3f{};
  raycasting_cloud.reserve(cloud_in_size);

  // 1) process all non-inf points, update occupied probability
  int temperol_cnt{0};
  for (const auto & pcl_p : input_cloud) {
    // 1.1) intensity filter
    if (cfg_.intensity_thresh > 0 && pcl_p.intensity < cfg_.intensity_thresh) {
      continue;
    }

    // 1.2) temporal filter
    if (temperol_cnt++ % cfg_.point_filt_num) {
      continue;
    }

    Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
    Vec3i pt_id_g;

    // no raycasting, purely add occ pints
    if (!cfg_.raycasting_en) {
      if (insideLocalMap(p)) {
        double sqrdis = (p - cur_odom).squaredNorm();
        if (sqrdis < cfg_.sqr_raycast_range_min) {
          runtime_stats_.raycast_skipped_near_count += 1.0;
          continue;
        }
        posToGlobalIndex(p, pt_id_g);
        insertUpdateCandidate(pt_id_g, true);
        runtime_stats_.raycast_used_point_count += 1.0;
        // record cache box size;
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
      }
      continue;
    }

    bool update_hit{true};
    // 1.3) filter for virtual ceil and ground
    if (p.z() > cfg_.virtual_ceil_height) {
      update_hit = false;
      // find the intersect point with the ceil
      const double dz = p.z() - cur_odom.z();
      const double pc = cfg_.virtual_ceil_height - cur_odom.z();
      p = cur_odom + (p - cur_odom).normalized() * pc / dz;
    } else if (p.z() < cfg_.virtual_ground_height) {
      update_hit = false;
      // find the intersect point with the ground
      const double dz = p.z() - cur_odom.z();
      const double pc = cfg_.virtual_ground_height - cur_odom.z();
      p = cur_odom + (p - cur_odom).normalized() * pc / dz;
    }

    // 1.4) bounding box filter
    // raycasting max
    const double sqr_dis = (p - cur_odom).squaredNorm();
    if (sqr_dis > cfg_.sqr_raycast_range_max) {
      double k = cfg_.raycast_range_max / sqrt(sqr_dis);
      p = k * (p - cur_odom) + cur_odom;
      update_hit = false;
      runtime_stats_.raycast_skipped_far_count += 1.0;
    }

    if (sqr_dis < cfg_.sqr_raycast_range_min) {
      runtime_stats_.raycast_skipped_near_count += 1.0;
      continue;
    }

    // local map bound
    if (((p - raycast_box_min).minCoeff() < 0) || ((p - raycast_box_max).maxCoeff() > 0)) {
      p = lineBoxIntersectPoint(p, cur_odom, raycast_box_min, raycast_box_max);
      update_hit = false;
      runtime_stats_.raycast_skipped_outside_count += 1.0;
    }

    // record cache box size;
    raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
    raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);

    // 1.4) for all validate hit points, update probability
    raycasting_cloud.push_back(p);
    runtime_stats_.raycast_used_point_count += 1.0;

    if (update_hit) {
      posToGlobalIndex(p, pt_id_g);
      insertUpdateCandidate(pt_id_g, true);
    }
  }

  if (cfg_.raycasting_en) {
    // 4) process all inf points, updae free probability
    for (const auto & p : raycasting_cloud) {
      Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
      raycast_data_.raycaster.setInput(raycast_start, p);
      Vec3f ray_pt;
      while (raycast_data_.raycaster.step(ray_pt)) {
        Vec3i cur_ray_id_g;
        posToGlobalIndex(ray_pt, cur_ray_id_g);
        if (!insideLocalMap(cur_ray_id_g)) {
          break;
        }
        insertUpdateCandidate(cur_ray_id_g, false);
      }
    }
  }
}

void ProbMap::raycastProcessParallel(const PointCloud & input_cloud, const Vec3f & cur_odom)
{
  runtime_stats_.raycast_input_point_count = static_cast<double>(input_cloud.size());
  raycast_data_.cache_box_min = cur_odom;
  raycast_data_.cache_box_max = cur_odom;
  Vec3f raycast_box_min, raycast_box_max;

  {
    std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
    raycast_box_max = raycast_data_.local_update_box_max;
    raycast_box_min = raycast_data_.local_update_box_min;
  }

  vec_E<Vec3f> raycasting_cloud;
  vec_E<Vec3i> hit_ids;
  raycasting_cloud.reserve(input_cloud.size());
  hit_ids.reserve(input_cloud.size());

  int temperol_cnt{0};
  for (const auto & pcl_p : input_cloud) {
    if (cfg_.intensity_thresh > 0 && pcl_p.intensity < cfg_.intensity_thresh) {
      continue;
    }
    if (temperol_cnt++ % cfg_.point_filt_num) {
      continue;
    }

    Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
    Vec3i pt_id_g;

    if (!cfg_.raycasting_en) {
      if (insideLocalMap(p)) {
        const double sqrdis = (p - cur_odom).squaredNorm();
        if (sqrdis < cfg_.sqr_raycast_range_min) {
          runtime_stats_.raycast_skipped_near_count += 1.0;
          continue;
        }
        posToGlobalIndex(p, pt_id_g);
        hit_ids.push_back(pt_id_g);
        runtime_stats_.raycast_used_point_count += 1.0;
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
      }
      continue;
    }

    bool update_hit{true};
    if (p.z() > cfg_.virtual_ceil_height) {
      update_hit = false;
      const double dz = p.z() - cur_odom.z();
      const double pc = cfg_.virtual_ceil_height - cur_odom.z();
      p = cur_odom + (p - cur_odom).normalized() * pc / dz;
    } else if (p.z() < cfg_.virtual_ground_height) {
      update_hit = false;
      const double dz = p.z() - cur_odom.z();
      const double pc = cfg_.virtual_ground_height - cur_odom.z();
      p = cur_odom + (p - cur_odom).normalized() * pc / dz;
    }

    const double sqr_dis = (p - cur_odom).squaredNorm();
    if (sqr_dis > cfg_.sqr_raycast_range_max) {
      const double k = cfg_.raycast_range_max / sqrt(sqr_dis);
      p = k * (p - cur_odom) + cur_odom;
      update_hit = false;
      runtime_stats_.raycast_skipped_far_count += 1.0;
    }
    if (sqr_dis < cfg_.sqr_raycast_range_min) {
      runtime_stats_.raycast_skipped_near_count += 1.0;
      continue;
    }

    if (((p - raycast_box_min).minCoeff() < 0) || ((p - raycast_box_max).maxCoeff() > 0)) {
      p = lineBoxIntersectPoint(p, cur_odom, raycast_box_min, raycast_box_max);
      update_hit = false;
      runtime_stats_.raycast_skipped_outside_count += 1.0;
    }

    raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
    raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
    raycasting_cloud.push_back(p);
    runtime_stats_.raycast_used_point_count += 1.0;

    if (update_hit) {
      posToGlobalIndex(p, pt_id_g);
      hit_ids.push_back(pt_id_g);
    }
  }

  const auto parallel_start = std::chrono::steady_clock::now();
  int num_threads = 1;
#ifdef _OPENMP
  num_threads = std::max(1, cfg_.raycast_num_threads);
#endif
  std::vector<RaycastLocalBuffer> local_buffers(static_cast<size_t>(num_threads));

#ifdef _OPENMP
#pragma omp parallel num_threads(num_threads)
  {
    const int tid = omp_get_thread_num();
    auto & local = local_buffers[static_cast<size_t>(tid)];
    raycaster::RayCaster local_raycaster;
    local_raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;

#pragma omp for schedule(dynamic, 32)
    for (int i = 0; i < static_cast<int>(raycasting_cloud.size()); ++i) {
      const Vec3f & p = raycasting_cloud[static_cast<size_t>(i)];
      const Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
      if (!local_raycaster.setInput(raycast_start, p)) {
        continue;
      }
      while (local_raycaster.step(ray_pt)) {
        Vec3i cur_ray_id_g;
        posToGlobalIndex(ray_pt, cur_ray_id_g);
        if (!insideLocalMap(cur_ray_id_g)) {
          break;
        }
        local.miss_ids.push_back(cur_ray_id_g);
      }
    }
  }
#else
  (void)local_buffers;
#endif
  runtime_stats_.raycast_parallel_time = elapsedMs(parallel_start);

  const auto merge_start = std::chrono::steady_clock::now();
  vec_E<Vec3i> miss_ids;
  size_t miss_count = 0;
  for (const auto & local : local_buffers) {
    miss_count += local.miss_ids.size();
  }
  miss_ids.reserve(miss_count);
  for (const auto & local : local_buffers) {
    miss_ids.insert(miss_ids.end(), local.miss_ids.begin(), local.miss_ids.end());
  }

  uniqueVec3i(hit_ids);
  uniqueVec3i(miss_ids);

  for (const auto & id_g : hit_ids) {
    insertUpdateCandidate(id_g, true);
  }
  for (const auto & id_g : miss_ids) {
    if (!std::binary_search(hit_ids.begin(), hit_ids.end(), id_g, vec3iLess)) {
      insertUpdateCandidate(id_g, false);
    }
  }
  runtime_stats_.raycast_merge_time = elapsedMs(merge_start);
}

void ProbMap::insertUpdateCandidate(const Vec3i & id_g, bool is_hit)
{
  const auto & hash_id = getHashIndexFromGlobalIndex(id_g);
  raycast_data_.operation_cnt[hash_id]++;
  if (raycast_data_.operation_cnt[hash_id] == 1) {
    raycast_data_.update_cache_id_g.push(id_g);
  }
  if (is_hit) {
    raycast_data_.hit_cnt[hash_id]++;
    runtime_stats_.hit_count += 1.0;
  } else {
    runtime_stats_.miss_count += 1.0;
  }
}

void ProbMap::markDirtyColumn(const Vec3i & id_g)
{
  if (dirty_column_flags_.empty()) {
    full_layer_refresh_required_ = true;
    return;
  }
  if (!insideLocalMap(id_g)) {
    full_layer_refresh_required_ = true;
    return;
  }
  const Vec3i min_id = local_map_bound_min_i_;
  const int lx = id_g.x() - min_id.x();
  const int ly = id_g.y() - min_id.y();
  if (lx < 0 || ly < 0 || lx >= sc_.map_size_i.x() || ly >= sc_.map_size_i.y()) {
    full_layer_refresh_required_ = true;
    return;
  }
  const int column_id = ly * sc_.map_size_i.x() + lx;
  if (column_id < 0 || column_id >= static_cast<int>(dirty_column_flags_.size())) {
    full_layer_refresh_required_ = true;
    return;
  }
  if (!dirty_column_flags_[column_id]) {
    dirty_column_flags_[column_id] = 1U;
    dirty_column_ids_.push_back(column_id);
  }
}

void ProbMap::clearDirtyColumns()
{
  for (const int column_id : dirty_column_ids_) {
    if (column_id >= 0 && column_id < static_cast<int>(dirty_column_flags_.size())) {
      dirty_column_flags_[column_id] = 0U;
    }
  }
  dirty_column_ids_.clear();
  full_layer_refresh_required_ = false;
}

void ProbMap::markAllDirtyColumns()
{
  full_layer_refresh_required_ = true;
  dirty_column_ids_.clear();
  std::fill(dirty_column_flags_.begin(), dirty_column_flags_.end(), 0U);
}

void ProbMap::updateLocalBox(const Vec3f & cur_odom)
{
  /* This function is only used for decide the local update range
   * and do not related to the map origin and map bound
   * the map origin and map bound is only update in [SlidingMap::mapSliding(const Vec3f &odom)]
   * */
  // The local map should be inside in index wise
  // update: the virtual floor and ceil should not influence the raycasting.
  // 2) local map size
  // The update box should follow odom.
  // The local map should follow current map center.
  std::lock_guard<std::mutex> lck(raycast_data_.raycast_range_mtx);

  Vec3i cur_odom_i;
  posToGlobalIndex(cur_odom, cur_odom_i);
  Vec3i local_updatebox_min_i, local_updatebox_max_i;

  if (cfg_.raycasting_en) {
    local_updatebox_max_i = cur_odom_i + cfg_.half_local_update_box_i;
    local_updatebox_min_i = cur_odom_i - cfg_.half_local_update_box_i;
  }

  globalIndexToPos(local_updatebox_min_i, raycast_data_.local_update_box_min);
  globalIndexToPos(local_updatebox_max_i, raycast_data_.local_update_box_max);

  // the local update box must inside the local map
  raycast_data_.local_update_box_max = raycast_data_.local_update_box_max.cwiseMin(local_map_bound_max_d_);
  raycast_data_.local_update_box_min = raycast_data_.local_update_box_min.cwiseMax(local_map_bound_min_d_);
}

void ProbMap::resetLocalMap()
{
  std::cout << YELLOW << " -- [Prob-Map] Clear all local map." << RESET << std::endl;
  double unk_value = (cfg_.l_free + cfg_.l_occ) / 2.0;
  // Clear local map
  std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), unk_value);
  std::fill(last_hit_time_.begin(), last_hit_time_.end(), 0.0f);
  std::fill(last_update_time_.begin(), last_update_time_.end(), 0.0f);
  std::fill(active_flags_.begin(), active_flags_.end(), 0U);
  active_ids_.clear();
  markAllDirtyColumns();
  while (!raycast_data_.update_cache_id_g.empty()) {
    raycast_data_.update_cache_id_g.pop();
  }
  raycast_data_.batch_update_counter = 0;
  std::fill(raycast_data_.operation_cnt.begin(), raycast_data_.operation_cnt.end(), 0);
  std::fill(raycast_data_.hit_cnt.begin(), raycast_data_.hit_cnt.end(), 0);
}
