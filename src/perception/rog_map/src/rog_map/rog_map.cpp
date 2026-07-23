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

#include "rog_map/rog_map.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <rog_map/map_registry.hpp>

using namespace rog_map;
using namespace super_utils;

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

InterpolationMode parseInterpolationMode(const std::string & mode)
{
  if (mode == "quadratic" || mode == "QUADRATIC") {
    return InterpolationMode::QUADRATIC;
  }
  return InterpolationMode::BILINEAR;
}

}  // namespace

void ROGMap::init()
{
  initProbMap();

  layer_ = std::make_shared<ProjectionLayer>();
  field_ = std::make_shared<DynamicLayer>();
  query_ = std::make_shared<QueryAdapter>();
  performance_monitor_ = std::make_unique<PerformanceMonitor>();
  PerformanceConfig perf_cfg;
  perf_cfg.enable = cfg_.performance_enable;
  perf_cfg.detailed_csv_enable = cfg_.performance_detailed_csv_enable;
  perf_cfg.detailed_csv_path = cfg_.performance_detailed_csv_path;
  perf_cfg.summary_csv_enable = cfg_.performance_summary_csv_enable;
  perf_cfg.summary_csv_path = cfg_.performance_summary_csv_path;
  perf_cfg.run_id = cfg_.performance_run_id;
  perf_cfg.scenario = cfg_.performance_scenario;
  perf_cfg.variant = cfg_.performance_variant;
  perf_cfg.csv_flush_every_n = cfg_.performance_csv_flush_every_n;
  perf_cfg.print_enable = cfg_.performance_print_enable;
  perf_cfg.summary_rate = cfg_.performance_summary_rate;
  performance_monitor_->configure(perf_cfg);
  MapRegistry::set(query_);

  robot_state_.p = cfg_.fix_map_origin;

  if (cfg_.map_sliding_en) {
    slideAllMap(Vec3f(0, 0, 0));
  } else {
    /// if disable map sliding, fix map origin to (0,0,0)
    /// update the local map bound as
    local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
    local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
    slideAllMap(cfg_.fix_map_origin);
  }

  if (cfg_.load_pcd_en) {
    string pcd_path = cfg_.pcd_name;
    PointCloud::Ptr pcd_map(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
      cout << YELLOW << "Load pcd file at: [" << cfg_.pcd_name << "] failed!" << RESET << endl;
      exit(-1);
    }
    Pose cur_pose;
    cur_pose.first = Vec3f(0, 0, 0);
    updateOccPointCloud(*pcd_map);
    if (cfg_.esdf_en) {
      esdf_map_->updateESDF3D(robot_state_.p);
    }
    refreshLayers();
    refreshQuery();
    cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET
         << endl;
    map_empty_ = false;
  }
}

bool ROGMap::findNearestCellThat(const bool & is,
  const GridType & target_type,
  const Vec3f & start_pos,
  Vec3f & nearest_pt,
  const double & max_dis) const
{
  Vec3i start_id;
  posToGlobalIndex(start_pos, start_id);
  nearest_pt.setConstant(NAN);

  for (const auto & nei_id : cfg_.spherical_neighbor) {
    const Vec3i q_id = start_id + nei_id;
    Vec3f q_pos;
    globalIndexToPos(q_id, q_pos);
    if ((q_pos - start_pos).norm() > max_dis) {
      return false;
    }

    if ((getGridType(q_pos) == target_type) == is) {
      nearest_pt = q_pos;
      return true;
    }
  }

  return false;
}

bool ROGMap::findNearestInfCellThat(const bool & is,
  const GridType & target_type,
  const Vec3f & start_pos,
  Vec3f & nearest_pt,
  const double & max_dis) const
{
  Vec3i start_id;
  posToGlobalIndex(start_pos, start_id);
  nearest_pt.setConstant(NAN);

  for (const auto & nei_id : cfg_.spherical_neighbor) {
    const Vec3i q_id = start_id + nei_id;
    Vec3f q_pos;
    globalIndexToPos(q_id, q_pos);
    if ((q_pos - start_pos).norm() > max_dis) {
      return false;
    }

    if ((getInfGridType(q_pos) == target_type) == is) {
      nearest_pt = q_pos;
      return true;
    }
  }
  fmt::print(fg(fmt::color::yellow),
    " -- [ROGMap] findNearestInfCellThat failed to find all {} neighbors at start_pos: {}, target_type: "
    "{}, is: {}\n",
    cfg_.spherical_neighbor.size(),
    start_pos.transpose(),
    target_type,
    is);
  return false;
}

bool ROGMap::isLineFree(const rog_map::Vec3f & start_pt,
  const rog_map::Vec3f & end_pt,
  const bool & use_inf_map,
  const bool & use_unk_as_occ) const
{
  if (start_pt.array().isNaN().any() || end_pt.array().isNaN().any()) {
    cout << YELLOW << " -- [ROGMap] Call isLineFree with NaN in start or end pt, return false." << RESET
         << endl;
    return false;
  }
  raycaster::RayCaster raycaster;
  if (use_inf_map) {
    raycaster.setResolution(cfg_.inflation_resolution);
  } else {
    raycaster.setResolution(cfg_.resolution);
  }
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (!use_unk_as_occ) {
      // allow both unk and free
      if (use_inf_map) {
        if (isOccupiedInflate(ray_pt)) {
          return false;
        }
      } else {
        if (isOccupied(ray_pt)) {
          return false;
        }
      }
    } else {
      // only allow known free
      if (use_inf_map) {
        if ((isUnknownInflate(ray_pt) || isOccupiedInflate(ray_pt)))
          return false;
      } else {
        if (!isKnownFree(ray_pt)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f & start_pt,
  const Vec3f & end_pt,
  const double & max_dis,
  const vec_Vec3i & neighbor_list) const
{
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
    } else {
      Vec3i ray_pt_id_g;
      posToGlobalIndex(ray_pt, ray_pt_id_g);
      for (const auto & nei : neighbor_list) {
        Vec3i shift_tmp = ray_pt_id_g + nei;
        if (isOccupied(shift_tmp)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f & start_pt,
  const Vec3f & end_pt,
  Vec3f & free_local_goal,
  const double & max_dis,
  const vec_Vec3i & neighbor_list) const
{
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  free_local_goal = start_pt;
  while (raycaster.step(ray_pt)) {
    free_local_goal = ray_pt;
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
    } else {
      Vec3i ray_pt_id_g;
      posToGlobalIndex(ray_pt, ray_pt_id_g);
      for (const auto & nei : neighbor_list) {
        Vec3i shift_tmp = ray_pt_id_g + nei;
        if (isOccupied(shift_tmp)) {
          return false;
        }
      }
    }
  }
  free_local_goal = end_pt;
  return true;
}

void ROGMap::updateMap(const PointCloud & cloud, const Pose & pose)
{
  TimeConsuming ssss("updateMap", true);
  if (cfg_.ros_callback_en) {
    std::cout << YELLOW << "ROS callback is enabled, can not insert map from updateMap API." << RESET
              << std::endl;
    return;
  }

  if (cloud.empty()) {
    static int local_cnt = 0;
    if (local_cnt++ > 100) {
      cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
      local_cnt = 0;
    }
    return;
  }

  updateMapInternal(cloud, pose);
}

void ROGMap::updateMapInternal(const PointCloud & cloud, const Pose & pose)
{
  const Pose & sensor_pose = pose;
  Vec3f map_center_pos = sensor_pose.first;
  if (cfg_.map_center_offset_enable) {
    const double yaw = get_yaw_from_quaternion<double>(sensor_pose.second);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Vec3f offset_world;
    offset_world.x() = c * cfg_.map_center_offset.x() - s * cfg_.map_center_offset.y();
    offset_world.y() = s * cfg_.map_center_offset.x() + c * cfg_.map_center_offset.y();
    offset_world.z() = cfg_.map_center_offset.z();
    map_center_pos += offset_world;
  }
  Pose map_center_pose = sensor_pose;
  map_center_pose.first = map_center_pos;

  const auto total_start = std::chrono::steady_clock::now();
  const double update_stamp = getSystemWalltimeNow();
  static uint64_t update_sequence = 0;
  const uint64_t this_update_sequence = ++update_sequence;
  const uint64_t projection_sequence_before = projection_sequence_;
  const uint64_t mask_sequence_before = mask_sequence_;
  const uint64_t field_sequence_before = field_sequence_;
  updateRobotState(map_center_pose);
  const double update_robot_state_ms = elapsedMs(total_start);
  const double now = getSystemWalltimeNow();
  setUpdateTime(now);
  updateProbMap(cloud, sensor_pose, map_center_pos);
  runtime_stats_.stamp = update_stamp;
  runtime_stats_.update_seq = static_cast<double>(this_update_sequence);
  runtime_stats_.update_robot_state_time = update_robot_state_ms;
  runtime_stats_.projection_sequence_delta =
    static_cast<double>(projection_sequence_ - projection_sequence_before);
  runtime_stats_.mask_sequence_delta = static_cast<double>(mask_sequence_ - mask_sequence_before);
  runtime_stats_.field_sequence_delta = static_cast<double>(field_sequence_ - field_sequence_before);
  if (performance_monitor_) {
    performance_monitor_->fillInputStats(runtime_stats_);
  }

  bool decay_changed = false;
  if (cfg_.decay_en) {
    const auto decay_start = std::chrono::steady_clock::now();
    decay_changed = applyDecay(now);
    runtime_stats_.decay_time = elapsedMs(decay_start);
  }
  if (decay_changed && cfg_.esdf_en) {
    esdf_map_->updateESDF3D(robot_state_.p);
  }

  const auto layers_start = std::chrono::steady_clock::now();
  refreshLayers();
  runtime_stats_.refresh_layers_time = elapsedMs(layers_start);
  const auto query_start = std::chrono::steady_clock::now();
  refreshQuery();
  runtime_stats_.query_refresh_time = elapsedMs(query_start);
  runtime_stats_.projection_sequence_delta =
    static_cast<double>(projection_sequence_ - projection_sequence_before);
  runtime_stats_.mask_sequence_delta = static_cast<double>(mask_sequence_ - mask_sequence_before);
  runtime_stats_.field_sequence_delta = static_cast<double>(field_sequence_ - field_sequence_before);
  runtime_stats_.total_update_time = elapsedMs(total_start);
  runtime_stats_.cpu_thread_hint = static_cast<double>(std::max(1U, std::thread::hardware_concurrency()));
  if (performance_monitor_) {
    performance_monitor_->stats() = runtime_stats_;
    performance_monitor_->observeUpdate(runtime_stats_);
  }
}

void ROGMap::refreshLayers()
{
  const auto config_start = std::chrono::steady_clock::now();
  // 从三维概率占据地图按 xy 列生成二维 layer，并把 layer mask 作为 field/ESDF 的障碍输入。
  // scan_z_min_abs/scan_z_max_abs 是 ROGMap frame 中的绝对 Z 坐标，不是相对地面的高度。
  if (!cfg_.layer_en || !layer_) {
    fused_projection_mask_.clear();
    fused_projection_values_.clear();
    prior_projection_mask_.clear();
    runtime_stats_.projection_refresh_reason = "layer_disabled";
    return;
  }

  ProjectionLayerConfig layer_cfg;
  layer_cfg.unknown_as_occupied = cfg_.unknown_as_occupied;
  layer_cfg.min_observed_voxels = cfg_.min_observed_voxels;
  layer_cfg.surface_height_delta_max = cfg_.surface_height_delta_max;
  layer_cfg.wall_height_delta_min = cfg_.wall_height_delta_min;
  layer_cfg.wall_occupancy_ratio_min = cfg_.wall_occupancy_ratio_min;
  layer_cfg.tunnel_height_delta_min = cfg_.tunnel_height_delta_min;
  layer_cfg.tunnel_height_delta_max = cfg_.tunnel_height_delta_max;
  layer_cfg.tunnel_occupancy_ratio_max = cfg_.tunnel_occupancy_ratio_max;
  layer_cfg.passable_cost = static_cast<uint8_t>(std::clamp(cfg_.passable_cost, 0, 252));
  layer_cfg.passable_as_free = cfg_.passable_as_free;
  layer_cfg.hysteresis_en = cfg_.layer_hysteresis_en;
  layer_cfg.hysteresis_count = cfg_.layer_hysteresis_count;
  layer_cfg.obstacle_hold_time = cfg_.layer_obstacle_hold_time;
  layer_cfg.mask_filter_en = cfg_.layer_mask_filter_en;
  layer_cfg.fill_occ_min = cfg_.layer_fill_occ_min;
  layer_cfg.denoise_occ_max = cfg_.layer_denoise_occ_max;

  const int width = mapWidth();
  const int height = mapHeight();
  const double res = getResolution();
  const Vec3i min_id = localMapMinIndex();
  const Vec3f min_pos = localMapMinPosition();
  const Eigen::Vector2d origin(min_pos.x() - 0.5 * res, min_pos.y() - 0.5 * res);

  int z_min = 0;
  int z_max = 0;
  posToGlobalIndex(cfg_.scan_z_min_abs, z_min);
  posToGlobalIndex(cfg_.scan_z_max_abs, z_max);
  if (z_min > z_max) {
    std::swap(z_min, z_max);
  }
  z_min = std::max(z_min, localMapMinIndex().z());
  z_max = std::min(z_max, localMapMaxIndex().z());

  const std::vector<uint8_t> old_fused_mask = fused_projection_mask_;
  const ProjectionSlideResult slide_result = layer_->syncSlidingWindow(
    width, height, res, Eigen::Vector2i(min_id.x(), min_id.y()), origin, layer_cfg);
  const bool geometry_changed = slide_result.full_refresh_required;
  const size_t cell_count =
    static_cast<size_t>(std::max(0, width)) * static_cast<size_t>(std::max(0, height));
  const auto & dirty_columns = dirtyColumnIds();
  std::vector<int> projection_dirty_columns = dirty_columns;
  projection_dirty_columns.insert(
    projection_dirty_columns.end(), slide_result.dirty_columns.begin(), slide_result.dirty_columns.end());
  std::sort(projection_dirty_columns.begin(), projection_dirty_columns.end());
  projection_dirty_columns.erase(
    std::unique(projection_dirty_columns.begin(), projection_dirty_columns.end()),
    projection_dirty_columns.end());
  const double dirty_ratio = std::clamp(cfg_.dirty_full_ratio, 0.0, 1.0);
  const bool dirty_over_ratio =
    cell_count > 0 &&
    projection_dirty_columns.size() > static_cast<size_t>(dirty_ratio * static_cast<double>(cell_count));
  const bool explicit_full_refresh = fullLayerRefreshRequired() && !slide_result.window_moved;
  const bool force_full_refresh =
    geometry_changed || explicit_full_refresh || !cfg_.dirty_column_en || dirty_over_ratio;
  const bool has_dirty_update =
    cfg_.dirty_column_en && !projection_dirty_columns.empty() && !force_full_refresh;
  runtime_stats_.projection_config_time = elapsedMs(config_start);
  runtime_stats_.projection_cell_count = static_cast<double>(cell_count);
  runtime_stats_.projection_z_min_id = static_cast<double>(z_min);
  runtime_stats_.projection_z_max_id = static_cast<double>(z_max);
  runtime_stats_.projection_z_layers = static_cast<double>(std::max(0, z_max - z_min + 1));
  runtime_stats_.projection_geometry_changed = geometry_changed ? 1.0 : 0.0;
  runtime_stats_.projection_full_layer_required = fullLayerRefreshRequired() ? 1.0 : 0.0;
  runtime_stats_.projection_dirty_column_enabled = cfg_.dirty_column_en ? 1.0 : 0.0;
  runtime_stats_.projection_dirty_over_ratio = dirty_over_ratio ? 1.0 : 0.0;

  const auto projection_start = std::chrono::steady_clock::now();
  auto rawGridType = [this](const Vec3i & id_g) -> GridType {
    if (!insideLocalMap(id_g)) {
      return GridType::OUT_OF_MAP;
    }
    const int hash_id = getHashIndexFromGlobalIndex(id_g);
    if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
      return GridType::OUT_OF_MAP;
    }
    const double ret = occupancy_buffer_[hash_id];
    if (isKnownFree(ret)) {
      return GridType::KNOWN_FREE;
    }
    if (isOccupied(ret)) {
      return GridType::OCCUPIED;
    }
    return GridType::UNKNOWN;
  };

  auto scanner = [this, min_id, z_min, z_max, rawGridType](int mx, int my) {
    ColumnStats stats;
    const int gx = min_id.x() + mx;
    const int gy = min_id.y() + my;
    for (int gz = z_min; gz <= z_max; ++gz) {
      Vec3i id_g(gx, gy, gz);
      GridType gt = rawGridType(id_g);
      if (gt == GridType::OCCUPIED || gt == GridType::KNOWN_FREE) {
        ++stats.observed_count;
        stats.last_update_time = std::max(stats.last_update_time, cellLastUpdateTime(id_g));
      }
      if (gt == GridType::OCCUPIED) {
        ++stats.occupied_count;
        stats.occupied_z_index_min = std::min(stats.occupied_z_index_min, gz);
        stats.occupied_z_index_max = std::max(stats.occupied_z_index_max, gz);
        Vec3f pos;
        globalIndexToPos(id_g, pos);
        stats.occupied_z_min_abs = std::min(stats.occupied_z_min_abs, static_cast<double>(pos.z()));
        stats.occupied_z_max_abs = std::max(stats.occupied_z_max_abs, static_cast<double>(pos.z()));
        stats.last_hit_time = std::max(stats.last_hit_time, cellLastHitTime(id_g));
      }
    }
    return stats;
  };

  ProjectionUpdateStats projection_stats;
  if (force_full_refresh || !cfg_.dirty_column_en) {
    const auto full_start = std::chrono::steady_clock::now();
    layer_->updateFull(
      width, height, res, origin, current_update_time_, layer_cfg, scanner, &projection_stats);
    runtime_stats_.projection_update_full_time = elapsedMs(full_start);
    runtime_stats_.full_layer_refresh_count += 1.0;
    if (geometry_changed) {
      runtime_stats_.projection_refresh_reason = "full_geometry_changed";
    } else if (explicit_full_refresh) {
      runtime_stats_.projection_refresh_reason = "full_required";
    } else if (!cfg_.dirty_column_en) {
      runtime_stats_.projection_refresh_reason = "full_dirty_disabled";
    } else if (dirty_over_ratio) {
      runtime_stats_.projection_refresh_reason = "full_dirty_over_ratio";
    } else {
      runtime_stats_.projection_refresh_reason = "full_required";
    }
  } else if (has_dirty_update) {
    const auto dirty_start = std::chrono::steady_clock::now();
    layer_->updateDirty(
      width,
      height,
      res,
      origin,
      current_update_time_,
      layer_cfg,
      scanner,
      projection_dirty_columns,
      false,
      &projection_stats);
    runtime_stats_.projection_update_dirty_time = elapsedMs(dirty_start);
    runtime_stats_.dirty_layer_update_count += 1.0;
    runtime_stats_.projection_refresh_reason = "dirty_update";
  } else {
    runtime_stats_.projection_no_update_count += 1.0;
    runtime_stats_.projection_refresh_reason = "no_dirty";
  }
  runtime_stats_.projection_time = elapsedMs(projection_start);
  runtime_stats_.projection_total_time = runtime_stats_.projection_time;
  if (projection_stats.update_full_time_ms > 0.0) {
    runtime_stats_.projection_update_full_time = projection_stats.update_full_time_ms;
  }
  if (projection_stats.update_dirty_time_ms > 0.0) {
    runtime_stats_.projection_update_dirty_time = projection_stats.update_dirty_time_ms;
  }
  runtime_stats_.projection_mask_filter_time = projection_stats.mask_filter_time_ms;
  runtime_stats_.projection_value_mask_time = projection_stats.value_mask_time_ms;
  runtime_stats_.projection_thin_surface_count = projection_stats.thin_surface_count;
  runtime_stats_.projection_vertical_wall_count = projection_stats.vertical_wall_count;
  runtime_stats_.projection_hollow_tunnel_count = projection_stats.hollow_tunnel_count;
  runtime_stats_.projection_ambiguous_occupied_count = projection_stats.ambiguous_occupied_count;
  runtime_stats_.projection_empty_column_count = projection_stats.empty_column_count;
  runtime_stats_.projection_insufficient_observation_count =
    projection_stats.insufficient_observation_count;
  runtime_stats_.dirty_column_count = static_cast<double>(projection_dirty_columns.size());
  if (force_full_refresh) {
    runtime_stats_.dirty_expanded_column_count = static_cast<double>(cell_count);
  } else if (has_dirty_update) {
    constexpr int kFilterInfluenceRadius = 2;
    const int dirty_radius = layer_cfg.mask_filter_en ? kFilterInfluenceRadius : 0;
    runtime_stats_.dirty_expanded_column_count = static_cast<double>(std::min(cell_count,
      projection_dirty_columns.size() *
        static_cast<size_t>((2 * dirty_radius + 1) * (2 * dirty_radius + 1))));
  } else {
    runtime_stats_.dirty_expanded_column_count = 0.0;
  }
  const double scanned_columns = force_full_refresh ?
    static_cast<double>(cell_count) :
    (has_dirty_update ? static_cast<double>(projection_dirty_columns.size()) : 0.0);
  runtime_stats_.projection_scanned_voxel_estimate =
    scanned_columns * runtime_stats_.projection_z_layers;

  const bool projection_scanned = force_full_refresh || has_dirty_update;
  const bool time_clear_changed =
    layer_->advanceObstacleClearance(current_update_time_, layer_cfg);
  const bool layer_updated = projection_scanned || time_clear_changed;
  if (layer_updated) {
    ++projection_sequence_;
  }
  if (projection_scanned) {
    clearDirtyColumns();
  }
  if (!projection_scanned && time_clear_changed) {
    runtime_stats_.projection_refresh_reason = "time_clear";
  }

  PriorMapTransform2D prior_transform;
  const PriorMapTransform2D * prior_transform_ptr = nullptr;
  if (cfg_.prior_map_enable && prior_map_.loaded && getPriorMapTransform(prior_transform)) {
    prior_transform_ptr = &prior_transform;
  }
  rebuildFusedProjection(prior_transform_ptr);

  const auto & new_mask = fused_projection_mask_;
  bool mask_changed = slide_result.window_moved || old_fused_mask.size() != new_mask.size();
  if (!mask_changed) {
    mask_changed = !std::equal(old_fused_mask.begin(), old_fused_mask.end(), new_mask.begin());
  }
  if (mask_changed) {
    ++mask_sequence_;
  }
  runtime_stats_.projection_sequence = static_cast<double>(projection_sequence_);
  runtime_stats_.layer_mask_changed = mask_changed ? 1.0 : 0.0;
  runtime_stats_.mask_sequence = static_cast<double>(mask_sequence_);
  runtime_stats_.layer_mask_occupied_count = 0.0;
  runtime_stats_.layer_mask_free_count = 0.0;
  for (const auto mask : new_mask) {
    if (mask == 0U) {
      runtime_stats_.layer_mask_occupied_count += 1.0;
    } else {
      runtime_stats_.layer_mask_free_count += 1.0;
    }
  }
  const auto count_start = std::chrono::steady_clock::now();
  runtime_stats_.occupied_count = 0.0;
  runtime_stats_.unknown_count = 0.0;
  runtime_stats_.passable_count = 0.0;
  runtime_stats_.free_count = 0.0;
  runtime_stats_.layer_value_free_count = 0.0;
  runtime_stats_.layer_value_occupied_count = 0.0;
  runtime_stats_.layer_value_unknown_count = 0.0;
  runtime_stats_.layer_value_passable_count = 0.0;
  runtime_stats_.projection_thin_surface_count = 0.0;
  runtime_stats_.projection_vertical_wall_count = 0.0;
  runtime_stats_.projection_hollow_tunnel_count = 0.0;
  runtime_stats_.projection_ambiguous_occupied_count = 0.0;
  runtime_stats_.projection_empty_column_count = 0.0;
  runtime_stats_.projection_insufficient_observation_count = 0.0;
  for (const auto & cell : layer_->cells()) {
    switch (cell.type) {
    case CellType::OCCUPIED:
      runtime_stats_.occupied_count += 1.0;
      break;
    case CellType::UNKNOWN:
      runtime_stats_.unknown_count += 1.0;
      break;
    case CellType::PASSABLE:
      runtime_stats_.passable_count += 1.0;
      break;
    case CellType::FREE:
      runtime_stats_.free_count += 1.0;
      break;
    }
    switch (cell.raw_reason) {
    case ProjectionClassReason::INSUFFICIENT_OBSERVATION:
      runtime_stats_.projection_insufficient_observation_count += 1.0;
      break;
    case ProjectionClassReason::EMPTY_COLUMN:
      runtime_stats_.projection_empty_column_count += 1.0;
      break;
    case ProjectionClassReason::THIN_SURFACE:
      runtime_stats_.projection_thin_surface_count += 1.0;
      break;
    case ProjectionClassReason::SOLID_VERTICAL_WALL:
      runtime_stats_.projection_vertical_wall_count += 1.0;
      break;
    case ProjectionClassReason::HOLLOW_TUNNEL:
      runtime_stats_.projection_hollow_tunnel_count += 1.0;
      break;
    case ProjectionClassReason::AMBIGUOUS_OCCUPIED:
      runtime_stats_.projection_ambiguous_occupied_count += 1.0;
      break;
    }
  }
  for (const auto value : layer_->values()) {
    if (value == 0U) {
      runtime_stats_.layer_value_free_count += 1.0;
    } else if (value == 254U) {
      runtime_stats_.layer_value_occupied_count += 1.0;
    } else if (value == 255U) {
      runtime_stats_.layer_value_unknown_count += 1.0;
    } else {
      runtime_stats_.layer_value_passable_count += 1.0;
    }
  }
  runtime_stats_.projection_count_cells_time = elapsedMs(count_start);

  static auto last_projection_log = std::chrono::steady_clock::time_point{};
  const auto projection_log_now = std::chrono::steady_clock::now();
  if (performance_monitor_ && performance_monitor_->printEnabled() &&
      (last_projection_log.time_since_epoch().count() == 0 ||
       std::chrono::duration<double>(projection_log_now - last_projection_log).count() >= 1.0)) {
    std::cout << "[ROGMapProjection] projection occupied_count=" << runtime_stats_.occupied_count
              << ", projection free_count=" << runtime_stats_.free_count
              << ", projection passable_count=" << runtime_stats_.passable_count
              << ", projection unknown_count=" << runtime_stats_.unknown_count
              << ", classification: thin=" << runtime_stats_.projection_thin_surface_count
              << ", wall=" << runtime_stats_.projection_vertical_wall_count
              << ", tunnel=" << runtime_stats_.projection_hollow_tunnel_count
              << ", ambiguous=" << runtime_stats_.projection_ambiguous_occupied_count
              << ", empty=" << runtime_stats_.projection_empty_column_count
              << ", insufficient=" << runtime_stats_.projection_insufficient_observation_count
              << ", mask0_count=" << runtime_stats_.layer_mask_occupied_count
              << ", mask1_count=" << runtime_stats_.layer_mask_free_count << std::endl;
    last_projection_log = projection_log_now;
  }

  const bool field_geometry_changed =
    !field_ ||
    !field_->matchesGeometry(layer_->width(), layer_->height(), layer_->resolution(), layer_->origin());
  const bool should_update_field = cfg_.field_en && field_ && !layer_->empty() &&
                                   (layer_updated || mask_changed || field_geometry_changed ||
                                    !field_->isValid());
  runtime_stats_.field_enabled = cfg_.field_en ? 1.0 : 0.0;
  runtime_stats_.field_dirty_before = 0.0;
  runtime_stats_.field_period_ready = 1.0;
  runtime_stats_.field_should_update = should_update_field ? 1.0 : 0.0;
  if (should_update_field) {
    // field 紧随 projection 更新；inflation_radius 会整体减小 ESDF 距离。
    const auto field_start = std::chrono::steady_clock::now();
    FieldBuildStats field_stats;
    field_stale_ = true;
    field_->update(layer_->width(),
      layer_->height(),
      layer_->resolution(),
      layer_->origin(),
      fused_projection_mask_,
      cfg_.field_inflation_radius,
      cfg_.field_max_distance,
      cfg_.field_min_distance,
      cfg_.field_clamp_distance_en,
      parseInterpolationMode(cfg_.field_interpolation),
      &field_stats);
    const double previous_field_update_time = last_field_update_time_;
    last_field_update_time_ = current_update_time_;
    last_field_stamp_ = current_update_time_;
    ++field_sequence_;
    field_stale_ = false;
    runtime_stats_.field_time = elapsedMs(field_start);
    runtime_stats_.field_update_from_mask_time = field_stats.total_time_ms;
    runtime_stats_.field_edt_positive_time = field_stats.edt_positive_time_ms;
    runtime_stats_.field_inverse_mask_time = field_stats.inverse_mask_time_ms;
    runtime_stats_.field_edt_negative_time = field_stats.edt_negative_time_ms;
    runtime_stats_.field_distance_fill_time = field_stats.distance_fill_time_ms;
    runtime_stats_.field_copy_time = field_stats.commit_time_ms;
    runtime_stats_.field_actual_update = 1.0;
    runtime_stats_.field_skip_reason = "none";
    runtime_stats_.field_update_count = 1.0;
    runtime_stats_.field_update_interval_ms =
      std::isfinite(previous_field_update_time)
        ? std::max(0.0, current_update_time_ - previous_field_update_time) * 1000.0
        : 0.0;
    runtime_stats_.field_update_hz_window = runtime_stats_.field_update_interval_ms > 1.0e-6
                                              ? 1000.0 / runtime_stats_.field_update_interval_ms
                                              : 0.0;
  } else {
    runtime_stats_.field_time = 0.0;
    runtime_stats_.field_actual_update = 0.0;
    if (!cfg_.field_en || !field_) {
      runtime_stats_.field_skip_reason = "disabled";
      runtime_stats_.field_skip_disabled_count = 1.0;
    } else if (layer_->empty()) {
      runtime_stats_.field_skip_reason = "layer_empty";
      runtime_stats_.field_skip_layer_empty_count = 1.0;
    } else if (!layer_updated && !field_geometry_changed && field_->isValid()) {
      runtime_stats_.field_skip_reason = "layer_not_updated";
    } else {
      runtime_stats_.field_skip_reason = "unknown";
      field_stale_ = true;
    }
    runtime_stats_.field_skipped_count = 1.0;
  }
  runtime_stats_.field_sequence = static_cast<double>(field_sequence_);
}

void ROGMap::rebuildFusedProjection(const PriorMapTransform2D * transform)
{
  if (!layer_ || layer_->empty()) {
    fused_projection_mask_.clear();
    fused_projection_values_.clear();
    prior_projection_mask_.clear();
    return;
  }

  const PriorMapTransform2D * fused_transform =
    (cfg_.prior_map_enable && prior_map_.loaded) ? transform : nullptr;
  fusePriorMapProjection(prior_map_,
    fused_transform,
    layer_->width(),
    layer_->height(),
    layer_->resolution(),
    layer_->origin().x(),
    layer_->origin().y(),
    layer_->mask(),
    layer_->values(),
    fused_projection_mask_,
    fused_projection_values_,
    &prior_projection_mask_);
}

void ROGMap::refreshQuery()
{
  if (!query_ || !layer_ || layer_->empty()) {
    return;
  }

  const auto alloc_start = std::chrono::steady_clock::now();
  auto snapshot = std::make_shared<MapSnapshot>();
  runtime_stats_.query_snapshot_alloc_time = elapsedMs(alloc_start);
  snapshot->sequence = ++snapshot_sequence_;
  snapshot->snapshot_sequence = snapshot->sequence;
  snapshot->projection_sequence = projection_sequence_;
  snapshot->mask_sequence = mask_sequence_;
  snapshot->stamp = current_update_time_;
  snapshot->width = layer_->width();
  snapshot->height = layer_->height();
  snapshot->resolution = layer_->resolution();
  snapshot->origin_x = layer_->origin().x();
  snapshot->origin_y = layer_->origin().y();
  const auto values_start = std::chrono::steady_clock::now();
  snapshot->values = fused_projection_values_;
  runtime_stats_.query_copy_values_time = elapsedMs(values_start);
  const auto meta_start = std::chrono::steady_clock::now();
  snapshot->types.resize(layer_->cells().size(), 0U);
  snapshot->height_deltas.resize(layer_->cells().size(), 0.0f);
  snapshot->confidence.resize(layer_->cells().size(), 0.0f);
  for (size_t i = 0; i < layer_->cells().size(); ++i) {
    snapshot->types[i] = static_cast<uint8_t>(layer_->cells()[i].type);
    snapshot->height_deltas[i] = layer_->cells()[i].height_delta;
    snapshot->confidence[i] = layer_->cells()[i].confidence;
  }
  runtime_stats_.query_copy_types_height_delta_confidence_time = elapsedMs(meta_start);
  const bool field_valid =
    field_ && field_->isValid() &&
    field_->matchesGeometry(layer_->width(), layer_->height(), layer_->resolution(), layer_->origin()) &&
    !field_stale_;
  snapshot->field_sequence = field_sequence_;
  snapshot->field_stamp = last_field_stamp_;
  if (field_valid) {
    snapshot->distances = field_->distances();
    snapshot->field_stale = false;
    snapshot->field_max_distance = field_->maxDistance();
    snapshot->field_min_distance = field_->minDistance();
    snapshot->field_clamp_distance = field_->clampDistanceEnabled();
    snapshot->interpolation = field_->interpolationMode();
  } else {
    snapshot->distances.clear();
    snapshot->field_stale = true;
    if (field_) {
      snapshot->field_max_distance = field_->maxDistance();
      snapshot->field_min_distance = field_->minDistance();
      snapshot->field_clamp_distance = field_->clampDistanceEnabled();
      snapshot->interpolation = field_->interpolationMode();
    }
  }
  query_->update(snapshot, field_);
}

RobotState ROGMap::getRobotState() const
{
  return robot_state_;
}

void ROGMap::updateRobotState(const Pose & pose)
{
  robot_state_.p = pose.first;
  robot_state_.q = pose.second;
  robot_state_.rcv_time = getSystemWalltimeNow();
  robot_state_.rcv = true;
  robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);
  updateLocalBox(pose.first);
}
