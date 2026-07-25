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

#pragma once

#include <cmath>
#include <stdexcept>

#include <nav2_util/node_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rog_map/rog_map_core/common_lib.hpp>

#ifndef ORIGIN_AT_CORNER
#ifndef ORIGIN_AT_CENTER
#error "Please define either ORIGIN_AT_CORNER or ORIGIN_AT_CENTER, but not both."
#endif
#endif

#ifdef ORIGIN_AT_CORNER
#ifdef ORIGIN_AT_CENTER
#error "Cannot use both ORIGIN_AT_CENTER and ORIGIN_AT_CENTER at the same time. Please define only one."
#endif
#endif

namespace rog_map {
using color_text::BLUE;
using color_text::RED;
using color_text::RESET;
using std::string;
using std::vector;
typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

class Config
{
  static std::string replaceCmakeRootDir(const std::string & input)
  {
#define CMAKE_ROOT_DIR(name) (string(string(ROOT_DIR) + name))
    std::string replaced;
    std::regex cmakeRootDirRegex(R"(\$\{CMAKE_ROOT_DIR\}/)");
    if (std::regex_search(input, cmakeRootDirRegex)) {
      replaced = std::regex_replace(input, cmakeRootDirRegex, "");
      replaced = CMAKE_ROOT_DIR(replaced);
      printf("\033[0;32m The pcd_name is remapped to %s\033[0;0m \n", replaced.c_str());
    } else {
      replaced = input;
    }
    return replaced;
  }

public:
  Config(){};

  template <typename NodeT> void loadFromRosNode(const NodeT & node, const string & prefix)
  {
    auto load = [&node, &prefix](const string & key, auto & value) {
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(value));
      if (!node->get_parameter(param_name, value)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default.",
          param_name.c_str());
      }
    };

    auto loadVec3 = [&node, &prefix](const string & key, const vector<double> & default_value) {
      vector<double> values = default_value;
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(values));
      if (!node->get_parameter(param_name, values)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default vector.",
          param_name.c_str());
      }
      if (values.size() != 3) {
        throw std::invalid_argument(param_name + " size is not 3!");
      }
      return Vec3f(values[0], values[1], values[2]);
    };

    auto loadVec2 = [&node, &prefix](const string & key, const vector<double> & default_value) {
      vector<double> values = default_value;
      const string param_name = prefix + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, param_name, rclcpp::ParameterValue(values));
      if (!node->get_parameter(param_name, values)) {
        RCLCPP_WARN(node->get_logger(),
          "[ROGMap Config] parameter '%s' not found after declaration, using default vector.",
          param_name.c_str());
      }
      if (values.size() != 2) {
        throw std::invalid_argument(param_name + " size is not 2!");
      }
      return values;
    };

    esdf_resolution = 0.2;
    esdf_en = false;
    load("esdf.resolution", esdf_resolution);
    load("esdf.enable", esdf_en);
    esdf_local_update_box = loadVec3("esdf.local_update_box", vector<double>{10.0, 10.0, 2.0});

    load_pcd_en = false;
    pcd_name = "map.pcd";
    load("load_pcd_en", load_pcd_en);
    load("pcd_name", pcd_name);
    if (load_pcd_en) {
      pcd_name = replaceCmakeRootDir(pcd_name);
    }

    map_sliding_en = true;
    map_sliding_thresh = -1.0;
    map_center_offset_enable = false;
    map_center_offset = Vec3f::Zero();
    load("map_sliding.enable", map_sliding_en);
    load("map_sliding.threshold", map_sliding_thresh);
    load("map_sliding.center_offset_enable", map_center_offset_enable);
    vector<double> map_center_offset_values{0.0, 0.0, 0.0};
    load("map_sliding.center_offset", map_center_offset_values);
    if (map_center_offset_values.size() == 3) {
      map_center_offset = Vec3f(
        map_center_offset_values[0], map_center_offset_values[1], map_center_offset_values[2]);
    } else {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] parameter '%s.map_sliding.center_offset' size is %zu, using zero offset.",
        prefix.c_str(), map_center_offset_values.size());
      map_center_offset = Vec3f::Zero();
    }
    fix_map_origin = loadVec3("fix_map_origin", vector<double>{0.0, 0.0, 0.0});
    frontier_extraction_en = false;
    load("frontier_extraction_en", frontier_extraction_en);

    ros_callback_en = false;
    cloud_topic = "/cloud_registered_full";
    odom_topic = "/lidar_slam/odom";
    odom_timeout = 0.05;
    update_period_ms = 1;
    load("ros_callback.enable", ros_callback_en);
    load("ros_callback.cloud_topic", cloud_topic);
    load("ros_callback.odom_topic", odom_topic);
    load("ros_callback.odom_timeout", odom_timeout);
    load("ros_callback.update_period_ms", update_period_ms);
    if (update_period_ms <= 0) {
      update_period_ms = 1;
    }

    visualization_en = false;
    frame_id = "world";
    load("frame_id", frame_id);
    visualization_frame_id = "";
    visualization_rate = 5.0;
    load("visualization.enable", visualization_en);
    load("visualization.frame_id", visualization_frame_id);
    load("visualization.rate", visualization_rate);
    if (visualization_frame_id.empty()) {
      visualization_frame_id = frame_id;
    }
    if (visualization_rate <= 0.0) {
      visualization_rate = 5.0;
    }
    visualization_range = loadVec3("visualization.range", vector<double>{10.0, 10.0, 2.0});
    if (visualization_range.minCoeff() <= 0) {
      visualization_en = false;
    }

    resolution = 0.1;
    inflation_resolution = 0.1;
    unk_inflation_en = false;
    unk_inflation_step = 1;
    inflation_step = 1;
    intensity_thresh = -1;
    map_size_d = Vec3f(10.0, 10.0, 0.0);
    point_filt_num = 2;
    load("resolution", resolution);
    load("inflation_resolution", inflation_resolution);
    load("unk_inflation_en", unk_inflation_en);
    load("unk_inflation_step", unk_inflation_step);
    load("inflation_step", inflation_step);
    load("intensity_thresh", intensity_thresh);
    map_size_d = loadVec3("map_size", vector<double>{10.0, 10.0, 0.0});
    load("point_filt_num", point_filt_num);
    if (point_filt_num <= 0) {
      point_filt_num = 1;
    }

    raycasting_en = true;
    batch_update_size = 1;
    unk_thresh = 0.70;
    p_hit = 0.70f;
    p_miss = 0.70f;
    p_min = 0.12f;
    p_max = 0.97f;
    p_occ = 0.80f;
    p_free = 0.30f;
    parallel_raycast_en = true;
    raycast_num_threads = 4;
    load("raycasting.enable", raycasting_en);
    load("raycasting.batch_update_size", batch_update_size);
    load("raycasting.unk_thresh", unk_thresh);
    load("raycasting.p_hit", p_hit);
    load("raycasting.p_miss", p_miss);
    load("raycasting.p_min", p_min);
    load("raycasting.p_max", p_max);
    load("raycasting.p_occ", p_occ);
    load("raycasting.p_free", p_free);
    load("raycasting.parallel_enable", parallel_raycast_en);
    load("performance.parallel_raycast_enable", parallel_raycast_en);
    load("raycasting.num_threads", raycast_num_threads);
    load("performance.raycast_num_threads", raycast_num_threads);
    if (batch_update_size <= 0) {
      batch_update_size = 1;
    }
    if (raycast_num_threads <= 0) {
      raycast_num_threads = 1;
    }

    layer_en = true;
    scan_z_min_abs = -0.25;
    scan_z_max_abs = 1.50;
    min_observed_voxels = 2;
    unknown_as_occupied = true;
    surface_height_delta_max = 0.10;
    wall_height_delta_min = 0.20;
    wall_occupancy_ratio_min = 0.80;
    tunnel_height_delta_min = 0.24;
    tunnel_height_delta_max = 0.40;
    tunnel_occupancy_ratio_max = 0.55;
    passable_cost = 50;
    passable_as_free = true;
    layer_hysteresis_en = true;
    layer_hysteresis_count = 2;
    layer_obstacle_hold_time = 0.0;
    layer_mask_filter_en = true;
    layer_fill_occ_min = 5;
    layer_denoise_occ_max = 0;
    load("projection.enable", layer_en);
    load("projection.scan_z_min_abs", scan_z_min_abs);
    load("projection.scan_z_max_abs", scan_z_max_abs);
    load("projection.min_observed_voxels", min_observed_voxels);
    load("projection.unknown_as_occupied", unknown_as_occupied);
    load("projection.surface_height_delta_max", surface_height_delta_max);
    load("projection.wall_height_delta_min", wall_height_delta_min);
    load("projection.wall_occupancy_ratio_min", wall_occupancy_ratio_min);
    load("projection.tunnel_height_delta_min", tunnel_height_delta_min);
    load("projection.tunnel_height_delta_max", tunnel_height_delta_max);
    load("projection.tunnel_occupancy_ratio_max", tunnel_occupancy_ratio_max);
    load("projection.passable_cost", passable_cost);
    load("projection.passable_as_free", passable_as_free);
    load("projection.hysteresis_enable", layer_hysteresis_en);
    load("projection.hysteresis_count", layer_hysteresis_count);
    load("projection.obstacle_hold_time", layer_obstacle_hold_time);
    layer_obstacle_hold_time = std::max(0.0, layer_obstacle_hold_time);
    load("projection.mask_filter_en", layer_mask_filter_en);
    load("projection.fill_occ_min", layer_fill_occ_min);
    load("projection.denoise_occ_max", layer_denoise_occ_max);
    prior_map_enable = false;
    prior_map_yaml_path.clear();
    prior_map_pgm_path.clear();
    prior_map_frame = "map";
    load("projection.prior_map.enable", prior_map_enable);
    load("projection.prior_map.yaml_path", prior_map_yaml_path);
    load("projection.prior_map.pgm_path", prior_map_pgm_path);
    load("projection.prior_map.frame_id", prior_map_frame);
    if (prior_map_enable && prior_map_yaml_path.empty()) {
      throw std::invalid_argument(
        "projection.prior_map.yaml_path must not be empty when prior map fusion is enabled.");
    }
    if (prior_map_enable && prior_map_frame.empty()) {
      throw std::invalid_argument(
        "projection.prior_map.frame_id must not be empty when prior map fusion is enabled.");
    }
    if (layer_fill_occ_min < 1 || layer_fill_occ_min > 8 ||
        layer_denoise_occ_max < 0 || layer_denoise_occ_max > 7 ||
        layer_denoise_occ_max >= layer_fill_occ_min) {
      throw std::invalid_argument(
        "projection mask filter requires fill_occ_min in [1, 8], "
        "denoise_occ_max in [0, 7], and denoise_occ_max < fill_occ_min.");
    }
    if (!std::isfinite(scan_z_min_abs) || !std::isfinite(scan_z_max_abs) ||
        scan_z_max_abs <= scan_z_min_abs) {
      throw std::invalid_argument(
        "projection.scan_z_min_abs and projection.scan_z_max_abs must be finite absolute Z "
        "coordinates in the ROGMap frame, and scan_z_max_abs must be greater than scan_z_min_abs.");
    }
    if (!std::isfinite(surface_height_delta_max) || surface_height_delta_max < 0.0) {
      throw std::invalid_argument(
        "projection.surface_height_delta_max must be a non-negative vertical height difference.");
    }
    if (!std::isfinite(wall_height_delta_min) || wall_height_delta_min <= surface_height_delta_max) {
      throw std::invalid_argument(
        "projection.wall_height_delta_min must be a vertical height difference greater than "
        "surface_height_delta_max.");
    }
    if (!std::isfinite(wall_occupancy_ratio_min) || wall_occupancy_ratio_min < 0.0 ||
        wall_occupancy_ratio_min > 1.0) {
      throw std::invalid_argument(
        "projection.wall_occupancy_ratio_min must be a dimensionless ratio in [0, 1].");
    }
    if (!std::isfinite(tunnel_occupancy_ratio_max) || tunnel_occupancy_ratio_max < 0.0 ||
        tunnel_occupancy_ratio_max > 1.0) {
      throw std::invalid_argument(
        "projection.tunnel_occupancy_ratio_max must be a dimensionless ratio in [0, 1].");
    }
    if (tunnel_occupancy_ratio_max >= wall_occupancy_ratio_min) {
      throw std::invalid_argument(
        "projection.tunnel_occupancy_ratio_max must be a dimensionless ratio smaller than "
        "wall_occupancy_ratio_min.");
    }
    if (!std::isfinite(tunnel_height_delta_min) || tunnel_height_delta_min <= surface_height_delta_max) {
      throw std::invalid_argument(
        "projection.tunnel_height_delta_min must be a vertical height difference greater than "
        "surface_height_delta_max.");
    }
    if (!std::isfinite(tunnel_height_delta_max) || tunnel_height_delta_max < tunnel_height_delta_min) {
      throw std::invalid_argument(
        "projection.tunnel_height_delta_max must be a vertical height difference greater than or equal "
        "to tunnel_height_delta_min.");
    }

    field_en = true;
    field_inflation_radius = 0.33;
    field_max_distance = 3.0;
    field_min_distance = -1.0;
    field_clamp_distance_en = true;
    field_interpolation = "bilinear";
    field_update_rate = 20.0;
    load("field.enable", field_en);
    load("field.inflation_radius", field_inflation_radius);
    load("field.max_distance", field_max_distance);
    load("field.min_distance", field_min_distance);
    load("field.clamp_distance", field_clamp_distance_en);
    load("field.interpolation", field_interpolation);
    load("field.update_rate", field_update_rate);
    load("performance.field_update_rate", field_update_rate);
    if (field_max_distance <= 0.0) {
      field_max_distance = 3.0;
    }
    if (field_min_distance > 0.0) {
      field_min_distance = -1.0;
    }

    decay_en = true;
    keep_time = 0.4;
    decay_clear_time = 0.0;
    decay_active_list_en = true;
    dirty_column_en = false;
    dirty_full_ratio = 0.30;
    performance_enable = true;
    performance_detailed_csv_enable = false;
    performance_detailed_csv_path = "/tmp/rog_map_perf_detailed.csv";
    performance_summary_csv_enable = false;
    performance_summary_csv_path = "/tmp/rog_map_perf_summary.csv";
    performance_run_id = "";
    performance_scenario = "";
    performance_variant = "";
    performance_csv_flush_every_n = 30;
    performance_print_enable = false;
    performance_summary_rate = 1.0;
    debug_layer_pub_en = true;
    debug_field_pub_en = true;
    debug_pub_rate = 5.0;
    load("decay.enable", decay_en);
    load("decay.keep_time", keep_time);
    load("decay.clear_time", decay_clear_time);
    if (decay_clear_time <= keep_time) {
      throw std::runtime_error("[ROGMap] decay.clear_time must be greater than decay.keep_time");
    }
    load("decay.active_list_enable", decay_active_list_en);
    load("performance.dirty_column_enable", dirty_column_en);
    load("performance.dirty_full_ratio", dirty_full_ratio);
    load("performance.enable", performance_enable);
    load("performance.detailed_csv_enable", performance_detailed_csv_enable);
    load("performance.detailed_csv_path", performance_detailed_csv_path);
    load("performance.summary_csv_enable", performance_summary_csv_enable);
    load("performance.summary_csv_path", performance_summary_csv_path);
    load("performance.run_id", performance_run_id);
    load("performance.scenario", performance_scenario);
    load("performance.variant", performance_variant);
    load("performance.csv_flush_every_n", performance_csv_flush_every_n);
    load("performance.print_enable", performance_print_enable);
    load("performance.summary_rate", performance_summary_rate);
    load("debug.layer_pub_enable", debug_layer_pub_en);
    load("debug.field_pub_enable", debug_field_pub_en);
    load("debug.pub_rate", debug_pub_rate);

    auto ray_range = loadVec2("raycasting.ray_range", vector<double>{0.3, 10.0});
    raycast_range_min = ray_range[0];
    raycast_range_max = ray_range[1];
    sqr_raycast_range_max = raycast_range_max * raycast_range_max;
    sqr_raycast_range_min = raycast_range_min * raycast_range_min;
    local_update_box_d = loadVec3("raycasting.local_update_box", vector<double>{999.0, 999.0, 999.0});

    virtual_ground_height = -0.80;
    virtual_ceil_height = 1.80;
    load("virtual_ground_height", virtual_ground_height);
    load("virtual_ceil_height", virtual_ceil_height);
    if (virtual_ground_height >= virtual_ceil_height) {
      RCLCPP_WARN(node->get_logger(),
        "[ROGMap Config] invalid virtual height range [%.3f, %.3f], reset to [-0.80, 1.80].",
        virtual_ground_height,
        virtual_ceil_height);
      virtual_ground_height = -0.80;
      virtual_ceil_height = 1.80;
    }

    resetMapSize();

#define logit(x) (log((x) / (1 - (x))))
    l_hit = logit(p_hit);
    l_miss = logit(p_miss);
    l_min = logit(p_min);
    l_max = logit(p_max);
    l_occ = logit(p_occ);
    l_free = logit(p_free);
    inf_spherical_neighbor.clear();
    unk_inf_spherical_neighbor.clear();
    spherical_neighbor.clear();
    for (int dx = -inflation_step; dx <= inflation_step; dx++) {
      for (int dy = -inflation_step; dy <= inflation_step; dy++) {
        for (int dz = -inflation_step; dz <= inflation_step; dz++) {
          if (inflation_step == 1 || dx * dx + dy * dy + dz * dz <= inflation_step * inflation_step) {
            inf_spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(
      inf_spherical_neighbor.begin(), inf_spherical_neighbor.end(), [](const Vec3i & a, const Vec3i & b) {
        return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() <
               b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
      });

    if (unk_inflation_en) {
      for (int dx = -unk_inflation_step; dx <= unk_inflation_step; dx++) {
        for (int dy = -unk_inflation_step; dy <= unk_inflation_step; dy++) {
          for (int dz = -unk_inflation_step; dz <= unk_inflation_step; dz++) {
            if (unk_inflation_step == 1 ||
                dx * dx + dy * dy + dz * dz <= unk_inflation_step * unk_inflation_step) {
              unk_inf_spherical_neighbor.emplace_back(dx, dy, dz);
            }
          }
        }
      }
      std::sort(unk_inf_spherical_neighbor.begin(),
        unk_inf_spherical_neighbor.end(),
        [](const Vec3i & a, const Vec3i & b) {
          return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() <
                 b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
        });
    }

    constexpr double max_search_dis = 5.0;
    const int max_seach_step = ceil(max_search_dis / resolution);
    for (int dx = -max_seach_step; dx <= max_seach_step; dx++) {
      for (int dy = -max_seach_step; dy <= max_seach_step; dy++) {
        for (int dz = -max_seach_step; dz <= max_seach_step; dz++) {
          if (dx * dx + dy * dy + dz * dz <= max_seach_step * max_seach_step) {
            spherical_neighbor.emplace_back(dx, dy, dz);
          }
        }
      }
    }
    std::sort(spherical_neighbor.begin(), spherical_neighbor.end(), [](const Vec3i & a, const Vec3i & b) {
      return a.x() * a.x() + a.y() * a.y() + a.z() * a.z() < b.x() * b.x() + b.y() * b.y() + b.z() * b.z();
    });
    RCLCPP_INFO(node->get_logger(),
      "[ROGMap Config] loaded prefix='%s', frame_id='%s', cloud='%s', "
      "odom='%s', resolution=%.3f, "
      "map_size=[%.2f %.2f %.2f], projection=%s, field=%s/%s, performance=%s, visualization=%s",
      prefix.c_str(),
      frame_id.c_str(),
      cloud_topic.c_str(),
      odom_topic.c_str(),
      resolution,
      map_size_d.x(),
      map_size_d.y(),
      map_size_d.z(),
      layer_en ? "on" : "off",
      field_en ? "on" : "off",
      field_interpolation.c_str(),
      performance_enable ? "on" : "off",
      visualization_en ? "on" : "off");
  }

  // add 24.07.18 add esdf
  bool esdf_en{false};
  Vec3f esdf_local_update_box{};
  double esdf_resolution{};

  bool layer_en{true};
  double scan_z_min_abs{-0.25};
  double scan_z_max_abs{1.50};
  int min_observed_voxels{2};
  bool unknown_as_occupied{true};
  double surface_height_delta_max{0.10};
  double wall_height_delta_min{0.20};
  double wall_occupancy_ratio_min{0.80};
  double tunnel_height_delta_min{0.24};
  double tunnel_height_delta_max{0.40};
  double tunnel_occupancy_ratio_max{0.55};
  int passable_cost{50};
  bool layer_hysteresis_en{true};
  int layer_hysteresis_count{2};
  double layer_obstacle_hold_time{0.0};
  bool layer_mask_filter_en{true};
  int layer_fill_occ_min{5};
  int layer_denoise_occ_max{0};
  bool passable_as_free{true};
  bool prior_map_enable{false};
  string prior_map_yaml_path{};
  string prior_map_pgm_path{};
  string prior_map_frame{"map"};

  bool field_en{true};
  double field_inflation_radius{0.33};
  double field_max_distance{3.0};
  double field_min_distance{-1.0};
  bool field_clamp_distance_en{true};
  string field_interpolation{"bilinear"};

  bool decay_en{true};
  double keep_time{0.4};
  double decay_clear_time{0.0};
  bool decay_active_list_en{true};

  bool parallel_raycast_en{true};
  int raycast_num_threads{4};
  bool dirty_column_en{false};
  double dirty_full_ratio{0.30};
  double field_update_rate{20.0};
  bool performance_enable{true};
  bool performance_detailed_csv_enable{false};
  string performance_detailed_csv_path{"/tmp/rog_map_perf_detailed.csv"};
  bool performance_summary_csv_enable{false};
  string performance_summary_csv_path{"/tmp/rog_map_perf_summary.csv"};
  string performance_run_id{};
  string performance_scenario{};
  string performance_variant{};
  int performance_csv_flush_every_n{30};
  bool performance_print_enable{false};
  double performance_summary_rate{1.0};
  bool debug_layer_pub_en{true};
  bool debug_field_pub_en{true};
  double debug_pub_rate{5.0};

  bool load_pcd_en{false};
  string pcd_name{"map.pcd"};

  double resolution{}, inflation_resolution{};
  int inflation_step{};
  Vec3f local_update_box_d, half_local_update_box_d{};
  Vec3i local_update_box_i, half_local_update_box_i{};
  Vec3f map_size_d, half_map_size_d{};
  Vec3i inf_half_map_size_i{}, half_map_size_i{}, fro_half_map_size_i{};

  /* The inf map virtual ceil should consider the inflation step
   *  the float type ceil and ground height is remap to the lower and upper bound of
   *  the inflation layer respectively
   */
  double virtual_ceil_height{}, virtual_ground_height{};
  int inf_virtual_ceil_height_id_g{}, inf_virtual_ground_height_id_g{};

  bool visualization_en{false}, frontier_extraction_en{false}, raycasting_en{true}, ros_callback_en{false};

  /* Spherical neighbor for inflation*/
  std::vector<Vec3i> inf_spherical_neighbor{};
  std::vector<Vec3i> unk_inf_spherical_neighbor{};
  /* Spherical neighbor for nearest search within x m*/
  std::vector<Vec3i> spherical_neighbor{};

  /* intensity noise filter*/
  int intensity_thresh{};
  /* aster properties */
  string frame_id{};
  bool map_sliding_en{true};
  bool map_center_offset_enable{false};
  Vec3f map_center_offset{Vec3f::Zero()};
  Vec3f fix_map_origin{};
  string odom_topic{}, cloud_topic{};
  int update_period_ms{1};
  /* probability update */
  double raycast_range_min{}, raycast_range_max{};
  double sqr_raycast_range_min{}, sqr_raycast_range_max{};
  int point_filt_num{}, batch_update_size{};
  float p_hit{}, p_miss{}, p_min{}, p_max{}, p_occ{}, p_free{};
  float l_hit{}, l_miss{}, l_min{}, l_max{}, l_occ{}, l_free{};

  /* for unknown inflation */
  bool unk_inflation_en{false};
  int unk_inflation_step{0};

  double odom_timeout{};
  Vec3f visualization_range{};
  string visualization_frame_id{};
  double visualization_rate{};

  double unk_thresh{};
  double map_sliding_thresh{};

  void resetMapSize()
  {
    int inflation_ratio = ceil(inflation_resolution / resolution);

#ifdef ORIGIN_AT_CENTER
    // When discretized in such a way that the origin is at cell center
    // the inflation ratio should be an odd number
    if (inflation_ratio % 2 == 0) {
      inflation_ratio += 1;
    }
    std::cout << YELLOW << " -- [RM-Config] inflation_ratio: " << inflation_ratio << std::endl;
#endif

    inflation_resolution = resolution * inflation_ratio;
    std::cout << color_text::GREEN << " -- [RM-Config] inflation_resolution: " << inflation_resolution
              << color_text::RESET << std::endl;

    half_map_size_d = map_size_d / 2;

    // Size_d is only for calculate index number.
    // 1) we calculate the index number of the inf map
    int max_step = 0;
    if (!unk_inflation_en) {
      max_step = inflation_step;
    } else {
      max_step = std::max(inflation_step, unk_inflation_step);
    }
    inf_half_map_size_i =
      (half_map_size_d / inflation_resolution).cast<int>() + (max_step + 1) * Vec3i::Ones();

    // 2) we calculate the index number of the prob map, which should be smaller than infmap
    half_map_size_i = (inf_half_map_size_i - (max_step + 1) * Vec3i::Ones()) * inflation_ratio;

    // 3) compute the frontier counter map size
    if (frontier_extraction_en) {
      fro_half_map_size_i = half_map_size_i + Vec3i::Constant(1);
    }

    // 4) re-compute the map_size_d, the map_size_d is not that important.
    map_size_d = (half_map_size_i * 2 + Vec3i::Constant(1)).cast<double>() * resolution;
    half_map_size_d = map_size_d / 2;

    // 5) compute the index of raycasting update box
    half_local_update_box_d = local_update_box_d / 2;
    half_local_update_box_i = (half_local_update_box_d / resolution).cast<int>();
    local_update_box_i = half_local_update_box_i * 2 + Vec3i::Constant(1);
    local_update_box_d = local_update_box_i.cast<double>() * resolution;

    // 6) reset the virtual ground and ceil size
#ifdef ORIGIN_AT_CENTER
    inf_virtual_ceil_height_id_g =
      static_cast<int>(virtual_ceil_height / inflation_resolution + SIGN(virtual_ceil_height) * 0.5);
    inf_virtual_ground_height_id_g =
      static_cast<int>(virtual_ground_height / inflation_resolution + SIGN(virtual_ground_height) * 0.5);
    fmt::print(" -- [ROG-Map] Init, resetMapSize: ORIGIN_AT_CENTER.\n");
#endif

#ifdef ORIGIN_AT_CORNER
    inf_virtual_ceil_height_id_g =
      static_cast<int>(floor(virtual_ceil_height / inflation_resolution)) - inflation_step;
    inf_virtual_ground_height_id_g =
      static_cast<int>(floor(virtual_ground_height / inflation_resolution)) + inflation_step;

    fmt::print(" -- [ROG-Map] Init, resetMapSize: ORIGIN_AT_CORNER.\n");
#endif

    virtual_ceil_height = inf_virtual_ceil_height_id_g * inflation_resolution - 0.5 * inflation_resolution;
    virtual_ground_height =
      inf_virtual_ground_height_id_g * inflation_resolution + 0.5 * inflation_resolution;
    fmt::print(" -- [ROG-Map] Init, resetMapSize: virtual_ceil_height: {}, virtual_ground_height: {}.\n",
      virtual_ceil_height,
      virtual_ground_height);
  }
};
}  // namespace rog_map
