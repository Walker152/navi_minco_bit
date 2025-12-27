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

#include <rog_map/elevation_map.h>
#include <rog_map/prob_map.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace rog_map;
using namespace color_text;
using namespace super_utils;

namespace rog_map {

    ElevationMap::ElevationMap(rog_map::Config &cfg) {
        if (had_been_initialized) {
            throw std::runtime_error(" -- [ElevationMap]: init can only be called once!");
        }
        
        cfg_ = cfg;
        int max_step = cfg_.inflation_step;
        if (cfg_.unk_inflation_en) {
            max_step = std::max(max_step, cfg_.unk_inflation_step);
        }

        initCounterMap(cfg_.half_map_size_i,
                       cfg_.resolution,
                       cfg_.inflation_resolution,
                       max_step,
                       cfg_.map_sliding_en,
                       cfg_.map_sliding_thresh,
                       cfg_.fix_map_origin,
                       cfg_.unk_thresh);

        int map_size = sc_.map_size_i.prod();
        
        // 初始化高程地图数据
        emd_.elevations.resize(map_size, 0.0f);
        emd_.update_counts.resize(map_size, 0);
        emd_.slope_data.resize(map_size, 0.0f);
        emd_.roughness_data.resize(map_size, 0.0f);
        
        // 初始化统计信息
        emd_.min_elevation = 0.0f;
        emd_.max_elevation = 0.0f;
        emd_.avg_elevation = 0.0f;

        resetLocalMap();
        had_been_initialized = true;
        
        std::cout << GREEN << " -- [ElevationMap] Init successfully -- ." << RESET << std::endl;
        printMapInformation();
    }

    void ElevationMap::initElevationMap(const Vec3i &half_map_size_i,
                                        const double &resolution,
                                        const bool &map_sliding_en,
                                        const double &sliding_thresh,
                                        const Vec3f &fix_map_origin) {
        // 不重复初始化 Sliding/CounterMap，这里仅确保二维缓冲已分配
        // 使用当前 sc_ 尺寸分配 XY 缓冲
        const int sz = sc_.map_size_i(0) * sc_.map_size_i(1);
        elevation_xy_.assign(sz, std::numeric_limits<double>::quiet_NaN());

        // 初始化统计缓冲（保持原有接口可用）
        const int map_size = sc_.map_size_i.prod();
        emd_.elevations.assign(map_size, 0.0f);
        emd_.update_counts.assign(map_size, 0);
        emd_.slope_data.assign(map_size, 0.0f);
        emd_.roughness_data.assign(map_size, 0.0f);
        emd_.min_elevation = 0.0f;
        emd_.max_elevation = 0.0f;
        emd_.avg_elevation = 0.0f;
        had_been_initialized = true;
    }

    float ElevationMap::getElevation(const Vec3f &pos) const {
        if (!insideLocalMap(pos)) {
            return 0.0f;
        }
        Vec3i id_g; 
        posToGlobalIndex(pos, id_g);
        double z{}; 
        if (!getElevationAtIndex(id_g, z)) {
            return 0.0f;
        }
        return static_cast<float>(z);
    }

    float ElevationMap::getElevation(const Vec3i &id_g) const {
        if (!insideLocalMap(id_g)) {
            return 0.0f;
        }
        double z{};
        if (!getElevationAtIndex(id_g, z)) {
            return 0.0f;
        }
        return static_cast<float>(z);
    }

    float ElevationMap::getSlope(const Vec3f &pos) const {
        if (!insideLocalMap(pos)) {
            return 0.0f;
        }
        Vec3i id_g;
        elevationMapPosToGlobalIndex(pos, id_g);
        return emd_.slope_data[getHashIndexFromGlobalIndex(id_g)];
    }

    float ElevationMap::getRoughness(const Vec3f &pos, float radius) const {
        if (!insideLocalMap(pos)) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        
        Vec3i center_id_g;
        elevationMapPosToGlobalIndex(pos, center_id_g);
        
        // 搜索半径内的点
        int search_radius = static_cast<int>(radius / sc_.resolution);
        std::vector<float> neighbor_elevations;
        
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            for (int dy = -search_radius; dy <= search_radius; ++dy) {
                Vec3i neighbor_id_g = center_id_g + Vec3i(dx, dy, 0);
                double elevation;
                if (getElevationAtIndex(neighbor_id_g, elevation)) {
                    neighbor_elevations.push_back(static_cast<float>(elevation));
                }
            }
        }
        
        if (neighbor_elevations.size() < 2) {
            return 0.0f; // 如果点数不足，无法计算标准差，返回0
        }
        
        // 计算粗糙度（高程标准差）
        float sum = 0.0f;
        for (float elev : neighbor_elevations) {
            sum += elev;
        }
        float mean = sum / neighbor_elevations.size();
        
        float variance = 0.0f;
        for (float elev : neighbor_elevations) {
            variance += std::pow(elev - mean, 2);
        }
        
        // 注意：这里是样本标准差，分母是 n-1
        if (neighbor_elevations.size() > 1) {
            return std::sqrt(variance / (neighbor_elevations.size() - 1));
        } else {
            return 0.0f;
        }
    }

    bool ElevationMap::isTraversable(const Vec3f &pos, float max_slope) const {
        float slope = getSlope(pos);
        return slope <= max_slope;
    }

    void ElevationMap::updateElevation(const Vec3f &pos, float elevation, GridType grid_type) {
        TimeConsuming tc("updateElevation", false);
        
        if (!insideLocalMap(pos)) {
#ifdef ELEVATION_MAP_DEBUG
            std::cout << YELLOW << " -- [ElevationMap]: Update point outside local map." << RESET << std::endl;
#endif
            return;
        }
        
        Vec3i id_g;
        elevationMapPosToGlobalIndex(pos, id_g);
        const int addr = getHashIndexFromGlobalIndex(id_g);
        
        // 获取之前的网格类型
        GridType prev_type = CounterMap::getGridType(addr);
        
        // 更新高程值（加权平均）
        int count = emd_.update_counts[addr];
        emd_.elevations[addr] = (emd_.elevations[addr] * count + elevation) / (count + 1);
        emd_.update_counts[addr] = count + 1;
        
        // 更新统计信息
        emd_.min_elevation = std::min(emd_.min_elevation, elevation);
        emd_.max_elevation = std::max(emd_.max_elevation, elevation);
        
        // 更新CounterMap的状态
        if (prev_type != grid_type) {
            updateGridCounter(pos, prev_type, grid_type);
        }
        
        // 更新地形分析
        updateTerrainAnalysis(id_g);
        
        update_num_++;
        update_time_ += tc.stop();
    }

    void ElevationMap::updateElevationBatch(const std::vector<Vec3f> &points, 
                                           const std::vector<float> &elevations,
                                           GridType grid_type) {
        if (points.size() != elevations.size()) {
            throw std::invalid_argument(" -- [ElevationMap]: Points and elevations size mismatch!");
        }
        
        for (size_t i = 0; i < points.size(); ++i) {
            updateElevation(points[i], elevations[i], grid_type);
        }
    }

    void ElevationMap::getTerrainProfile(const Vec3f &start, const Vec3f &end,
                                        std::vector<float> &elevations) const {
        elevations.clear();
        
        Vec3f direction = (end - start).normalized();
        float distance = (end - start).norm();
        float step_size = sc_.resolution;
        int steps = static_cast<int>(distance / step_size);
        
        for (int i = 0; i <= steps; ++i) {
            Vec3f point = start + direction * (i * step_size);
            elevations.push_back(getElevation(point));
        }
    }

    void ElevationMap::findLocalMinMax(const Vec3f &center, float radius,
                                      float &min_elev, float &max_elev) const {
        min_elev = std::numeric_limits<float>::max();
        max_elev = std::numeric_limits<float>::lowest();
        
        Vec3i center_id_g;
        elevationMapPosToGlobalIndex(center, center_id_g);
        int search_radius = static_cast<int>(radius / sc_.resolution);
        
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            for (int dy = -search_radius; dy <= search_radius; ++dy) {
                Vec3i id_g = center_id_g + Vec3i(dx, dy, 0);
                double z_val;
                if (getElevationAtIndex(id_g, z_val)) {
                    min_elev = std::min(min_elev, static_cast<float>(z_val));
                    max_elev = std::max(max_elev, static_cast<float>(z_val));
                }
            }
        }
    }

    void ElevationMap::boxSearch(const Vec3f& box_min, const Vec3f& box_max, vec_E<Vec3f>& out_points) const {
        out_points.clear();
        Vec3i min_id, max_id;
        posToGlobalIndex(box_min, min_id);
        posToGlobalIndex(box_max, max_id);
        for(int i=0; i<3; ++i) if(min_id(i) > max_id(i)) std::swap(min_id(i), max_id(i));

        for (int x = min_id.x(); x <= max_id.x(); ++x) {
            for (int y = min_id.y(); y <= max_id.y(); ++y) {
                Vec3i id(x, y, 0);
                Vec3f pos;
                globalIndexToPos(id, pos);
                double z_val = 0;
                if (getElevationAtPosXY(pos, z_val)) {
                    pos.z() = z_val;
                    out_points.push_back(pos);
                }
            }
        }
    }

    void ElevationMap::resetLocalMap() {
        // 重置计数器（参考 InfMap::resetLocalMap 实现）
        std::cout << YELLOW << " -- [ElevationMap] Clear all local map." << RESET << std::endl;
        if (!md_.unknown_cnt.empty()) {
            std::fill(md_.unknown_cnt.begin(), md_.unknown_cnt.end(), md_.sub_grid_num);
        }
        if (!md_.occupied_cnt.empty()) {
            std::fill(md_.occupied_cnt.begin(), md_.occupied_cnt.end(), 0);
        }
        
        // 重置高程数据
        std::fill(emd_.elevations.begin(), emd_.elevations.end(), 0.0f);
        std::fill(emd_.update_counts.begin(), emd_.update_counts.end(), 0);
        std::fill(emd_.slope_data.begin(), emd_.slope_data.end(), 0.0f);
        std::fill(emd_.roughness_data.begin(), emd_.roughness_data.end(), 0.0f);
        // 二维地面高程设为未知
        if (!elevation_xy_.empty()) {
            std::fill(elevation_xy_.begin(), elevation_xy_.end(), std::numeric_limits<double>::quiet_NaN());
        }
        
        emd_.min_elevation = 0.0f;
        emd_.max_elevation = 0.0f;
        emd_.avg_elevation = 0.0f;
        
        update_num_ = 0;
        update_time_ = 0.0;

        // elevation_xy_ 已在上面置 NaN
    }

    void ElevationMap::writeMapInfoToLog(std::ofstream &log_file) {
        log_file << "[ElevationMap]" << std::endl;
        log_file << "\tresolution: " << sc_.resolution << std::endl;
        log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << std::endl;
        log_file << "\tmap_size_d: " << (sc_.map_size_i.cast<double>() * sc_.resolution).transpose() << std::endl;
        log_file << "\tmin_elevation: " << emd_.min_elevation << std::endl;
        log_file << "\tmax_elevation: " << emd_.max_elevation << std::endl;
        log_file << "\tavg_elevation: " << emd_.avg_elevation << std::endl;
        log_file << "\tupdate_count: " << update_num_ << std::endl;
    }

    void ElevationMap::elevationMapPosToGlobalIndex(const Vec3f &pos, Vec3i &id) const {
        posToGlobalIndex(pos, id);
    }

    void ElevationMap::elevationMapGlobalIndexToPos(const Vec3i &id_g, Vec3f &pos) const {
        globalIndexToPos(id_g, pos);
    }

    //================== 2.5D Elevation functions (Route A) ==================

    float ElevationMap::interpolateElevation(const Vec3f &pos) const {
        // 双线性插值实现（基于 2.5D 缓冲）
        Vec3i base_id_g; elevationMapPosToGlobalIndex(pos, base_id_g);
        Vec3f base_pos; elevationMapGlobalIndexToPos(base_id_g, base_pos);

        float fx = (pos.x() - base_pos.x()) / sc_.resolution;
        float fy = (pos.y() - base_pos.y()) / sc_.resolution;
        fx = std::clamp(fx, 0.0f, 1.0f);
        fy = std::clamp(fy, 0.0f, 1.0f);

        auto fetch2d = [&](const Vec3i &id)->double{
            const int addr = idx2d_from_global(id);
            if (addr < 0 || addr >= static_cast<int>(elevation_xy_.size())) return std::numeric_limits<double>::quiet_NaN();
            return elevation_xy_[addr];
        };

        double elev00 = fetch2d(base_id_g);
        double elev10 = fetch2d(base_id_g + Vec3i(1, 0, 0));
        double elev01 = fetch2d(base_id_g + Vec3i(0, 1, 0));
        double elev11 = fetch2d(base_id_g + Vec3i(1, 1, 0));

        auto mix = [](double a, double b, double t)->double{
            if (std::isnan(a) && std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
            if (std::isnan(a)) return b;
            if (std::isnan(b)) return a;
            return a * (1 - t) + b * t;
        };

        double elev0 = mix(elev00, elev10, fx);
        double elev1 = mix(elev01, elev11, fx);
        double out = mix(elev0, elev1, fy);
        if (std::isnan(out)) return 0.0f;
        return static_cast<float>(out);
    }

    void ElevationMap::updateTerrainAnalysis(const Vec3i &id_g) {
        if (!insideLocalMap(id_g)) return;
        
        const int addr = getHashIndexFromGlobalIndex(id_g);
        
        // 计算坡度（使用相邻网格的高程差，基于 2.5D 缓冲）
        std::vector<float> neighbor_elevs; getNeighborElevations(id_g, neighbor_elevs);
        const int addr2d = idx2d_from_global(id_g);
        float center_elev = std::numeric_limits<float>::quiet_NaN();
        if (addr2d >= 0 && addr2d < static_cast<int>(elevation_xy_.size())) center_elev = elevation_xy_[addr2d];

        if (!std::isnan(center_elev) && neighbor_elevs.size() >= 3) {
            float max_diff = 0.0f;
            for (float elev : neighbor_elevs) {
                if (!std::isnan(elev)) max_diff = std::max(max_diff, std::abs(elev - center_elev));
            }
            emd_.slope_data[addr] = std::atan2(max_diff, sc_.resolution) * 180.0f / static_cast<float>(M_PI);
        } else {
            // 高程未知或邻域不足，标记坡度为 NaN，避免误判可通行
            emd_.slope_data[addr] = std::numeric_limits<float>::quiet_NaN();
        }

        // 计算粗糙度：使用世界坐标
        Vec3f world_pos; elevationMapGlobalIndexToPos(id_g, world_pos);
        emd_.roughness_data[addr] = getRoughness(world_pos, 0.2f);
    }

    void ElevationMap::getNeighborElevations(const Vec3i &id_g, 
                                            std::vector<float> &neighbor_elevs) const {
        neighbor_elevs.clear();
        
        // 8-邻域搜索（基于 2.5D 缓冲）
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                
                Vec3i neighbor_id = id_g + Vec3i(dx, dy, 0);
                if (insideLocalMap(neighbor_id)) {
                    const int a = idx2d_from_global(neighbor_id);
                    if (a >= 0 && a < static_cast<int>(elevation_xy_.size())) neighbor_elevs.push_back(static_cast<float>(elevation_xy_[a]));
                }
            }
        }
    }

    void ElevationMap::triggerJumpingEdge(const Vec3i &id_g,
                                         const GridType &from_type,
                                         const GridType &to_type) {
        // 若网格状态未变化，直接返回
        if (from_type == to_type) return;

        // 仅依据 XY 检查和处理，避免 z 维度误判（与 getElevationAtPosXY 对齐）
        Vec3i id_xy_chk(id_g.x(), id_g.y(), local_map_origin_i_.z());
        if (!insideLocalMap(id_xy_chk)) return;

        // 1) 失效该 (x,y) 的 2.5D 高程（置为 NaN，防止规划读取到过期值）
        const int addr2d = idx2d_from_global(id_xy_chk);
        if (addr2d >= 0 && addr2d < static_cast<int>(elevation_xy_.size())) {
            elevation_xy_[addr2d] = std::numeric_limits<double>::quiet_NaN();
        }

        // 2) 将本格与 8 邻域的坡度/粗糙度置为 NaN，表示未知，避免误判为可通行
        auto reset_sr = [&](const Vec3i &cell) {
            if (!insideLocalMap(cell)) return;
            const int a3d = getHashIndexFromGlobalIndex(cell);
            if (a3d >= 0 && a3d < static_cast<int>(emd_.slope_data.size())) {
                emd_.slope_data[a3d] = std::numeric_limits<float>::quiet_NaN();
                emd_.roughness_data[a3d] = std::numeric_limits<float>::quiet_NaN();
            }
        };
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Vec3i n(id_g.x() + dx, id_g.y() + dy, id_g.z());
                reset_sr(n);
            }
        }

        // 3) 触发地形分析重算（当前高程 NaN 时保持 NaN）
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Vec3i n(id_g.x() + dx, id_g.y() + dy, id_g.z());
                updateTerrainAnalysis(n);
            }
        }
    }

    void ElevationMap::resetOneCell(const int &hash_id) {
        // 重置单个网格的高程数据
        emd_.elevations[hash_id] = 0.0f;
        emd_.update_counts[hash_id] = 0;
        emd_.slope_data[hash_id] = 0.0f;
        emd_.roughness_data[hash_id] = 0.0f;
    }

    void ElevationMap::updateFromMap(const ProbMap &map, const Vec3f &box_min, const Vec3f &box_max) {
    // 将包围盒限制到 ProbMap 的本地地图范围
    Vec3f bmin = box_min, bmax = box_max;
    map.boundBoxByLocalMap(bmin, bmax);

    // 转为全局索引（仅用 XY）；确保顺序
    Vec3i min_g, max_g;
    posToGlobalIndex(bmin, min_g);
    posToGlobalIndex(bmax, max_g);
    for (int i = 0; i < 3; ++i) if (min_g(i) > max_g(i)) std::swap(min_g(i), max_g(i));

    // 垂直扫描范围：用本地地图边界（与 SlidingMap 同步），并应用高度过滤
        int z_ground_id{}, z_ceil_id{};
        
        // 计算过滤高度的索引
        int filter_min_id{}, filter_max_id{};
        posToGlobalIndex(cfg_.elevation_filter_z_min, filter_min_id);
        posToGlobalIndex(cfg_.elevation_filter_z_max, filter_max_id);
        
        // 计算本地地图边界索引
        int map_min_id{}, map_max_id{};
        posToGlobalIndex(local_map_bound_min_d_.z(), map_min_id);
        posToGlobalIndex(local_map_bound_max_d_.z(), map_max_id);
        
        if (filter_min_id > filter_max_id) std::swap(filter_min_id, filter_max_id);
        if (map_min_id > map_max_id) std::swap(map_min_id, map_max_id);
        
        // 取交集：[max(min1, min2), min(max1, max2)]
        z_ground_id = std::max(map_min_id, filter_min_id);
        z_ceil_id = std::min(map_max_id, filter_max_id);
        
        if (z_ground_id > z_ceil_id) {
             // 范围无效，直接返回
             return;
        }

        // 对每个 (x,y) 自顶向下扫描，找“上方自由且下方有支撑（或在虚拟地面以下）”的最高 z
        for (int x = min_g.x(); x <= max_g.x(); ++x) {
            for (int y = min_g.y(); y <= max_g.y(); ++y) {
                double elev = std::numeric_limits<double>::quiet_NaN();
                for (int z = z_ceil_id; z >= z_ground_id; --z) {
                    Vec3i id_g(x, y, z);
                    Vec3f pos;
                    globalIndexToPos(id_g, pos);
                    const bool is_occ = map.isOccupied(pos);
                    const bool is_unk = map.isUnknown(pos);
                    // 取下方一点作为支撑检测
                    const Vec3f pos_below(pos.x(), pos.y(), pos.z() - sc_.resolution);
                    const bool is_bottom = (z - 1 < z_ground_id);
                    const bool below_occ = is_bottom || map.isOccupied(pos_below);
                    const bool below_loose = is_bottom || !map.isKnownFree(pos_below);

                    if ((is_occ && below_loose) || (is_unk && below_occ)) {
                        elev = static_cast<double>(pos.z());
                        break;
                    }
                }
                // 写回 XY 缓冲
                Vec3i id_xy(x, y, 0);
                const int addr = idx2d_from_global(id_xy);
                if (addr >= 0 && addr < static_cast<int>(elevation_xy_.size())) {
                    elevation_xy_[addr] = elev;
                }
            }
        }
    }

    bool ElevationMap::getElevationAtPosXY(const Vec3f &pos, double &z_out) const {
        // 仅按 XY 检查是否在本地地图内：将 z 固定在当前本地原点高度，避免 z 维度误判
        Vec3f xy_pos(pos.x(), pos.y(), local_map_origin_d_.z());
        if (!insideLocalMap(xy_pos)) {
            return false;
        }
        Vec3i id_g; 
        posToGlobalIndex(Vec3f(pos.x(), pos.y(), local_map_origin_d_.z()), id_g);
        const int addr = idx2d_from_global(id_g);
        if (addr < 0 || addr >= static_cast<int>(elevation_xy_.size())) {
            return false;
        }
        const double v = elevation_xy_[addr];
        if (std::isnan(v)) {
            return false;
        }
        z_out = v;
        return true;
    }

    bool ElevationMap::getElevationAtIndex(const Vec3i &id_g, double &z_out) const {
        // 仅按 XY 检查是否在本地地图内：将 z 固定在当前本地原点高度，避免 z 维度误判
        Vec3i id_xy_chk(id_g.x(), id_g.y(), local_map_origin_i_.z());
        if (!insideLocalMap(id_xy_chk)) {
            return false;
        }
        Vec3i id_xy(id_g.x(), id_g.y(), local_map_origin_i_.z());
        const int addr = idx2d_from_global(id_xy);
        if (addr < 0 || addr >= static_cast<int>(elevation_xy_.size())) {
            return false;
        }
        const double v = elevation_xy_[addr];
        if (std::isnan(v)) {
            return false;
        }
        z_out = v;
        return true;
    }
}