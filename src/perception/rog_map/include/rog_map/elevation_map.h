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

#include <rog_map/rog_map_core/counter_map.h>
#include <rog_map/rog_map_core/config.hpp>
#include <vector>
#include <memory>
#include <fstream>

namespace rog_map { class ProbMap; }
#include <limits>

// #define ELEVATION_MAP_DEBUG
namespace rog_map {
    using super_utils::GridType;
    class ProbMap; // forward declaration

    class ElevationMap : public CounterMap {
    public:
        typedef std::shared_ptr<ElevationMap> Ptr;

        ElevationMap() = default;
        explicit ElevationMap(rog_map::Config &cfg);
        virtual ~ElevationMap() = default;

        // 2.5D Elevation interface aligned with ROGMap/A*
        // 初始化二维高程缓冲（不重复初始化Sliding/CounterMap），尺寸基于现有sc_
        void initElevationMap(const Vec3i &half_map_size_i,
                              const double &resolution,
                              const bool &map_sliding_en,
                              const double &sliding_thresh,
                              const Vec3f &fix_map_origin);

        // 从 ProbMap 在 [box_min, box_max] 的 XY 区域沿 z 扫描重建二维地面高程
        void updateFromMap(const ProbMap &map, const Vec3f &box_min, const Vec3f &box_max);

        // 查询：按 XY 查询地面高程；未知返回 false
        bool getElevationAtPosXY(const Vec3f &pos, double &z_out) const;
        bool getElevationAtIndex(const Vec3i &id_g, double &z_out) const;

        // 高程查询接口
        float getElevation(const Vec3f &pos) const;
        float getElevation(const Vec3i &id_g) const;
        
        // 地形属性查询
        float getSlope(const Vec3f &pos) const;
        float getRoughness(const Vec3f &pos, float radius = 0.1f) const;
        bool isTraversable(const Vec3f &pos, float max_slope = 30.0f) const;
        
        // 高程更新接口
        void updateElevation(const Vec3f &pos, float elevation, GridType grid_type = GridType::OCCUPIED);
        void updateElevationBatch(const std::vector<Vec3f> &points, 
                                 const std::vector<float> &elevations,
                                 GridType grid_type = GridType::OCCUPIED);
        
        // 地形分析
        void getTerrainProfile(const Vec3f &start, const Vec3f &end, 
                              std::vector<float> &elevations) const;
        void findLocalMinMax(const Vec3f &center, float radius,
                            float &min_elev, float &max_elev) const;
        
        void boxSearch(const Vec3f& box_min, const Vec3f& box_max, vec_E<Vec3f>& out_points) const;

        // 地图操作
        void resetLocalMap() override;
        void writeMapInfoToLog(std::ofstream &log_file);
        
        // 坐标转换
        void elevationMapPosToGlobalIndex(const Vec3f &pos, Vec3i &id) const;
        void elevationMapGlobalIndexToPos(const Vec3i &id_g, Vec3f &pos) const;

        // 获取地图数据（用于可视化等）
        const std::vector<float>& getElevationData() const { return emd_.elevations; }
        const std::vector<int>& getUpdateCounts() const { return emd_.update_counts; }
        const std::vector<float>& getSlopeData() const { return emd_.slope_data; }
        const std::vector<float>& getRoughnessData() const { return emd_.roughness_data; }

    protected:
        struct ElevationMapData {
            std::vector<float> elevations;      // 高程值
            std::vector<int> update_counts;     // 更新次数
            std::vector<float> slope_data;      // 坡度数据
            std::vector<float> roughness_data;  // 粗糙度数据
            
            // 统计信息
            float min_elevation{0.0f};
            float max_elevation{0.0f};
            float avg_elevation{0.0f};
        } emd_;

        rog_map::Config cfg_;
        
        // 地形分析参数
        float slope_threshold_{30.0f};  // 坡度阈值（度）
        float roughness_threshold_{0.1f}; // 粗糙度阈值

        // 更新统计
        int update_num_{0};
        double update_time_{0.0};

        // 高程插值方法
        float interpolateElevation(const Vec3f &pos) const;
        void updateTerrainAnalysis(const Vec3i &id_g);
        
        // 邻居搜索
        void getNeighborElevations(const Vec3i &id_g, 
                                  std::vector<float> &neighbor_elevs) const;
        
        // 重写CounterMap的虚函数
        void triggerJumpingEdge(const Vec3i &id_g,
                               const GridType &from_type,
                               const GridType &to_type) override;
        
        void resetOneCell(const int &hash_id) override;

    private:
        // 2.5D 二维高程缓冲：行主序存储，大小为 map_size_x * map_size_y
        std::vector<double> elevation_xy_;

        inline int idx2d_from_local(const Vec3i &id_l) const {
            Vec3i id = id_l + sc_.half_map_size_i; // shift to [0, size)
            return id(0) * sc_.map_size_i(1) + id(1);
        }

        inline int idx2d_from_global(const Vec3i &id_g) const {
            Vec3i id_l; globalIndexToLocalIndex(id_g, id_l);
            return idx2d_from_local(id_l);
        }

        bool had_been_initialized{false};
    };
}