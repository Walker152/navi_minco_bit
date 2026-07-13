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

#include <queue>
#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <rog_map/esdf_map.h>
#include <rog_map/performance_monitor.hpp>
#include <rog_map/rog_map_core/raycaster.h>


namespace rog_map {
    using super_utils::Pose;


    class ProbMap : public SlidingMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap() = default;

        ~ProbMap() override = default;

        void initProbMap();

        bool isOccupied(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isKnownFree(const Vec3f &pos) const;

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isKnownFreeInflate(const Vec3f & pos) const;

        bool isFrontier(const Vec3f &pos) const;

        bool isFrontier(const Vec3i &id_g) const;

        // Query result
        GridType getGridType(Vec3i &id_g) const;

        GridType getGridType(const Vec3f &pos) const;

        GridType getInfGridType(const Vec3f &pos) const;

        double getMapValue(const Vec3f &pos) const;

        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void rawOccupiedBoxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                                  vec_E<Vec3f> &out_points) const;

        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        Vec3f getLocalMapOrigin() const;

        Vec3f getLocalMapSize() const;

        double getResolution() const{
            return sc_.resolution;
        }

        double getInfResolution()const {
            return inf_map_->getResolution();
        }

        void updateOccPointCloud(const PointCloud &input_cloud);

        void updateProbMap(const PointCloud &cloud, const Pose &sensor_pose,
                           const Vec3f &map_center_pos);

        void setUpdateTime(double now);

        int mapWidth() const { return sc_.map_size_i.x(); }
        int mapHeight() const { return sc_.map_size_i.y(); }
        int mapDepth() const { return sc_.map_size_i.z(); }
        Vec3i localMapMinIndex() const { return local_map_bound_min_i_; }
        Vec3i localMapMaxIndex() const { return local_map_bound_max_i_; }
        Vec3f localMapMinPosition() const { return local_map_bound_min_d_; }
        Vec3f localMapMaxPosition() const { return local_map_bound_max_d_; }
        double cellLastHitTime(const Vec3i &id_g) const;
        double cellLastUpdateTime(const Vec3i &id_g) const;

        bool applyDecay(double now);

        RuntimeStats runtimeStats() const { return runtime_stats_; }

    protected:
        rog_map::Config cfg_;
        InfMap::Ptr inf_map_;
        FreeCntMap::Ptr fcnt_map_;
        ESDFMap::Ptr esdf_map_;
        /// Spherical neighborhood lookup table
        std::vector<float> occupancy_buffer_;
        std::vector<double> last_hit_time_;
        std::vector<double> last_update_time_;
        std::vector<int> active_ids_;
        std::vector<uint8_t> active_flags_;
        std::vector<int> dirty_column_ids_;
        std::vector<uint8_t> dirty_column_flags_;
        bool full_layer_refresh_required_{true};
        double current_update_time_{0.0};

        bool map_empty_{true};
        struct RaycastData {
            raycaster::RayCaster raycaster;
            std::queue<Vec3i> update_cache_id_g;
            std::vector<uint16_t> operation_cnt;
            std::vector<uint16_t> hit_cnt;
            Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
            int batch_update_counter{0};
            std::mutex raycast_range_mtx;
        } raycast_data_;

        struct RaycastLocalBuffer {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            vec_E<Vec3i> hit_ids;
            vec_E<Vec3i> miss_ids;
        };

        RuntimeStats runtime_stats_;

        // standardization query
        // Known free < l_free
        // occupied >= l_occ
        bool isKnownFree(const double &prob) const {
            return prob < cfg_.l_free;
        }

        bool isOccupied(const double &prob) const {
            return prob >= cfg_.l_occ;
        }

        bool isUnknown(const double &prob) const {
            return prob >= cfg_.l_free && prob < cfg_.l_occ;
        }

        void slideAllMap(const Vec3f &pos);

        // warning using this function will cause memory leak if the id_g is not in the map
        bool isOccupied(const Vec3i &id_g) const;

        bool isUnknown(const Vec3i &id_g) const;

        bool isKnownFree(const Vec3i &id_g) const;

        //====================================================================
        void resetCell(const int &hash_id) override;

        void probabilisticMapFromCache();

        GridType classifyProb(const float &prob) const;

        void updateCellState(const Vec3f &pos, const GridType &from_type, const GridType &to_type);

        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);
        void raycastProcessSerial(const PointCloud &input_cloud, const Vec3f &cur_odom);
        void raycastProcessParallel(const PointCloud &input_cloud, const Vec3f &cur_odom);

        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);
        void markDirtyColumn(const Vec3i &id_g);
        void clearDirtyColumns();
        void markAllDirtyColumns();
        const std::vector<int> &dirtyColumnIds() const { return dirty_column_ids_; }
        bool fullLayerRefreshRequired() const { return full_layer_refresh_required_; }

        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;
    };
}
