#include <rog_map/spatio_temporal_map.h>

#include <algorithm>
#include <cmath>

using namespace rog_map;

void SpatioTemporalMap::initSpatioTemporalMap(const Vec3i &half_map_size_i,
                                              const double &resolution,
                                              const STVLConfig &cfg) {
    cfg_ = cfg;
    initSlidingMap(half_map_size_i, resolution, true, 0.0, Vec3f::Zero());
    voxel_time_.assign(sc_.map_vox_num, 0.0);
}

void SpatioTemporalMap::resetCell(const int &hash_id) {
    voxel_time_[hash_id] = 0.0;
}

void SpatioTemporalMap::resetLocalMap() {
    std::fill(voxel_time_.begin(), voxel_time_.end(), 0.0);
}

bool SpatioTemporalMap::isInMid360Frustum(const super_utils::Vec3f &pt_world,
                                          const super_utils::Vec3f &sensor_pos) const {
    const Vec3f diff = pt_world - sensor_pos;
    const double dist = diff.norm();
    if (dist < cfg_.min_z || dist > cfg_.max_z) {
        return false;
    }

    const double dz = static_cast<double>(diff.z());
    const double ratio = std::max(-1.0, std::min(1.0, dz / dist));
    const double elevation_angle = std::asin(ratio);

    const double half_vfov = 0.5 * cfg_.vertical_fov_angle;
    const double lower = cfg_.vertical_fov_offset - half_vfov;
    const double upper = cfg_.vertical_fov_offset + half_vfov;
    return elevation_angle >= lower && elevation_angle <= upper;
}

void SpatioTemporalMap::applyTemporalDecay(ProbMap::Ptr prob_map,
                                           const super_utils::RobotState &robot_state,
                                           double current_time) {
    if (!cfg_.enabled || !prob_map) {
        return;
    }

    // Keep STVL's sliding window synchronized with current robot pose.
    mapSliding(robot_state.p);

    Vec3i min_i, max_i;
    prob_map->getLocalUpdateBox(min_i, max_i);

    const double tau = std::max(cfg_.voxel_decay, 1e-6);
    for (int x = min_i.x(); x <= max_i.x(); ++x) {
        for (int y = min_i.y(); y <= max_i.y(); ++y) {
            for (int z = min_i.z(); z <= max_i.z(); ++z) {
                const Vec3i id_g(x, y, z);
                if (!insideLocalMap(id_g)) {
                    continue;
                }

                const int hash_id = getHashIndexFromGlobalIndex(id_g);
                Vec3i id_query = id_g;
                if (prob_map->getGridType(id_query) == GridType::OCCUPIED) {
                    Vec3f pt_world;
                    globalIndexToPos(id_g, pt_world);
                    const bool in_fov = isInMid360Frustum(pt_world, robot_state.p);

                    double dt = current_time - voxel_time_[hash_id];
                    if (voxel_time_[hash_id] == 0.0) {
                        voxel_time_[hash_id] = current_time;
                        dt = 0.0;
                    }

                    if (in_fov && dt > 0.05) {
                        dt += cfg_.decay_acceleration;
                    }

                    bool timeout = false;
                    if (cfg_.decay_model == 0) {
                        timeout = dt > cfg_.voxel_decay;
                    }
                    else {
                        const double survival = std::exp(-dt / tau);
                        timeout = survival < 0.5;
                    }

                    if (timeout) {
                        prob_map->forceUnknown(id_g);
                        voxel_time_[hash_id] = 0.0;
                    }
                }
                else {
                    voxel_time_[hash_id] = 0.0;
                }
            }
        }
    }
}
