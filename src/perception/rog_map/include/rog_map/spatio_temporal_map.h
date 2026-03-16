#pragma once

#include <rog_map/rog_map_core/sliding_map.h>
#include <rog_map/prob_map.h>
#include <rog_map/rog_map_core/common_lib.hpp>
#include <memory>
#include <vector>

namespace rog_map {

// STVL 配置参数
struct STVLConfig {
    bool enabled = true;
    double voxel_decay = 0.6;           // 衰减时间常数
    int decay_model = 0;                // 0: 线性 (Linear), 1: 指数 (Exponential)
    double decay_acceleration = 15.0;   // 在 FOV 内但没被观测到时的加速衰减系数

    // Mid360 视锥模型参数
    double min_z = 0.1;
    double max_z = 10.0;
    double vertical_fov_angle = 1.8151;  // ~104度
    double vertical_fov_offset = 0.3927; // ~22.5度偏置
};

class SpatioTemporalMap : public SlidingMap {
public:
    using Ptr = std::shared_ptr<SpatioTemporalMap>;

    SpatioTemporalMap() = default;
    virtual ~SpatioTemporalMap() = default;

    // 初始化图层，分配与 ProbMap 相同大小的时间戳数组
    void initSpatioTemporalMap(const Vec3i &half_map_size_i,
                               const double &resolution,
                               const STVLConfig &cfg);

    // 核心数据流方法：根据 ProbMap 状态更新时间戳，并触发超时擦除
    void applyTemporalDecay(ProbMap::Ptr prob_map, const super_utils::RobotState &robot_state, double current_time);

private:
    // [极其重要] 重写基类滑动生命周期钩子
    void resetCell(const int &hash_id) override;
    void resetLocalMap() override;

    // 内部方法：判断某点是否在传感器的有效视锥体内
    bool isInMid360Frustum(const super_utils::Vec3f &pt_world, const super_utils::Vec3f &sensor_pos) const;

    STVLConfig cfg_;
    std::vector<double> voxel_time_; // 核心 1D 数组，存储被判定为 Occupied 的绝对时间
};

} // namespace rog_map
