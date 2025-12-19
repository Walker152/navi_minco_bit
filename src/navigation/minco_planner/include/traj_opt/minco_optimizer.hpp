#ifndef MINCO_PLANNER__MINCO_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>

#include "traj_opt/minco.h" 
#include "data_structure/base/trajectory.h" 

namespace minco_planner {

class MincoOptimizer {
public:
    struct Config {
        double max_vel = 2.5;       // 最大速度 (m/s)
        double max_acc = 3.0;       // 最大加速度 (m/s^2)
        int time_allocation_iters = 15; // 时间重分配最大迭代次数
    };

    MincoOptimizer(const Config& cfg) : config_(cfg) {
        minco_solver_ = std::make_shared<traj_opt::MINCO_S3NU>();
    }

    /**
     * @brief 核心接口
     * @param waypoints A* 输出的离散路径点
     * @param start_state 起点状态 (P, V, A)
     * @param end_state 终点状态 (P, V, A)
     * @param out_traj [输出] 优化后的轨迹对象
     */
    bool optimize(const std::vector<Eigen::Vector3d>& waypoints,
                 const Eigen::Matrix3d& start_state,
                 const Eigen::Matrix3d& end_state,
                 geometry_utils::Trajectory& out_traj);

private:
    std::shared_ptr<traj_opt::MINCO_S3NU> minco_solver_;
    Config config_;

    // 初始时间分配
    Eigen::VectorXd allocateInitialTimes(const std::vector<Eigen::Vector3d>& waypoints);

    // 返回值: 全局最大的超限比例 (>1.0 代表违规)
    double checkConstraints(const geometry_utils::Trajectory& traj, 
                            const Eigen::VectorXd& current_times,
                            Eigen::VectorXd& new_times);
};

} // namespace minco_planner

#endif