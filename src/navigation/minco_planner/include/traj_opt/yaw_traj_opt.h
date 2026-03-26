#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "data_structure/base/trajectory.h"

namespace traj_opt
{

class YawTrajOpt
{
public:
    // === Public Type Aliases ===
    using Ptr = std::shared_ptr<YawTrajOpt>;
    using Trajectory = geometry_utils::Trajectory;

    // === Constructor & Lifecycle ===
    explicit YawTrajOpt(double yaw_dot_max);

    // === Core Planning Interfaces ===
    // --- Yaw Time Allocation ---
    void getYawTimeAllocation(double duration, Eigen::VectorXd & times) const;

    // --- Yaw Waypoint Allocation ---
    static void getYawWaypointAllocation(
        const Eigen::Vector4d & init_state,
        Eigen::Vector4d & goal_state,
        Eigen::VectorXd & way_pts,
        const Eigen::VectorXd & times,
        const Trajectory & pos_traj);

    // --- Yaw Trajectory Optimization ---
    bool optimize(
        const Eigen::Vector4d & istate_in,
        const Eigen::Vector4d & gstate_in,
        const Trajectory & pos_traj,
        Trajectory & out_traj,
        int order = 3,
        bool free_start = false,
        bool free_goal = true);

private:
    // === State Variables & Caches ===
    bool free_goal_{false};
    double yaw_dot_max_{10.0};
};

}  // namespace traj_opt