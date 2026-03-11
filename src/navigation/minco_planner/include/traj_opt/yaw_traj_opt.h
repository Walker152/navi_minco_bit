#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "data_structure/base/trajectory.h"

namespace traj_opt
{

class YawTrajOpt
{
private:
    bool free_goal_{false};
    double yaw_dot_max_{10.0};

public:
    using Ptr = std::shared_ptr<YawTrajOpt>;
    using Trajectory = geometry_utils::Trajectory;

    explicit YawTrajOpt(double yaw_dot_max);

    void getYawTimeAllocation(double duration, Eigen::VectorXd & times) const;

    static void getYawWaypointAllocation(
        const Eigen::Vector4d & init_state,
        Eigen::Vector4d & goal_state,
        Eigen::VectorXd & way_pts,
        const Eigen::VectorXd & times,
        const Trajectory & pos_traj);

    bool optimize(
        const Eigen::Vector4d & istate_in,
        const Eigen::Vector4d & gstate_in,
        const Trajectory & pos_traj,
        Trajectory & out_traj,
        int order = 3,
        bool free_start = false,
        bool free_goal = true);
};

}  // namespace traj_opt