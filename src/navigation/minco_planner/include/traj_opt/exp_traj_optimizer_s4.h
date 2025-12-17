#ifndef TRAJ_OPT__EXP_TRAJ_OPT_H_
#define TRAJ_OPT__EXP_TRAJ_OPT_H_

#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "traj_opt/minco.h"

namespace traj_opt {

struct Config {
    double max_vel{2.0};
    double max_acc{2.0};
    double max_jerk{4.0};
    double max_snap{4.0};
    double rho{100.0}; // Time regularization weight
};

class ExpTrajOpt {
public:
    ExpTrajOpt(const Config &cfg, std::shared_ptr<void> ros_ptr); // ros_ptr is void* to avoid dependency for now
    ~ExpTrajOpt();

    bool optimize(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, 
                  const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel,
                  const std::vector<Eigen::Vector3d> &guide_path,
                  std::vector<Eigen::Vector3d> &optimized_path);

private:
    Config cfg_;
    
    // Simplified optimization variables
    struct OptimizationVariables {
        double rho;
        Eigen::VectorXd times;
        Eigen::Matrix3Xd points;
        MINCO_S4NU minco;
    } opt_vars;

    static double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);
};

} // namespace traj_opt

#endif // TRAJ_OPT__EXP_TRAJ_OPT_H_
