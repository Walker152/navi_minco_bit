#include "traj_opt/exp_traj_optimizer_s4.h"
#include "utils/optimization/lbfgs.h"
#include <iostream>

namespace traj_opt {

ExpTrajOpt::ExpTrajOpt(const Config &cfg, std::shared_ptr<void> ros_ptr)
    : cfg_(cfg) {
}

ExpTrajOpt::~ExpTrajOpt() {
}

bool ExpTrajOpt::optimize(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, 
                          const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel,
                          const std::vector<Eigen::Vector3d> &guide_path,
                          std::vector<Eigen::Vector3d> &optimized_path) {
    
    int piece_num = guide_path.size();
    if (piece_num < 1) piece_num = 1;

    // Initialize optimization variables
    opt_vars.rho = cfg_.rho;
    opt_vars.times.resize(piece_num);
    opt_vars.times.setConstant(1.0); // Initial time allocation
    opt_vars.points.resize(3, piece_num - 1);
    
    // Initialize points from guide path (excluding start/end)
    for (int i = 0; i < piece_num - 1; ++i) {
        if (i < guide_path.size()) {
            opt_vars.points.col(i) = guide_path[i];
        } else {
            opt_vars.points.col(i) = (start_pos + end_pos) * 0.5;
        }
    }

    // Set Minco conditions
    Eigen::Matrix<double, 3, 4> headState, tailState;
    headState.setZero(); tailState.setZero();
    headState.col(0) = start_pos;
    headState.col(1) = start_vel;
    tailState.col(0) = end_pos;
    tailState.col(1) = end_vel;
    
    opt_vars.minco.setConditions(headState, tailState, piece_num);

    // L-BFGS Optimization
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 16;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 1.0e-5;
    lbfgs_params.min_step = 1.0e-20;
    lbfgs_params.delta = 1.0e-5;

    Eigen::VectorXd x(piece_num + (piece_num - 1) * 3);
    // Map times and points to x
    // Note: This mapping needs to match costFunctional decoding
    // Simple mapping: [times, points]
    x.head(piece_num) = opt_vars.times;
    if (piece_num > 1) {
        Eigen::Map<Eigen::VectorXd> points_vec(opt_vars.points.data(), opt_vars.points.size());
        x.tail((piece_num - 1) * 3) = points_vec;
    }

    double final_cost;
    int ret = lbfgs::lbfgs_optimize(x, final_cost, ExpTrajOpt::costFunctional, NULL, NULL, this, lbfgs_params);

    if (ret < 0) {
        // Optimization failed or stopped
        // std::cout << "L-BFGS failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    }

    // Retrieve optimized trajectory
    // Update opt_vars from x
    opt_vars.times = x.head(piece_num);
    if (piece_num > 1) {
        Eigen::Map<const Eigen::VectorXd> points_vec(x.tail((piece_num - 1) * 3).data(), (piece_num - 1) * 3);
        opt_vars.points = Eigen::Map<const Eigen::Matrix3Xd>(points_vec.data(), 3, piece_num - 1);
    }
    
    opt_vars.minco.setParameters(opt_vars.points, opt_vars.times);
    Trajectory traj;
    opt_vars.minco.getTrajectory(traj);

    // Sample trajectory for output
    optimized_path.clear();
    double total_time = opt_vars.times.sum();
    double dt = 0.1;
    for (double t = 0; t <= total_time; t += dt) {
        // optimized_path.push_back(traj.getPos(t)); // Need to implement getPos in Trajectory or use Minco to eval
        // For now, just push back waypoints to show something
    }
    
    // Add waypoints
    optimized_path.push_back(start_pos);
    for(int i=0; i<opt_vars.points.cols(); ++i) {
        optimized_path.push_back(opt_vars.points.col(i));
    }
    optimized_path.push_back(end_pos);

    return true;
}

double ExpTrajOpt::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    ExpTrajOpt *obj = static_cast<ExpTrajOpt *>(ptr);
    int piece_num = obj->opt_vars.times.size();
    
    // Decode x
    Eigen::VectorXd times = x.head(piece_num);
    Eigen::Matrix3Xd points;
    if (piece_num > 1) {
        Eigen::Map<const Eigen::VectorXd> points_vec(x.tail((piece_num - 1) * 3).data(), (piece_num - 1) * 3);
        points = Eigen::Map<const Eigen::Matrix3Xd>(points_vec.data(), 3, piece_num - 1);
    } else {
        points.resize(3, 0);
    }

    // Calculate cost and gradient
    double cost = 0.0;
    g.setZero();
    
    // Time regularization
    cost += obj->opt_vars.rho * times.sum();
    g.head(piece_num).array() += obj->opt_vars.rho;

    // Minco Energy (Snap)
    obj->opt_vars.minco.setParameters(points, times);
    double energy;
    obj->opt_vars.minco.getEnergy(energy);
    cost += energy;
    
    Eigen::VectorXd gdT;
    Eigen::MatrixX3d gdC;
    obj->opt_vars.minco.getEnergyPartialGradByTimes(gdT);
    obj->opt_vars.minco.getEnergyPartialGradByCoeffs(gdC);
    
    // Propagate gradients
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    obj->opt_vars.minco.propogateGrad(gdC, gdT, gradByPoints, gradByTimes);
    
    g.head(piece_num) += gradByTimes;
    if (piece_num > 1) {
        Eigen::Map<Eigen::VectorXd> gradPointsVec(gradByPoints.data(), gradByPoints.size());
        g.tail((piece_num - 1) * 3) += gradPointsVec;
    }

    // TODO: Add Attraction, Vel, Acc, Jerk costs
    // This requires evaluating the trajectory and its derivatives, and adding penalties.
    // For brevity in this rough framework, we skip the detailed implementation of these penalties
    // but the structure is here to add them.

    return cost;
}

} // namespace traj_opt
