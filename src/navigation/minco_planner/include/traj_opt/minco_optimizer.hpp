#ifndef MINCO_PLANNER__MINCO_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>

#include "utils/header/eigen_alias.hpp"            
#include "traj_opt/minco.h"           
#include "data_structure/base/trajectory.h" 
#include "utils/optimization/lbfgs.h" 
#include "utils/optimization/optimization_utils.h"
#include "utils/header/color_text.hpp"

#include "minco_core/static_esdf_map.hpp"

namespace minco_planner {

using namespace super_utils;
using namespace traj_opt;
using namespace optimization_utils;
using namespace math_utils;
using namespace color_text;

class MincoOptimizer {
public:
    struct Config {
        double safe_dist{0.3};
        double max_vel{5.0};               
        double max_acc{5.0};
        
        double rho{0.01};
        double smooth_eps{0.01};

        VecDf magnitudeBounds, penaltyWeights;
        int time_allocation_iters{5};
        int integral_res{16};      
        double opt_accuracy{1e-4};

        bool print_optimizer_log{true};
    } cfg_;

    MincoOptimizer(const Config& cfg) : cfg_(cfg) {
        opt_vars_.minco_solver_ = std::make_shared<traj_opt::MINCO_S3NU>();
        opt_vars_.magnitudeBounds.resize(cfg_.magnitudeBounds.size());
        opt_vars_.penaltyWeights.resize(cfg_.penaltyWeights.size());
        opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
        opt_vars_.penaltyWeights = cfg_.penaltyWeights;
        opt_vars_.rho = cfg_.rho;
        opt_vars_.smooth_eps = cfg_.smooth_eps;
        opt_vars_.integral_res = cfg_.integral_res;
    }
    
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    void setInitPsAndTs(const vec_Vec3f& init_ps, const VecDf& init_ts);

    void setESDFMap(const StaticESDFMap::Ptr& esdf_map) {
        opt_vars_.static_esdf_map = esdf_map;
    }

    double optimize(const std::vector<Eigen::Vector3d>& waypoints,
                 const Eigen::Matrix3d& start_state,
                 const Eigen::Matrix3d& end_state,
                 geometry_utils::Trajectory& out_traj);

private:
    struct OptVars {
        int piece_num;
        int dim_t; 
        int dim_p;
        int iter_num{0}; 
        double rho;
        double smooth_eps;
        double integral_res;
        bool default_init{true};

        // 环境地图指针
        StaticESDFMap::Ptr static_esdf_map;

        VecDf magnitudeBounds;
        VecDf penaltyWeights;

        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        Mat3Df waypoint_attractor;

        // 优化变量缓存（热启动初始猜测，存储上一次优化结果）
        VecDf init_ts; 
        vec_Vec3f init_ps;        

        // 优化变量
        VecDf times;
        Mat3Df points;
        VecDf x; 

        // 梯度缓存
        Mat3Df gradByPoints;        
        VecDf gradByTimes;          
        MatD3f partialGradByCoeffs; 
        VecDf partialGradByTimes;   

        // 结果缓存
        VecDf penalty_log;

        // MINCO 求解器
        std::shared_ptr<traj_opt::MINCO_S3NU> minco_solver_;
    } opt_vars_;


    bool setupProblemAndCheck(const std::vector<Eigen::Vector3d>& waypoints,
                              const Eigen::Matrix3d& start_state,
                              const Eigen::Matrix3d& end_state);

    void DefaultInit();

    static double costFunctional(void* ptr, const VecDf& x, VecDf& g);

    static void constraintsFunctional(const VecDf& T, 
                               const MatD3f& coeffs,
                               const Mat3Df& waypoint_attractor,
                               const StaticESDFMap::Ptr& static_esdf_map,
                               const double& smooth_eps,
                               const int& integral_res,
                               const VecDf& magnitudeBounds,
                               const VecDf& penaltyWeights,
                               double& cost,
                               VecDf& partialGradByTimes,
                               MatD3f& partialGradByCoeffs,
                               VecDf& penalty_log);

    static void forwardT(const VecDf& tau, VecDf& T) { T = tau.array().exp(); }
    static void backwardT(const VecDf& T, VecDf& tau) { tau = T.array().log(); }
};

} // namespace minco_planner

#endif