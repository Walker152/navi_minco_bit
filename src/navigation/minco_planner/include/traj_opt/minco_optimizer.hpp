#ifndef MINCO_PLANNER__MINCO_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "data_structure/base/trajectory.h"
#include "traj_opt/minco.h"
#include "utils/header/color_text.hpp"
#include "utils/header/eigen_alias.hpp"
#include "utils/optimization/lbfgs.h"
#include "utils/optimization/optimization_utils.h"

#include "small_rog_map/hybrid_esdf_map.hpp"

namespace minco_planner {

using namespace super_utils;
using namespace traj_opt;
using namespace optimization_utils;
using namespace math_utils;
using namespace color_text;

class MincoOptimizer
{
public:
  // === Internal Types ===
  // --- Optimizer Configuration ---
  struct Config
  {
    double safe_dist{0.3};
    double max_vel{5.0};
    double max_acc{5.0};
    double turn_angle_deadzone{0.174};
    double turn_angle_saturation{1.57};
    double min_turn_vel{1.0};
    double decay_power{2.0};

    double rho{0.01};
    double smooth_eps{0.01};

    VecDf magnitudeBounds, penaltyWeights;
    int time_allocation_iters{5};
    int integral_res{16};
    double opt_accuracy{1e-4};

    bool print_optimizer_log{true};
  } cfg_;

  // === Constructor & Lifecycle ===
  MincoOptimizer(const Config & cfg) : cfg_(cfg)
  {
    opt_vars_.minco_solver_ = std::make_shared<traj_opt::MINCO_S3NU>();
    opt_vars_.magnitudeBounds.resize(cfg_.magnitudeBounds.size());
    opt_vars_.penaltyWeights.resize(cfg_.penaltyWeights.size());
    opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
    opt_vars_.penaltyWeights = cfg_.penaltyWeights;
    opt_vars_.rho = cfg_.rho;
    opt_vars_.smooth_eps = cfg_.smooth_eps;
    opt_vars_.integral_res = cfg_.integral_res;
  }

  // === Core Planning Interfaces ===
  // --- Configuration and Initialization ---
  void setConfig(const Config & cfg)
  {
    cfg_ = cfg;
    opt_vars_.magnitudeBounds.resize(cfg_.magnitudeBounds.size());
    opt_vars_.penaltyWeights.resize(cfg_.penaltyWeights.size());
    opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
    opt_vars_.penaltyWeights = cfg_.penaltyWeights;
    opt_vars_.rho = cfg_.rho;
    opt_vars_.smooth_eps = cfg_.smooth_eps;
    opt_vars_.integral_res = cfg_.integral_res;
  }

  void setInitPsAndTs(const vec_Vec3f & init_ps, const VecDf & init_ts);

  void setESDFMap(const small_rog_map::HybridESDFMap::Ptr & esdf_map)
  {
    opt_vars_.hybrid_esdf_map = esdf_map;
  }

  // --- Trajectory Optimization ---
  double optimize(const std::vector<Eigen::Vector3d> & waypoints,
    const Eigen::Matrix3d & start_state,
    const Eigen::Matrix3d & end_state,
    const VecDf & local_magnitudes,
    geometry_utils::Trajectory & out_traj);

private:
  // === Internal Types ===
  // --- Optimization Runtime Variables ---
  struct OptVars
  {
    int piece_num;
    int dim_t;
    int dim_p;
    int iter_num{0};
    double rho;
    double smooth_eps;
    double integral_res;
    bool default_init{true};

    // Environment map pointer.
    small_rog_map::HybridESDFMap::Ptr hybrid_esdf_map;

    VecDf magnitudeBounds;
    VecDf penaltyWeights;
    VecDf local_magnitudes;

    Eigen::Matrix3d headPVA;
    Eigen::Matrix3d tailPVA;
    Mat3Df waypoint_attractor;

    // Optimization cache for warm-start initialization from previous solution.
    VecDf init_ts;
    vec_Vec3f init_ps;

    // Optimization variables.
    VecDf times;
    Mat3Df points;
    VecDf x;

    // Gradient cache.
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    MatD3f partialGradByCoeffs;
    VecDf partialGradByTimes;

    // Result cache.
    VecDf penalty_log;

    // MINCO solver instance.
    std::shared_ptr<traj_opt::MINCO_S3NU> minco_solver_;
  };

  // === Utility & Helper Functions ===
  // --- Problem Setup ---

  bool setupProblemAndCheck(const std::vector<Eigen::Vector3d> & waypoints,
    const Eigen::Matrix3d & start_state,
    const Eigen::Matrix3d & end_state);

  void DefaultInit();

  static double costFunctional(void * ptr, const VecDf & x, VecDf & g);

  // --- Constraint and Barrier Functions ---
  static void constraintsFunctional(const VecDf & T,
    const MatD3f & coeffs,
    const Mat3Df & waypoint_attractor,
    const small_rog_map::HybridESDFMap::Ptr & hybrid_esdf_map,
    const double & smooth_eps,
    const int & integral_res,
    const VecDf & magnitudeBounds,
    const VecDf & local_magnitudes,
    const VecDf & penaltyWeights,
    double & cost,
    VecDf & partialGradByTimes,
    MatD3f & partialGradByCoeffs,
    VecDf & penalty_log);

  static void computeTimeBarrier(const OptVars & opt_vars,
    const VecDf & times,
    const VecDf & magnitudeBounds,
    double & cost,
    VecDf & gradByTimes,
    VecDf & penalty_log);

  // --- Time Reparameterization ---
  static void forwardT(const VecDf & tau, VecDf & T) { T = tau.array().exp(); }
  static void backwardT(const VecDf & T, VecDf & tau) { tau = T.array().log(); }

  // === State Variables & Caches ===
  OptVars opt_vars_;
};

}  // namespace minco_planner

#endif