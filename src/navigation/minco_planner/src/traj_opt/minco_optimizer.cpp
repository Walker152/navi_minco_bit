#include "traj_opt/minco_optimizer.hpp"

#define POS_IDX 0
#define VEL_IDX 1
#define ACC_IDX 2
#define ATT_IDX 3
namespace minco_planner {
using Mat63f = Eigen::Matrix<double, 6, 3>;

double MincoOptimizer::optimize(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state,
  const VecDf & local_magnitudes,
  geometry_utils::Trajectory & out_traj)
{
  // 1. Setup the optimization problem
  last_iteration_count_ = 0;
  last_return_code_ = 0;
  last_objective_total_ = std::numeric_limits<double>::quiet_NaN();
  last_query_failure_count_ = 0;
  opt_vars_.query_failure_count = 0;

  if (!setupProblemAndCheck(waypoints, start_state, end_state)) {
    cout << YELLOW << " -- [TrajOpt] Error in setup problem, force return." << RESET << endl;
    last_return_code_ = lbfgs::LBFGSERR_INVALIDPARAMETERS;
    return INFINITY;
  }

  // 2. Pack variables x = [tau, xi]
  // tau: unconstrained time variables
  // xi: stacked intermediate control points
  // [Theory] We optimize an unconstrained vector and map tau -> positive segment times.
  VecDf x(opt_vars_.dim_t + opt_vars_.dim_p);
  Eigen::Map<VecDf> tau(x.data(), opt_vars_.dim_t);
  Eigen::Map<VecDf> xi(x.data() + opt_vars_.dim_t, opt_vars_.dim_p);
  opt_vars_.local_magnitudes = local_magnitudes;
  opt_vars_.penalty_log.resize(6);  // Energy, pos, vel, acc, attract, time_barrier
  opt_vars_.penalty_log.setZero();

  if (opt_vars_.times.minCoeff() < 1e-3) {
    cout << YELLOW << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
    cout << " -- Head PVA: " << endl;
    cout << opt_vars_.headPVA << endl;
    cout << " -- Tail PVA: " << endl;
    cout << opt_vars_.tailPVA << endl;
    cout << " -- Times: " << endl;
    cout << opt_vars_.times.transpose() << endl;
    return INFINITY;
  }

  gcopter::backwardMapTToTau(opt_vars_.times, tau);
  xi = Eigen::Map<const VecDf>(opt_vars_.points.data(), opt_vars_.points.size());

  opt_vars_.iter_num = 0;
  double minCostFunctional = 0.0;

  // 3. Configure L-BFGS
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 3;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.delta = cfg_.opt_accuracy;  // Gradient tolerance
  lbfgs_params.max_iterations = 256;

  // 4. Run the optimizer
  // [Theory] L-BFGS is a limited-memory quasi-Newton method.
  int ret = lbfgs::lbfgs_optimize(x,
    minCostFunctional,
    &MincoOptimizer::costFunctional,
    nullptr,
    nullptr,
    &this->opt_vars_,
    lbfgs_params);
  last_iteration_count_ = opt_vars_.iter_num;
  last_return_code_ = ret;
  last_objective_total_ = minCostFunctional;
  last_query_failure_count_ = opt_vars_.query_failure_count;

  if (cfg_.print_optimizer_log) {
    cout << " -- [MincoOpt] Opt finish, with iter num: " << opt_vars_.iter_num << "\n";
    cout << "\tEnergy: " << opt_vars_.penalty_log(0) << endl;
    cout << "\tPos: " << opt_vars_.penalty_log(1) << endl;
    cout << "\tVel: " << opt_vars_.penalty_log(2) << endl;
    cout << "\tAcc: " << opt_vars_.penalty_log(3) << endl;
    cout << "\tAttract: " << opt_vars_.penalty_log(4) << endl;
    cout << "\tTime Barrier: " << opt_vars_.penalty_log(5) << endl;
    cout << "\tOptimized Time: " << opt_vars_.times.norm() << endl;
  }

  if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMITERATION) {
    // 5. Decode the solution and build the output trajectory
    gcopter::forwardMapTauToT(tau, opt_vars_.times);
    opt_vars_.points = Eigen::Map<const Mat3Df>(xi.data(), 3, opt_vars_.piece_num - 1);

    // opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, opt_vars_.piece_num);
    opt_vars_.minco_solver_->setParameters(opt_vars_.points, opt_vars_.times);
    opt_vars_.minco_solver_->getTrajectory(out_traj);

    opt_vars_.init_ts = opt_vars_.times;
    opt_vars_.init_ps.clear();
    opt_vars_.init_ps.reserve(static_cast<size_t>(opt_vars_.points.cols()));
    for (int i = 0; i < opt_vars_.points.cols(); ++i) {
      opt_vars_.init_ps.emplace_back(opt_vars_.points.col(i));
    }
    opt_vars_.default_init = false;
  } else {
    // 5'. Optimization failed
    minCostFunctional = INFINITY;
  }
  return minCostFunctional + ret;
}

double MincoOptimizer::costFunctional(void * ptr, const VecDf & x, VecDf & g)
{
  OptVars & opt_vars_ = *(static_cast<OptVars *>(ptr));
  const auto & dim_t = opt_vars_.dim_t;
  const auto & dim_p = opt_vars_.dim_p;
  const auto & waypoint_attractor = opt_vars_.waypoint_attractor;
  const auto & integral_res = opt_vars_.integral_res;
  const auto & smooth_eps = opt_vars_.smooth_eps;
  const auto & rho = opt_vars_.rho;
  const auto & penaltyWeights = opt_vars_.penaltyWeights;
  const auto & magnitudeBounds = opt_vars_.magnitudeBounds;
  const auto & local_magnitudes = opt_vars_.local_magnitudes;
  const auto & minco_solver_ = opt_vars_.minco_solver_;
  const auto & map = opt_vars_.map;

  opt_vars_.iter_num++;

  // 1. Unpack variables and gradient buffers
  const Eigen::Map<const VecDf> tau(x.data(), dim_t);
  const Eigen::Map<const VecDf> xi(x.data() + dim_t, dim_p);
  Eigen::Map<VecDf> grad_tau(g.data(), dim_t);
  Eigen::Map<VecDf> grad_points(g.data() + dim_t, dim_p);

  // 2. Map tau -> segment times
  VecDf times;
  gcopter::forwardMapTauToT(tau, times);

  Mat3Df points;
  if (dim_p > 0) {
    points = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi.data(), 3, dim_p / 3);
  } else {
    points.resize(3, 0);
  }

  // 3. Update MINCO polynomial parameters
  minco_solver_->setParameters(points, times);

  // 4. Energy term (MINCO internal objective)
  double cost = 0.0;
  MatD3f partialGradByCoeffs(6 * times.size(), 3);
  VecDf partialGradByTimes(times.size());
  partialGradByCoeffs.setZero();
  partialGradByTimes.setZero();
  minco_solver_->getEnergy(cost);
  minco_solver_->getEnergyPartialGradByCoeffs(partialGradByCoeffs);
  minco_solver_->getEnergyPartialGradByTimes(partialGradByTimes);
  opt_vars_.penalty_log(0) = cost;

  // 5. Penalty terms (ESDF / vel / acc / attract)
  constraintsFunctional(times,
    minco_solver_->getCoeffs(),
    waypoint_attractor,
    map,
    smooth_eps,
    integral_res,
    magnitudeBounds,
    local_magnitudes,
    penaltyWeights,
    cost,
    partialGradByTimes,
    partialGradByCoeffs,
    opt_vars_.penalty_log,
    opt_vars_.query_failure_count);

  // 6. Propagate gradients to points and times
  Mat3Df gradByPoints;
  VecDf gradByTimes;
  minco_solver_->propogateGrad(partialGradByCoeffs, partialGradByTimes, gradByPoints, gradByTimes);

  // 7. Time regularization
  cost += rho * times.sum();
  gradByTimes.array() += rho;

  // 7.5 Kinematic Time Barrier
  computeTimeBarrier(opt_vars_, times, magnitudeBounds, cost, gradByTimes, opt_vars_.penalty_log);

  // 8. Backprop time gradient (T -> tau)
  gcopter::propagateGradientTToTau(tau, gradByTimes, grad_tau);

  if (dim_p > 0) {
    grad_points = Eigen::Map<VecDf>(gradByPoints.data(), gradByPoints.size());
  }

  return cost;
}

void MincoOptimizer::computeTimeBarrier(const OptVars & opt_vars,
  const VecDf & times,
  const VecDf & magnitudeBounds,
  double & cost,
  VecDf & gradByTimes,
  VecDf & penalty_log)
{
  const int N = static_cast<int>(times.size());
  const auto & w_barrier = opt_vars.penaltyWeights(4);  // Time barrier weight
  // const double vmax_safe = std::max(1e-3, magnitudeBounds[1] * 0.6);
  const double amax_safe = std::max(1e-3, magnitudeBounds[2]);
  const double v_curr = opt_vars.headPVA.col(1).norm();
  const double v_tail = opt_vars.tailPVA.col(1).norm();
  const bool has_init_ps = (opt_vars.init_ps.size() == static_cast<size_t>(std::max(0, N - 1)));

  for (int i = 0; i < N; ++i) {
    const double t_i = times(i);
    const double local_vmax = opt_vars.local_magnitudes(i);
    const double vmax_safe = std::max(1e-3, local_vmax);
    Eigen::Vector3d p_start = opt_vars.headPVA.col(0);
    Eigen::Vector3d p_end = opt_vars.tailPVA.col(0);

    if (i > 0) {
      p_start =
        has_init_ps ? opt_vars.init_ps[static_cast<size_t>(i - 1)] : opt_vars.waypoint_attractor.col(i);
    }
    if (i < N - 1) {
      p_end =
        has_init_ps ? opt_vars.init_ps[static_cast<size_t>(i)] : opt_vars.waypoint_attractor.col(i + 1);
    }

    const double dist = (p_end - p_start).norm();
    double t_min = dist / vmax_safe;

    if (i == N - 1 && v_tail < 0.1) {
      t_min = std::max({t_min, v_curr / amax_safe, vmax_safe / amax_safe});
    }

    if (t_i < t_min) {
      const double violation = t_min - t_i;
      const double violation2 = violation * violation;
      cost += w_barrier * violation2 * violation;
      gradByTimes(i) += -3.0 * w_barrier * violation2;
      penalty_log(5) = violation;
    }
  }
}

void MincoOptimizer::constraintsFunctional(const VecDf & T,
  const MatD3f & coeffs,
  const Mat3Df & waypoint_attractor,
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const double & smooth_eps,
  const int & integral_res,
  const VecDf & magnitudeBounds,
  const VecDf & local_magnitudes,
  const VecDf & penaltyWeights,
  // outputs
  double & cost,
  VecDf & partialGradByTimes,
  MatD3f & partialGradByCoeffs,
  VecDf & penalty_log,
  uint64_t & query_failure_count)
{
  // 1. Unpack bounds and weights
  const auto & safe_dist = magnitudeBounds[0];
  // const auto & vmax = magnitudeBounds[1];
  const auto & amax = magnitudeBounds[2];

  // const auto & vmaxSqr = vmax * vmax;
  const auto & amaxSqr = amax * amax;

  const auto & weightPos = penaltyWeights[0];
  const auto & weightVel = penaltyWeights[1];
  const auto & weightAcc = penaltyWeights[2];
  const auto & weightAtt = penaltyWeights[3];

  const auto & piece_num = T.size();

  // 2. Setup piece-wise sampling
  const double integralFrac = 1.0 / integral_res;
  VecDf max_pena(4);
  max_pena.setZero();

  // 3. Loop over pieces and samples
  for (int i = 0; i < piece_num; i++) {
    const Mat63f & c = coeffs.block<6, 3>(i * 6, 0);
    const auto & step = T(i) * integralFrac;
    const auto & local_vmax = local_magnitudes(i);
    const auto & local_vmaxSqr = local_vmax * local_vmax;
    for (int j = 0; j <= integral_res; j++) {
      double s1 = j * step;
      double s2 = s1 * s1;
      double s3 = s2 * s1;
      double s4 = s2 * s2;
      double s5 = s4 * s1;
      Vec6f beta0, beta1, beta2, beta3, beta4;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      // beta4 << 0.0, 0.0, 0.0, 0., 0.0, 120.0;

      const Vec3f pos = c.transpose() * beta0;
      const Vec3f vel = c.transpose() * beta1;
      const Vec3f acc = c.transpose() * beta2;
      const Vec3f jer = c.transpose() * beta3;

      double tmp_cost{0.0};
      Vec3f gradPos{0.0, 0.0, 0.0}, gradVel{0.0, 0.0, 0.0}, gradAcc{0.0, 0.0, 0.0};

      // For position cost
      if (weightPos > 0.0 && map) {
        double esdf_dist = -safe_dist;
        Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
        const auto query = map->query(pos.cast<double>());
        if (query.ok) {
          esdf_dist = query.distance;
          esdf_grad = query.gradient;
        } else {
          ++query_failure_count;
        }

        const double violaPos = safe_dist - esdf_dist;
        double violaPosPena, violaPosPenaD;
        if (gcopter::smoothedL1(violaPos, smooth_eps, violaPosPena, violaPosPenaD)) {
          // d(violaPos)/dpos = -grad(dist)
          gradPos += (-weightPos * violaPosPenaD) * esdf_grad.cast<double>();
          tmp_cost += weightPos * violaPosPena;
          if (violaPos > max_pena(POS_IDX))
            max_pena(POS_IDX) = violaPos;
        }
      }

      // For velocity cost
      const auto & violaVel = vel.squaredNorm() - local_vmaxSqr;
      double violaVelPena, violaVelPenaD;
      if (weightVel > 0 && gcopter::smoothedL1(violaVel, smooth_eps, violaVelPena, violaVelPenaD)) {
        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
        tmp_cost += weightVel * violaVelPena;
        if (violaVel > max_pena(VEL_IDX))
          max_pena(VEL_IDX) = violaVel;
      }

      // For acceleration cost
      const auto & violaAcc = acc.squaredNorm() - amaxSqr;
      double violaAccPena, violaAccPenaD;
      if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smooth_eps, violaAccPena, violaAccPenaD)) {
        gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
        tmp_cost += weightAcc * violaAccPena;
        if (violaAcc > max_pena(ACC_IDX))
          max_pena(ACC_IDX) = violaAcc;
      }

      // For attract point cost
      if (weightAtt > 0.0) {
        const auto is_end = ((j == integral_res) && (i != piece_num - 1));
        const auto idx = is_end ? i + 1 : i;
        if (is_end) {
          Vec3f p_a = pos - waypoint_attractor.col(idx);
          const auto & violaAtt = p_a.squaredNorm() - 0.1 * 0.1;  // Dead zone: 0.1m
          double violaAttPena, violaAttPenaD;
          if (violaAtt > max_pena(ATT_IDX))
            max_pena(ATT_IDX) = violaAtt;
          if (gcopter::smoothedL1(violaAtt, smooth_eps, violaAttPena, violaAttPenaD)) {
            Vec3f gradAtt = weightAtt * violaAttPenaD * 2.0 * p_a;
            partialGradByCoeffs.block<6, 3>(i * 6, 0) += beta0 * gradAtt.transpose();
            cost += weightAtt * violaAttPena;
            if (is_end) {
              partialGradByTimes(i) += gradAtt.dot(vel);
            }
          }
        }
      }

      const auto node = (j == 0 || j == integral_res) ? 0.5 : 1.0;
      const double alpha = j * integralFrac;
      partialGradByCoeffs.block<6, 3>(i * 6, 0) +=
        (beta0 * gradPos.transpose() + beta1 * gradVel.transpose() + beta2 * gradAcc.transpose()) * node *
        step;
      partialGradByTimes(i) +=
        (gradPos.dot(vel) + gradVel.dot(acc) + gradAcc.dot(jer)) * alpha * node * step +
        node * integralFrac * tmp_cost;
      cost += node * step * tmp_cost;
    }
  }

  // 4. Save penalty logs
  penalty_log(1) = max_pena(POS_IDX);
  penalty_log(2) = max_pena(VEL_IDX);
  penalty_log(3) = max_pena(ACC_IDX);
  penalty_log(4) = max_pena(ATT_IDX);
}

bool MincoOptimizer::setupProblemAndCheck(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state)
{
  // 1. Decide the number of polynomial pieces
  int N = waypoints.size() - 1;
  if (N <= 0) {
    return false;
  }

  // 2. Allocate optimization context
  opt_vars_.piece_num = N;
  opt_vars_.headPVA = start_state;
  opt_vars_.tailPVA = end_state;
  opt_vars_.times.resize(N);
  opt_vars_.points.resize(3, N - 1);
  opt_vars_.waypoint_attractor.resize(3, N + 1);
  for (int i = 0; i < N - 1; ++i) {
    opt_vars_.waypoint_attractor.col(i + 1) = waypoints[i + 1];
  }
  opt_vars_.waypoint_attractor.col(0) = opt_vars_.headPVA.col(0);
  opt_vars_.waypoint_attractor.rightCols(1) = opt_vars_.tailPVA.col(0);

  // 3. Set variable dimensions
  opt_vars_.dim_t = N;
  opt_vars_.dim_p = 3 * (N - 1);

  // 4. Allocate gradient buffers
  opt_vars_.gradByPoints.resize(3, N - 1);
  opt_vars_.gradByTimes.resize(N);
  opt_vars_.partialGradByCoeffs.resize(6 * N, 3);
  opt_vars_.partialGradByTimes.resize(N);

  // 5. Initialize time and point guesses
  DefaultInit();

  // Apply warm-start (shifted hot start) if provided and sizes match.
  if (!opt_vars_.default_init) {
    const bool ts_ok = (opt_vars_.init_ts.size() == opt_vars_.times.size());
    const bool ps_ok = (opt_vars_.init_ps.size() == static_cast<size_t>(opt_vars_.points.cols()));
    if (ts_ok && ps_ok) {
      opt_vars_.times = opt_vars_.init_ts;
      for (int i = 0; i < opt_vars_.points.cols(); ++i) {
        opt_vars_.points.col(i) = opt_vars_.init_ps[static_cast<size_t>(i)];
      }
    } else {
      // Size mismatch: fallback to cold init.
      opt_vars_.default_init = true;
    }
  }

  // 6. Set boundary conditions for MINCO
  opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, N);
  return true;
}

void MincoOptimizer::DefaultInit()
{
  // 1. Extract segment distance computation logic.
  const auto computeDis = [this]() -> VecDf {
    return (opt_vars_.waypoint_attractor.leftCols(opt_vars_.piece_num) -
            opt_vars_.waypoint_attractor.rightCols(opt_vars_.piece_num))
      .colwise()
      .norm()
      .transpose();
  };
  const VecDf dis = computeDis();

  // 2. Allocate initial times with kinematic-aware startup / stopping compensation.
  const double max_vel = cfg_.max_vel;
  const double max_acc = cfg_.max_acc;
  const double cruise_speed = std::max(1e-3, max_vel);
  const double v_tail = opt_vars_.tailPVA.col(1).norm();
  const bool is_stopping = (v_tail < 0.1);

  opt_vars_.times.resize(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    const double d = std::max(0.0, dis(i));
    double t = 0.1;
    if (i == 0) {
      // First segment: conservative startup time to avoid speed spike.
      const double t_acc = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      const double t_cons = d / std::max(1e-3, max_vel * 0.5);
      t = std::max(t_acc, t_cons);
    } else if (i == opt_vars_.piece_num - 1 && is_stopping) {
      // Last segment: add braking compensation for stop behavior.
      const double t_cruise = d / cruise_speed;
      const double t_brake = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      t = std::max(t_cruise, t_brake);
    } else {
      // Middle segments: keep original uniform-speed allocation.
      t = d / cruise_speed;
    }

    opt_vars_.times(i) = t;
  }
  opt_vars_.times = opt_vars_.times.cwiseMax(0.1);

  // 3. Initialize intermediate points from guide waypoints (unchanged).
  opt_vars_.points = opt_vars_.waypoint_attractor.block(0, 1, 3, opt_vars_.piece_num - 1);
}

void MincoOptimizer::setInitPsAndTs(const vec_Vec3f & init_ps, const VecDf & init_ts)
{
  // Store warm-start guesses. They will be applied after problem dimensions
  // are known (inside setupProblemAndCheck()).
  if (init_ps.empty() || init_ts.size() == 0) {
    opt_vars_.default_init = true;
    opt_vars_.init_ps.clear();
    opt_vars_.init_ts.resize(0);
    return;
  }

  opt_vars_.default_init = false;
  opt_vars_.init_ps = init_ps;
  opt_vars_.init_ts = init_ts;
}
}  // namespace minco_planner
