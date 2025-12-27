#include "traj_opt/minco_optimizer.hpp"

#define POS_IDX 0
#define VEL_IDX 1
#define ACC_IDX 2
#define ATT_IDX 3
namespace minco_planner {
using Mat63f = Eigen::Matrix<double, 6, 3>;

    // 在 minco_optimizer.cpp 中
double MincoOptimizer::optimize(const std::vector<Eigen::Vector3d>& waypoints,
                             const Eigen::Matrix3d& start_state,
                             const Eigen::Matrix3d& end_state,
                             geometry_utils::Trajectory& out_traj) 
{
    // 1. 设置问题
    if (!setupProblemAndCheck(waypoints, start_state, end_state)) {
        cout << YELLOW << " -- [TrajOpt] Error in setup problem, force return." << RESET << endl;
        return INFINITY;
    }
    VecDf x(opt_vars_.dim_t + opt_vars_.dim_p);
    Eigen::Map<VecDf> tau(x.data(), opt_vars_.dim_t);
    Eigen::Map<VecDf> xi(x.data() + opt_vars_.dim_t, opt_vars_.dim_p);
    
    opt_vars_.penalty_log.resize(5); // energy, pos, vel, acc, attract
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
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = cfg_.opt_accuracy; // 确保 Config 里有这个，或者写死 1e-4
    lbfgs_params.max_iterations = 256;
    VecDf times_init = opt_vars_.times;

    // 4. 执行优化
    int ret = lbfgs::lbfgs_optimize(
        x,
        minCostFunctional,
        &MincoOptimizer::costFunctional,
        nullptr,
        nullptr,
        &this->opt_vars_,
        lbfgs_params
    );

    if (cfg_.print_optimizer_log) {
        cout << " -- [MincoOpt] Opt finish, with iter num: " << opt_vars_.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars_.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars_.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars_.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars_.penalty_log(3) << endl;
        cout << "\tAttract: " << opt_vars_.penalty_log(4) << endl;
        cout << "\tOptimized Time: " << opt_vars_.times.norm() << endl;
    }

    if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMITERATION) {
        gcopter::forwardMapTauToT(tau, opt_vars_.times);
        opt_vars_.points = Eigen::Map<const Mat3Df>(xi.data(), 3, opt_vars_.piece_num - 1);

        // opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, opt_vars_.piece_num);
        opt_vars_.minco_solver_->setParameters(opt_vars_.points, opt_vars_.times);
        opt_vars_.minco_solver_->getTrajectory(out_traj);
        
        opt_vars_.init_ts = opt_vars_.times;
        opt_vars_.init_ps.clear();
        for (int i = 0; i < opt_vars_.points.cols(); ++i) {
            opt_vars_.init_ps.emplace_back(opt_vars_.points.col(i));
        }
        opt_vars_.default_init = false;
    } else {
        // 优化失败，恢复初始值
        minCostFunctional = INFINITY;
    }
    return minCostFunctional + ret;
}

double MincoOptimizer::costFunctional(void *ptr, const VecDf& x, VecDf& g) 
{
    OptVars& opt_vars_ = *(static_cast<OptVars*>(ptr));
    const auto &dim_t = opt_vars_.dim_t;
    const auto &dim_p = opt_vars_.dim_p;
    const auto &waypoint_attractor = opt_vars_.waypoint_attractor;
    const auto &integral_res = opt_vars_.integral_res;
    const auto &smooth_eps = opt_vars_.smooth_eps;
    const auto &rho = opt_vars_.rho;
    const auto &penaltyWeights = opt_vars_.penaltyWeights;
    const auto &magnitudeBounds = opt_vars_.magnitudeBounds;
    const auto &minco_solver_ = opt_vars_.minco_solver_;
    const auto &static_esdf_map = opt_vars_.static_esdf_map;

    opt_vars_.iter_num++;

    const Eigen::Map<const VecDf> tau(x.data(), dim_t);
    const Eigen::Map<const VecDf> xi(x.data() + dim_t, dim_p);
    Eigen::Map<VecDf> grad_tau(g.data(), dim_t);
    Eigen::Map<VecDf> grad_points(g.data() + dim_t, dim_p);
    
    // 时间变量转换
    VecDf times;
    gcopter::forwardMapTauToT(tau, times);
    
    Mat3Df points;
    if (dim_p > 0) {
        points = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi.data(), 3, dim_p / 3);
    } else {
        points.resize(3, 0);
    }
    
    // 设置 MINCO_S3NU 参数
    minco_solver_->setParameters(points, times);
    
    // 计算能量和梯度
    double cost = 0.0;
    MatD3f partialGradByCoeffs(6 * times.size(), 3);
    VecDf partialGradByTimes(times.size());
    partialGradByCoeffs.setZero();
    partialGradByTimes.setZero();
    minco_solver_->getEnergy(cost);
    minco_solver_->getEnergyPartialGradByCoeffs(partialGradByCoeffs);
    minco_solver_->getEnergyPartialGradByTimes(partialGradByTimes);
    opt_vars_.penalty_log(0) = cost;

    // 计算约束代价和梯度
        constraintsFunctional(times, minco_solver_->getCoeffs(),
                                                waypoint_attractor,
                                                    static_esdf_map,
                                                    smooth_eps, integral_res,
                                                    magnitudeBounds, penaltyWeights,
                                                    cost,
                                                    partialGradByTimes,
                                                    partialGradByCoeffs,
                                                    opt_vars_.penalty_log);
    
    // 传播梯度到点和时间
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    minco_solver_->propogateGrad(partialGradByCoeffs,
                                 partialGradByTimes,
                                 gradByPoints,
                                 gradByTimes);
    cost += rho * times.sum();
    gradByTimes.array() += rho;
    
    
    // 时间梯度反向传播 (T -> tau)
    gcopter::propagateGradientTToTau(tau, gradByTimes, grad_tau);
    
    if (dim_p > 0) {
        grad_points = Eigen::Map<VecDf>(gradByPoints.data(), gradByPoints.size());
    }


    return cost;
}

void MincoOptimizer::constraintsFunctional(const VecDf& T, 
                               const MatD3f& coeffs,
                               const Mat3Df& waypoint_attractor,
                               const StaticESDFMap::Ptr& static_esdf_map,
                               const double& smooth_eps,
                               const int& integral_res,
                               const VecDf& magnitudeBounds,
                               const VecDf& penaltyWeights,
                               // outputs
                               double& cost,
                               VecDf& partialGradByTimes,
                               MatD3f& partialGradByCoeffs,
                               VecDf& penalty_log) 
{
    const auto &safe_dist = magnitudeBounds[0];
    const auto &vmax = magnitudeBounds[1];
    const auto &amax = magnitudeBounds[2];

    const auto &vmaxSqr = vmax * vmax;
    const auto &amaxSqr = amax * amax;

    const auto &weightPos = penaltyWeights[0];
    const auto &weightVel = penaltyWeights[1];
    const auto &weightAcc = penaltyWeights[2];
    const auto &weightAtt = penaltyWeights[3];

    const auto &piece_num = T.size();

    const double integralFrac = 1.0 / integral_res;
    VecDf max_pena(4);
    max_pena.setZero();

    for (int i = 0; i < piece_num; i++)
    {
        const Mat63f &c = coeffs.block<6, 3>(i * 6, 0);
        const auto &step = T(i) * integralFrac;
        for (int j = 0; j <= integral_res; j++)
        {
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
            //beta4 << 0.0, 0.0, 0.0, 0., 0.0, 120.0;

            const Vec3f pos = c.transpose() * beta0;
            const Vec3f vel = c.transpose() * beta1;
            const Vec3f acc = c.transpose() * beta2;
            const Vec3f jer = c.transpose() * beta3;
            
            double tmp_cost{0.0};
            Vec3f gradPos{0.0, 0.0, 0.0}, gradVel{0.0, 0.0, 0.0}, gradAcc{0.0, 0.0, 0.0};

            // For position cost
            if (weightPos > 0.0 && static_esdf_map) {
                double esdf_dist = 0.0;
                Eigen::Vector3d esdf_grad;
                static_esdf_map->evaluate(pos.cast<double>(), esdf_dist, esdf_grad);

                const double violaPos = safe_dist - esdf_dist;
                double violaPosPena, violaPosPenaD;
                if (gcopter::smoothedL1(violaPos, smooth_eps, violaPosPena, violaPosPenaD)) {
                    // d(violaPos)/dpos = -grad(dist)
                    gradPos += (-weightPos * violaPosPenaD) * esdf_grad.cast<double>();
                    tmp_cost += weightPos * violaPosPena;
                    if (violaPos > max_pena(POS_IDX)) max_pena(POS_IDX) = violaPos;
                }
            }

            // For velocity cost
            const auto &violaVel = vel.squaredNorm() - vmaxSqr;
            double violaVelPena, violaVelPenaD;
            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smooth_eps, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                tmp_cost += weightVel * violaVelPena;
                if (violaVel > max_pena(VEL_IDX)) max_pena(VEL_IDX) = violaVel;
            }

            // For acceleration cost
            const auto &violaAcc = acc.squaredNorm() - amaxSqr;
            double violaAccPena, violaAccPenaD;
            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smooth_eps, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                tmp_cost += weightAcc * violaAccPena;
                if (violaAcc > max_pena(ACC_IDX)) max_pena(ACC_IDX) = violaAcc;
            }

            // For attract point cost
            if (weightAtt > 0.0) {
                const auto is_end = ((j == integral_res) && (i != piece_num - 1));
                const auto idx = is_end ? i + 1 : i;
                if (is_end) {
                    Vec3f p_a = pos - waypoint_attractor.col(idx);
                    const auto &violaAtt =
                            p_a.squaredNorm() - 0.1 * 0.1; // dead zone 0.1m
                    double violaAttPena, violaAttPenaD;
                    if (violaAtt > max_pena(ATT_IDX)) max_pena(ATT_IDX) = violaAtt;
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
            partialGradByCoeffs.block<6, 3>(i * 6, 0) += (beta0 * gradPos.transpose() +
                                                          beta1 * gradVel.transpose() +
                                                          beta2 * gradAcc.transpose()) *
                                                        node * step;
            partialGradByTimes(i) += (gradPos.dot(vel) +
                                      gradVel.dot(acc) +
                                      gradAcc.dot(jer)) *
                                     alpha * node * step +
                                     node * integralFrac * tmp_cost;
            cost += node * step * tmp_cost;
        }
    }
    penalty_log(1) = max_pena(POS_IDX);
    penalty_log(2) = max_pena(VEL_IDX);
    penalty_log(3) = max_pena(ACC_IDX);
    penalty_log(4) = max_pena(ATT_IDX);
}

bool MincoOptimizer::setupProblemAndCheck(const std::vector<Eigen::Vector3d>& waypoints,
                              const Eigen::Matrix3d& start_state,
                              const Eigen::Matrix3d& end_state) 
{   
    int N = waypoints.size() - 1; // 多项式段数
    if (N <= 0) {
        return false;
    }

    // 初始化优化变量上下文
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

    // 初始化优化变量
    opt_vars_.dim_t = N;
    opt_vars_.dim_p = 3 * (N - 1);

    // 初始梯度缓存
    opt_vars_.gradByPoints.resize(3, N - 1);
    opt_vars_.gradByTimes.resize(N);
    opt_vars_.partialGradByCoeffs.resize(6 * N, 3);
    opt_vars_.partialGradByTimes.resize(N);

    DefaultInit();
    // if (opt_vars_.default_init) {
    //     DefaultInit();
    // } else {
    //     // opt_vars_.times *= 0.8;
    //     if (opt_vars_.init_ps.size() == static_cast<size_t>(N - 1)) {
    //         for (int i = 0; i < N - 1; ++i) {
    //             opt_vars_.points.col(i) = opt_vars_.init_ps[i];
    //         }
    //     } else {
    //         // 如果缓存的路点数量不对，强制退化为冷启动
    //         DefaultInit();
    //     }
    // }
    
    opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, N);
    return true;
}

void MincoOptimizer::DefaultInit() 
{
    // 根据距离和最大速度分配初始时间
    const VecDf dis = (opt_vars_.waypoint_attractor.leftCols(opt_vars_.piece_num) -
                           opt_vars_.waypoint_attractor.rightCols(opt_vars_.piece_num)).colwise().norm().transpose();
    double speed = cfg_.max_vel;
    opt_vars_.times = (dis / speed).cwiseMax(0.1); // 防止除零
    opt_vars_.points = opt_vars_.waypoint_attractor.block(0, 1, 3, opt_vars_.piece_num - 1);
}

void MincoOptimizer::setInitPsAndTs(const vec_Vec3f& init_ps, const VecDf& init_ts) 
{
    opt_vars_.default_init = false;
    if (opt_vars_.times.size() != init_ts.size()) {
        return;
    }
    if (static_cast<size_t>(opt_vars_.points.cols()) != init_ps.size()) {
        return;
    }
    
    for (size_t i = 0; i < init_ps.size(); ++i) {
        opt_vars_.times[i] = init_ts[i];
        opt_vars_.points.col(i) = init_ps[i];
    }
}
} // namespace minco_planner