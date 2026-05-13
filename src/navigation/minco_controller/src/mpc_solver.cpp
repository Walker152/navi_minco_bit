#include "minco_controller/mpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "color_text.hpp"
#include <qpOASES.hpp>

namespace minco_controller {

MpcSolver::MpcSolver(const MPCConfig & config) : config_(config)
{
}

void MpcSolver::setConfig(const MPCConfig & config)
{
  config_ = config;
}

void MpcSolver::buildStepModel(
  double /*yaw_ref*/, Eigen::Matrix3d & A, Eigen::Matrix<double, 3, 3> & B) const
{
  A.setIdentity();

  B.setZero();
  // 纯全局坐标系控制，解耦 X, Y, Yaw
  B(0, 0) = config_.dt;
  B(1, 1) = config_.dt;
  B(2, 2) = config_.dt;
}

bool MpcSolver::buildCondensedQP(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Eigen::MatrixXd & H,
  Eigen::VectorXd & g,
  Eigen::VectorXd & lb,
  Eigen::VectorXd & ub,
  Eigen::MatrixXd & A_con,
  Eigen::VectorXd & lbA,
  Eigen::VectorXd & ubA)
{
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  // 构建堆叠模型 X = A_hat x0 + B_hat U
  // 其中nx，nu 分别为单步状态与控制量维度，nX，nU 为堆叠后维度
  const int nx = 3;
  const int nu = 3;
  const int nU = nu * N;
  const int nX = nx * N;

  // 堆叠状态：X = [x1..xN, y1..yN, yaw1..yawN]
  // X = [x1,y1,yaw1, x2,y2,yaw2, ..., xN,yN,yawN]^T
  // X = A_hat * x0 + B_hat * U
  Eigen::MatrixXd A_hat = Eigen::MatrixXd::Zero(nX, nx);
  Eigen::MatrixXd B_hat = Eigen::MatrixXd::Zero(nX, nU);

  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

  for (int i = 0; i < N; ++i) {
    Eigen::Matrix<double, 3, 3> Bi;
    buildStepModel(ref_traj[i].yaw, A, Bi);

    // A_hat block: A^{i+1}
    Eigen::Matrix3d A_power = Eigen::Matrix3d::Identity();
    for (int p = 0; p < i + 1; ++p) {
      A_power = A * A_power;
    }
    A_hat.block<3, 3>(i * nx, 0) = A_power;

    // B_hat: 影响 x_{i+1} 的所有 u_0..u_i
    for (int j = 0; j <= i; ++j) {
      // 由于 A=I，这里仍按通用形式写：A^{i-j} * B_j
      Eigen::Matrix3d A_ij = Eigen::Matrix3d::Identity();
      for (int p = 0; p < i - j; ++p) {
        A_ij = A * A_ij;
      }

      Eigen::Matrix<double, 3, 3> Bj;
      buildStepModel(ref_traj[j].yaw, A, Bj);

      B_hat.block<3, 3>(i * nx, j * nu) = A_ij * Bj;
    }
  }

  // 参考状态堆叠 X_ref
  Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(nX);
  for (int i = 0; i < N; ++i) {
    X_ref(i * nx + 0) = ref_traj[i].pos.x();
    X_ref(i * nx + 1) = ref_traj[i].pos.y();
    X_ref(i * nx + 2) = ref_traj[i].yaw;
  }

  // 参考控制堆叠 U_ref (直接使用全局速度)
  Eigen::VectorXd U_ref = Eigen::VectorXd::Zero(nU);
  for (int i = 0; i < N; ++i) {
    U_ref(i * nu + 0) = ref_traj[i].vel.x();
    U_ref(i * nu + 1) = ref_traj[i].vel.y();
    U_ref(i * nu + 2) = ref_traj[i].yaw_rate;
  }

  // Q_bar / R_bar (对角块)
  // P1: Rotate Q at each horizon step to decompose tracking penalty
  // into along-track (heading direction) and cross-track (perpendicular).
  // Higher cross-track weight enforces precision on curves/narrow passages.
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nX, nX);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nU, nU);
  for (int i = 0; i < N; ++i) {
    const double yaw_ref = ref_traj[i].yaw;
    const double c = std::cos(yaw_ref);
    const double s = std::sin(yaw_ref);
    // Q_xy = R^T * diag(q_along, q_cross) * R  (R is 2D rotation by yaw_ref)
    // R = [c, -s; s, c], so:
    // Q_xy = [c*q_along*c + s*q_cross*s,   c*q_along*(-s) + s*q_cross*c;
    //         (-s)*q_along*c + c*q_cross*s, (-s)*q_along*(-s) + c*q_cross*c]
    const double qa = config_.q_along;
    const double qc = config_.q_cross;
    Q_bar(i * nx + 0, i * nx + 0) = qa * c * c + qc * s * s;
    Q_bar(i * nx + 0, i * nx + 1) = (qc - qa) * c * s;
    Q_bar(i * nx + 1, i * nx + 0) = (qc - qa) * c * s;
    Q_bar(i * nx + 1, i * nx + 1) = qa * s * s + qc * c * c;
    Q_bar(i * nx + 2, i * nx + 2) = config_.Q.z();  // yaw weight unchanged

    R_bar.block<3, 3>(i * nu, i * nu) = config_.R.asDiagonal();
  }

  // d = A_hat * x0 - X_ref
  Eigen::Vector3d x0(curr.x, curr.y, curr.yaw);
  const Eigen::VectorXd d = A_hat * x0 - X_ref;

  // H = 2*(B^T Q B + R)
  H = 2.0 * (B_hat.transpose() * Q_bar * B_hat + R_bar);

  // 数值正则（避免 Hessian 奇异）
  H += config_.eps_regularization * Eigen::MatrixXd::Identity(nU, nU);

  // g = 2*(B^T Q d - R U_ref)
  g = 2.0 * (B_hat.transpose() * Q_bar * d - R_bar * U_ref);

  // 变量边界（速度约束，map/global 系）
  // Use -1e12 instead of -inf: qpOASES defines INFTY=1e20 internally.
  // IEEE -inf causes NaN in active-set arithmetic.
  lb = Eigen::VectorXd::Constant(nU, -1.0e12);
  ub = Eigen::VectorXd::Constant(nU, 1.0e12);
  for (int i = 0; i < N; ++i) {
    lb(i * nu + 0) = config_.vx_min;
    ub(i * nu + 0) = config_.vx_max;
    lb(i * nu + 1) = config_.vy_min;
    ub(i * nu + 1) = config_.vy_max;
    lb(i * nu + 2) = config_.omega_min;
    ub(i * nu + 2) = config_.omega_max;
  }

  // 加速度约束（可选）：a_min <= (v_k - v_{k-1})/dt <= a_max
  if (!config_.use_acc_constraints) {
    A_con.resize(0, 0);
    lbA.resize(0);
    ubA.resize(0);
    return true;
  }

  const int nC = 3 * N;  // 对每个 k 对每个轴各一个差分约束：v_k - v_{k-1}
  A_con = Eigen::MatrixXd::Zero(nC, nU);
  lbA = Eigen::VectorXd::Zero(nC);
  ubA = Eigen::VectorXd::Zero(nC);

  const double last_vx = last_u_global_.x();
  const double last_vy = last_u_global_.y();
  const double last_w = last_u_global_.z();

  // 约束形式：lbA <= A_con * U <= ubA
  // 这里 A_con * U 直接表示 (v_k - v_{k-1})，再用 bounds 限制到 [a_min*dt, a_max*dt]
  for (int k = 0; k < N; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      const int row = k * 3 + axis;
      const int col_curr = k * 3 + axis;
      A_con(row, col_curr) = 1.0;

      if (k == 0) {
        // v0 - v(-1)
        // v(-1) 使用当前时刻真实全局速度作为已知常数，转移到 bounds：
        // a_min*dt <= v0 - v_curr_global <= a_max*dt
        const double last = (axis == 0 ? last_vx : (axis == 1 ? last_vy : last_w));
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.dt +
          last;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.dt +
          last;
      } else {
        const int col_prev = (k - 1) * 3 + axis;
        A_con(row, col_prev) = -1.0;
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.dt;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.dt;
      }
    }
  }

  return true;
}

bool MpcSolver::solve(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Control & out_u,
  std::vector<State> * out_pred)
{
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::MatrixXd A_con;
  Eigen::VectorXd lbA;
  Eigen::VectorXd ubA;

  if (!buildCondensedQP(curr, ref_traj, H, g, lb, ub, A_con, lbA, ubA)) {
    return false;
  }

  const int nV = static_cast<int>(g.size());
  const int nC = static_cast<int>(lbA.size());

  std::vector<qpOASES::real_t> H_qp(static_cast<size_t>(nV) * static_cast<size_t>(nV), 0.0);
  for (int i = 0; i < nV; ++i) {
    for (int j = 0; j < nV; ++j) {
      H_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
        static_cast<qpOASES::real_t>(H(i, j));
    }
  }

  std::vector<qpOASES::real_t> g_qp(static_cast<size_t>(nV), 0.0);
  std::vector<qpOASES::real_t> lb_qp(static_cast<size_t>(nV), 0.0);
  std::vector<qpOASES::real_t> ub_qp(static_cast<size_t>(nV), 0.0);
  for (int i = 0; i < nV; ++i) {
    g_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(g(i));
    lb_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lb(i));
    ub_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ub(i));
  }

  std::vector<qpOASES::real_t> A_qp;
  std::vector<qpOASES::real_t> lbA_qp;
  std::vector<qpOASES::real_t> ubA_qp;

  const qpOASES::real_t * A_ptr = nullptr;
  const qpOASES::real_t * lbA_ptr = nullptr;
  const qpOASES::real_t * ubA_ptr = nullptr;

  if (nC > 0) {
    A_qp.assign(static_cast<size_t>(nC) * static_cast<size_t>(nV), 0.0);
    for (int i = 0; i < nC; ++i) {
      for (int j = 0; j < nV; ++j) {
        A_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
          static_cast<qpOASES::real_t>(A_con(i, j));
      }
    }

    lbA_qp.assign(static_cast<size_t>(nC), 0.0);
    ubA_qp.assign(static_cast<size_t>(nC), 0.0);
    for (int i = 0; i < nC; ++i) {
      lbA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lbA(i));
      ubA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ubA(i));
    }

    A_ptr = A_qp.data();
    lbA_ptr = lbA_qp.data();
    ubA_ptr = ubA_qp.data();
  }

  qpOASES::QProblem qp(nV, nC);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);

  qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(config_.max_working_set_recalculations);
  const qpOASES::returnValue ret =
    qp.init(H_qp.data(), g_qp.data(), A_ptr, lb_qp.data(), ub_qp.data(), lbA_ptr, ubA_ptr, nWSR);

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      std::cout << color_text::RED << "[MpcSolver] qpOASES Max NWSR Reached!" << color_text::RESET
                << std::endl;
    } else if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
      std::cout << color_text::RED << "[MpcSolver] qpOASES Infeasible!" << color_text::RESET << std::endl;
    } else {
      std::cout << color_text::RED << "[MpcSolver] qpOASES Error Code: " << static_cast<int>(ret)
                << color_text::RESET << std::endl;
    }
    return false;
  }

  std::vector<qpOASES::real_t> xOpt(static_cast<size_t>(nV), 0.0);
  qp.getPrimalSolution(xOpt.data());

  // 如果需要输出预测轨迹
  if (out_pred) {
    out_pred->clear();
    out_pred->reserve(N + 1);
    out_pred->push_back(curr);

    State s = curr;
    Eigen::Matrix3d A;  // Identity inside buildStepModel
    Eigen::Matrix<double, 3, 3> B;

    for (int i = 0; i < N; ++i) {
      const double ux = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 0)]);
      const double uy = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 1)]);
      const double uw = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 2)]);

      // 使用 QP 构建时的线性化点 (reference yaw) 进行推演
      buildStepModel(ref_traj[i].yaw, A, B);

      double dx = B(0, 0) * ux + B(0, 1) * uy + B(0, 2) * uw;
      double dy = B(1, 0) * ux + B(1, 1) * uy + B(1, 2) * uw;
      double dyaw = B(2, 0) * ux + B(2, 1) * uy + B(2, 2) * uw;

      s.x += dx;
      s.y += dy;
      s.yaw += dyaw;
      out_pred->push_back(s);
    }
  }

  // 取第一个控制量 u0 (global)
  const Eigen::Vector3d u0_global(
    static_cast<double>(xOpt[0]), static_cast<double>(xOpt[1]), static_cast<double>(xOpt[2]));

  // 记录上一帧全局速度，用于平滑加速度
  last_u_global_ = u0_global;
  has_last_u_ = true;

  // 直接输出全局速度
  out_u.vx = u0_global.x();
  out_u.vy = u0_global.y();
  out_u.omega = u0_global.z();

  return true;
}

}  // namespace minco_controller
