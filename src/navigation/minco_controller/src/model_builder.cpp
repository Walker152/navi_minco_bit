#include "minco_controller/model_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_controller {

ModelBuilder::ModelBuilder(const ModelConfig& config) : config_(config)
{
  initLTIMatrices();
}

void ModelBuilder::setConfig(const ModelConfig& config)
{
  config_ = config;
  initLTIMatrices();
}

void ModelBuilder::initLTIMatrices()
{
  const int N = config_.horizon;
  nX_ = nx_ * N;
  nU_ = nu_ * N;

  const double dt = config_.dt;
  const double m = config_.mass;
  const double Iz = config_.inertia_z;

  // A (6x6): identity + position-velocity kinematic coupling
  A_ = Eigen::Matrix<double, 6, 6>::Identity();
  A_(0, 3) = dt;
  A_(1, 4) = dt;
  A_(2, 5) = dt;

  // B (6x3): force-to-state dynamics
  B_.setZero();
  B_(0, 0) = dt * dt / (2.0 * m);
  B_(1, 1) = dt * dt / (2.0 * m);
  B_(3, 0) = dt / m;
  B_(4, 1) = dt / m;
  B_(5, 2) = dt / Iz;

  // --- Build A_hat (nX x nx) and B_hat (nX x nU) ---
  A_hat_ = Eigen::MatrixXd::Zero(nX_, nx_);
  B_hat_ = Eigen::MatrixXd::Zero(nX_, nU_);

  // Precompute powers of A
  std::vector<Eigen::Matrix<double, 6, 6>> A_powers(N + 1);
  A_powers[0] = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 1; i <= N; ++i) {
    A_powers[i] = A_ * A_powers[i - 1];
  }

  for (int i = 0; i < N; ++i) {
    // A_hat block: A^{i+1}
    A_hat_.block<6, 6>(i * nx_, 0) = A_powers[i + 1];

    // B_hat block row i: [A^{i}*B, A^{i-1}*B, ..., A*B, B, 0...]
    for (int j = 0; j <= i; ++j) {
      B_hat_.block<6, 3>(i * nx_, j * nu_) = A_powers[i - j] * B_;
    }
  }

  // --- Precompute H = 2*(B_hat^T * Q_bar * B_hat + R_bar) + eps * I ---
  updateCostWeights(config_.Q, config_.R);

  initialized_ = true;
}

void ModelBuilder::updateCostWeights(const Eigen::Matrix<double, 6, 1>& Q,
                                     const Eigen::Matrix<double, 3, 1>& R)
{
  config_.Q = Q;
  config_.R = R;

  if (nX_ <= 0 || nU_ <= 0) {
    return;
  }

  // Build block-diagonal Q_bar (6N x 6N) and R_bar (3N x 3N)
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nX_, nX_);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nU_, nU_);

  for (int i = 0; i < config_.horizon; ++i) {
    Q_bar.block<6, 6>(i * nx_, i * nx_) = config_.Q.asDiagonal();
    R_bar.block<3, 3>(i * nu_, i * nu_) = config_.R.asDiagonal();
  }

  H_ = 2.0 * (B_hat_.transpose() * Q_bar * B_hat_ + R_bar);
  H_ += config_.eps_reg * Eigen::MatrixXd::Identity(nU_, nU_);
}

void ModelBuilder::updateForceLimits(double f_max, double chassis_radius)
{
  config_.f_max = f_max;
  config_.chassis_radius = chassis_radius;
}

bool ModelBuilder::buildQP(const Eigen::Matrix<double, 6, 1>& x0,
                           const Attitude& attitude,
                           const std::vector<ReferencePoint>& ref_traj,
                           QPProblem& out_qp) const
{
  const int N = config_.horizon;
  if (!initialized_ || N <= 0 || static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  // --- Build variable bounds ---
  buildVariableBounds(out_qp.lb, out_qp.ub);

  // --- Build constraint matrix ---
  buildConstraintMatrix(ref_traj, out_qp.A_con, out_qp.lbA, out_qp.ubA);

  // --- Build disturbance stack ---
  const Eigen::VectorXd D_stack = buildDisturbanceStack(attitude, ref_traj);

  // --- Build X_ref (6N x 1) ---
  Eigen::VectorXd X_ref(nX_);
  for (int i = 0; i < N; ++i) {
    X_ref.segment<6>(i * nx_) << ref_traj[i].pos.x(), ref_traj[i].pos.y(), ref_traj[i].yaw,
                                 ref_traj[i].vel.x(), ref_traj[i].vel.y(), ref_traj[i].yaw_rate;
  }

  // --- Build U_ref (3N x 1): feedforward force = [m*ax, m*ay, Iz*yaw_acc] ---
  Eigen::VectorXd U_ref(nU_);
  const double m = config_.mass;
  const double Iz = config_.inertia_z;
  for (int i = 0; i < N; ++i) {
    U_ref.segment<3>(i * nu_) << m * ref_traj[i].acc.x(), m * ref_traj[i].acc.y(),
                                 Iz * ref_traj[i].yaw_acc;
  }

  // --- Build Q_bar and R_bar for g computation ---
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nX_, nX_);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nU_, nU_);
  for (int i = 0; i < N; ++i) {
    Q_bar.block<6, 6>(i * nx_, i * nx_) = config_.Q.asDiagonal();
    R_bar.block<3, 3>(i * nu_, i * nu_) = config_.R.asDiagonal();
  }

  // --- Compute g = 2*B_hat^T*Q_bar*(A_hat*x0 + B_hat*D_stack - X_ref) - 2*R_bar*U_ref ---
  const Eigen::VectorXd Ax0 = A_hat_ * x0;
  const Eigen::VectorXd BD = B_hat_ * D_stack;
  const Eigen::VectorXd err = Ax0 + BD - X_ref;

  out_qp.g = 2.0 * B_hat_.transpose() * Q_bar * err - 2.0 * R_bar * U_ref;

  // --- Copy precomputed H ---
  out_qp.H = H_;

  return true;
}

// --- Disturbance computation ---

Eigen::Vector3d ModelBuilder::computeGravityDisturbance(const Attitude& att) const
{
  const double cr = std::cos(att.roll);
  const double sr = std::sin(att.roll);
  const double cp = std::cos(att.pitch);
  const double sp = std::sin(att.pitch);
  const double cy = std::cos(att.yaw);
  const double sy = std::sin(att.yaw);

  // Body-frame gravity [0, 0, -g] rotated to map frame via Rz(yaw)*Ry(pitch)*Rx(roll)
  // g_map_x = -g * (cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll))
  // g_map_y = -g * (sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll))
  const double g_x = -config_.g * (cy * sp * cr + sy * sr);
  const double g_y = -config_.g * (sy * sp * cr - cy * sr);

  return Eigen::Vector3d(config_.mass * g_x, config_.mass * g_y, 0.0);
}

Eigen::Vector3d ModelBuilder::computeFrictionDisturbance(const Eigen::Vector2d& vel_ref, const double& omega_ref) const
{
  const double m = config_.mass;
  const double g = config_.g;
  const double R = config_.chassis_radius;
  const double N_contact = m * g;  // normal force

  const double F_fric_x = -(config_.mu_c * N_contact * smoothSign(vel_ref.x()) + config_.C_v * vel_ref.x());
  const double F_fric_y = -(config_.mu_c * N_contact * smoothSign(vel_ref.y()) + config_.C_v * vel_ref.y());
  const double M_fric = -(config_.mu_c * N_contact * R * smoothSign(omega_ref) + 
                          config_.C_v * R * R * omega_ref);
  return Eigen::Vector3d(F_fric_x, F_fric_y, M_fric);
}

Eigen::VectorXd ModelBuilder::buildDisturbanceStack(
    const Attitude& attitude, const std::vector<ReferencePoint>& ref_traj) const
{
  const int N = config_.horizon;
  Eigen::VectorXd D_stack(nU_);

  const Eigen::Vector3d D_grav = computeGravityDisturbance(attitude);

  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d D_fric = computeFrictionDisturbance(ref_traj[i].vel, ref_traj[i].yaw_rate);
    D_stack.segment<3>(i * nu_) = D_grav + D_fric;
  }

  return D_stack;
}

// --- Constraint and bound building ---

void ModelBuilder::buildVariableBounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub) const
{
  const int N = config_.horizon;
  lb = Eigen::VectorXd::Zero(nU_);
  ub = Eigen::VectorXd::Zero(nU_);

  const double F_bound = 2.0 * config_.f_max;

  for (int i = 0; i < N; ++i) {
    lb(i * nu_ + 0) = -F_bound;
    lb(i * nu_ + 1) = -F_bound;
    lb(i * nu_ + 2) = -config_.M_max;

    ub(i * nu_ + 0) = F_bound;
    ub(i * nu_ + 1) = F_bound;
    ub(i * nu_ + 2) = config_.M_max;
  }
}

void ModelBuilder::buildConstraintMatrix(const std::vector<ReferencePoint>& ref_traj,
                                         Eigen::MatrixXd& A_con,
                                         Eigen::VectorXd& lbA,
                                         Eigen::VectorXd& ubA) const
{
  const int N = config_.horizon;
  constexpr int kRowsPerStep = 17;
  const int nC = kRowsPerStep * N;

  A_con = Eigen::MatrixXd::Zero(nC, nU_);

  const double sqrt2_2 = std::sqrt(2.0) / 2.0;
  const double R = config_.chassis_radius;
  const double coupling_coeff = 1.0 / (2.0 * R);
  const double F_bound = 2.0 * config_.f_max;

  // 8 octagon projection directions (normalized)
  const double dirs[8][2] = {
    { 1.0,  0.0},
    { sqrt2_2,  sqrt2_2},
    { 0.0,  1.0},
    {-sqrt2_2,  sqrt2_2},
    {-1.0,  0.0},
    {-sqrt2_2, -sqrt2_2},
    { 0.0, -1.0},
    { sqrt2_2, -sqrt2_2}
  };

  for (int i = 0; i < N; ++i) {
    const int base_row = i * kRowsPerStep;
    const int col_offset = i * nu_;

    // Rows 0-15: dynamic coupled polytope
    // For each direction d: d·F + |Mz|/(2R) <= 2*f_max  →  split into ±Mz
    for (int d = 0; d < 8; ++d) {
      const int r_pos = base_row + 2 * d;      // +Mz variant
      const int r_neg = base_row + 2 * d + 1;  // -Mz variant

      const double dx = dirs[d][0];
      const double dy = dirs[d][1];

      // +Mz: dx*Fx + dy*Fy + (1/(2R))*Mz <= 2*f_max
      A_con(r_pos, col_offset + 0) = dx;
      A_con(r_pos, col_offset + 1) = dy;
      A_con(r_pos, col_offset + 2) = coupling_coeff;

      // -Mz: dx*Fx + dy*Fy - (1/(2R))*Mz <= 2*f_max
      A_con(r_neg, col_offset + 0) = dx;
      A_con(r_neg, col_offset + 1) = dy;
      A_con(r_neg, col_offset + 2) = -coupling_coeff;
    }

    // Row 16: power constraint vx_ref*Fx + vy_ref*Fy <= P_limit
    A_con(base_row + 16, col_offset + 0) = ref_traj[i].vel.x();
    A_con(base_row + 16, col_offset + 1) = ref_traj[i].vel.y();
  }

  // All constraints are one-sided (upper bound only)
  lbA = Eigen::VectorXd::Constant(nC, -std::numeric_limits<double>::infinity());
  ubA = Eigen::VectorXd::Zero(nC);

  for (int i = 0; i < N; ++i) {
    const int base_row = i * kRowsPerStep;
    // Coupled polytope rows: ubA = 2*f_max
    for (int r = 0; r < 16; ++r) {
      ubA(base_row + r) = F_bound;
    }
    // Power row: ubA = P_limit
    ubA(base_row + 16) = config_.P_limit;
  }
}

// --- Post-solve safety clipping ---

/* static */
void ModelBuilder::clampCoupledForce(Eigen::Vector3d& force, double f_max, double chassis_radius)
{
  const double R = chassis_radius;
  const double F_bound = 2.0 * f_max;
  const double coupling_coeff = 1.0 / (2.0 * R);
  const double sqrt2_2 = std::sqrt(2.0) / 2.0;

  const double Mz = force(2);

  // 8 projection directions
  const double dirs[8][2] = {
    { 1.0,  0.0},
    { sqrt2_2,  sqrt2_2},
    { 0.0,  1.0},
    {-sqrt2_2,  sqrt2_2},
    {-1.0,  0.0},
    {-sqrt2_2, -sqrt2_2},
    { 0.0, -1.0},
    { sqrt2_2, -sqrt2_2}
  };

  // Find worst-case violation scale factor
  double scale = 1.0;
  for (int d = 0; d < 8; ++d) {
    const double proj = dirs[d][0] * force(0) + dirs[d][1] * force(1);
    const double val_pos = proj + coupling_coeff * Mz;
    const double val_neg = proj - coupling_coeff * Mz;

    if (val_pos > F_bound && std::abs(proj) > 1e-12) {
      const double max_proj = F_bound - coupling_coeff * std::abs(Mz);
      if (max_proj > 1e-12) {
        scale = std::min(scale, max_proj / std::abs(proj));
      } else {
        scale = 0.0;
      }
    }
  }

  force(0) *= scale;
  force(1) *= scale;
}

/* static */
double ModelBuilder::smoothSign(double v, double epsilon)
{
  return std::tanh(v / epsilon);
}

}  // namespace minco_controller
