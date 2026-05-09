#pragma once

#include <vector>

#include <Eigen/Core>

#include "minco_controller/data_types.hpp"

namespace minco_controller {

// Pure C++ / Eigen: builds LTI-MPC QP matrices.
// Zero ROS dependencies. All math in map frame.
class ModelBuilder
{
public:
  explicit ModelBuilder(const ModelConfig& config);

  void setConfig(const ModelConfig& config);

  // Precompute constant LTI matrices (A, B, A_hat, B_hat) and Hessian H.
  // Must be called after dt, mass, inertia_z, or horizon change.
  void initLTIMatrices();

  // Lightweight: recompute only H when Q, R weights change.
  void updateCostWeights(const Eigen::Matrix<double, 6, 1>& Q,
                         const Eigen::Matrix<double, 3, 1>& R);

  // Lightweight: update only force limit config fields (no LTI rebuild).
  void updateForceLimits(double f_max, double chassis_radius);

  // Build QP matrices for the current control cycle.
  // x0: 6D state [px, py, yaw, vx, vy, omega] in map frame
  // attitude: current roll/pitch/yaw for gravity projection
  // ref_traj: horizon-length reference (size >= horizon)
  bool buildQP(const Eigen::Matrix<double, 6, 1>& x0,
               const Attitude& attitude,
               const std::vector<ReferencePoint>& ref_traj,
               QPProblem& out_qp) const;

  // Post-solve safety: clip force [Fx,Fy,Mz] to coupled polytope: |F_proj|+|Mz|/(2R) <= 2*f_max.
  static void clampCoupledForce(Eigen::Vector3d& force, double f_max, double chassis_radius);

  // Smooth sign for friction computation (avoids discontinuity at v=0).
  static double smoothSign(double v, double epsilon = 0.01);

  // Getters for precomputed matrices
  const Eigen::Matrix<double, 6, 6>& A() const { return A_; }
  const Eigen::Matrix<double, 6, 3>& B() const { return B_; }

private:
  // Gravity projection: body-frame gravity -> map XY disturbance forces.
  Eigen::Vector3d computeGravityDisturbance(const Attitude& att) const;

  // Friction disturbance: Coulomb + viscous at reference velocity.
  Eigen::Vector3d computeFrictionDisturbance(const Eigen::Vector2d& vel_ref) const;

  // Build stacked disturbance vector [D_0; D_1; ...; D_{N-1}] (3N x 1).
  Eigen::VectorXd buildDisturbanceStack(const Attitude& attitude,
                                         const std::vector<ReferencePoint>& ref_traj) const;

  // Build 17-row-per-step constraint matrix:
  // 16 dynamic coupled polytope rows (|F_proj|+|Mz|/(2R)<=2*f_max) + 1 power row.
  void buildConstraintMatrix(const std::vector<ReferencePoint>& ref_traj,
                             Eigen::MatrixXd& A_con,
                             Eigen::VectorXd& lbA,
                             Eigen::VectorXd& ubA) const;

  // Build variable bounds lb/ub (3N): axis-aligned box.
  void buildVariableBounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub) const;

  ModelConfig config_;

  // Constant LTI matrices (6x6, 6x3)
  Eigen::Matrix<double, 6, 6> A_;
  Eigen::Matrix<double, 6, 3> B_;

  // Stacked prediction matrices: A_hat (nX x nx), B_hat (nX x nU)
  Eigen::MatrixXd A_hat_;
  Eigen::MatrixXd B_hat_;

  // Precomputed constant Hessian (nU x nU)
  Eigen::MatrixXd H_;

  // Derived dimensionality
  int nx_{6};
  int nu_{3};
  int nX_{0};
  int nU_{0};

  bool initialized_{false};
};

}  // namespace minco_controller
