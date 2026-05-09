#pragma once

#include <Eigen/Core>

#include "minco_controller/data_types.hpp"

namespace minco_controller {

// Lightweight qpOASES wrapper — pure C++ / Eigen, zero ROS dependencies.
// No physics, no model building, no constraint math.
// Sole responsibility: convert Eigen QPProblem to qpOASES arrays and solve.
class MpcSolver
{
public:
  MpcSolver() = default;

  void setSolverParams(int max_wsr, double eps_reg);

  // Solve the QP. Returns delay_idx-th control [F_gx, F_gy, M_gz] in out_u.
  bool solve(const QPProblem& qp, Eigen::Vector3d& out_u, int delay_idx = 0);

  const Eigen::VectorXd& getFullSolution() const { return full_solution_; }

private:
  int max_wsr_{200};
  double eps_reg_{1e-9};
  Eigen::VectorXd full_solution_;
};

}  // namespace minco_controller
