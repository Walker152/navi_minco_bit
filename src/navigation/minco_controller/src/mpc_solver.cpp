#include "minco_controller/mpc_solver.hpp"

#include <iostream>
#include <vector>

#include "color_text.hpp"
#include <qpOASES.hpp>

namespace minco_controller {

void MpcSolver::setSolverParams(int max_wsr, double eps_reg)
{
  max_wsr_ = max_wsr;
  eps_reg_ = eps_reg;
}

bool MpcSolver::solve(const QPProblem& qp, Eigen::Vector3d& out_u, int delay_idx)
{
  const int nV = static_cast<int>(qp.g.size());
  const int nC = static_cast<int>(qp.lbA.size());

  if (nV <= 0) {
    return false;
  }

  // Convert Eigen H to qpOASES row-major array
  std::vector<qpOASES::real_t> H_qp(static_cast<size_t>(nV) * static_cast<size_t>(nV), 0.0);
  for (int i = 0; i < nV; ++i) {
    for (int j = 0; j < nV; ++j) {
      H_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
          static_cast<qpOASES::real_t>(qp.H(i, j));
    }
  }

  // Convert vectors
  std::vector<qpOASES::real_t> g_qp(nV);
  std::vector<qpOASES::real_t> lb_qp(nV);
  std::vector<qpOASES::real_t> ub_qp(nV);
  for (int i = 0; i < nV; ++i) {
    g_qp[i] = static_cast<qpOASES::real_t>(qp.g(i));
    lb_qp[i] = static_cast<qpOASES::real_t>(qp.lb(i));
    ub_qp[i] = static_cast<qpOASES::real_t>(qp.ub(i));
  }

  // Convert constraint matrix (if any)
  const qpOASES::real_t* A_ptr = nullptr;
  const qpOASES::real_t* lbA_ptr = nullptr;
  const qpOASES::real_t* ubA_ptr = nullptr;
  std::vector<qpOASES::real_t> A_qp;
  std::vector<qpOASES::real_t> lbA_qp;
  std::vector<qpOASES::real_t> ubA_qp;

  if (nC > 0) {
    A_qp.assign(static_cast<size_t>(nC) * static_cast<size_t>(nV), 0.0);
    for (int i = 0; i < nC; ++i) {
      for (int j = 0; j < nV; ++j) {
        A_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
            static_cast<qpOASES::real_t>(qp.A_con(i, j));
      }
    }

    lbA_qp.assign(nC, 0.0);
    ubA_qp.assign(nC, 0.0);
    for (int i = 0; i < nC; ++i) {
      lbA_qp[i] = static_cast<qpOASES::real_t>(qp.lbA(i));
      ubA_qp[i] = static_cast<qpOASES::real_t>(qp.ubA(i));
    }

    A_ptr = A_qp.data();
    lbA_ptr = lbA_qp.data();
    ubA_ptr = ubA_qp.data();
  }

  // Solve with qpOASES
  qpOASES::QProblem qp_solver(nV, nC);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp_solver.setOptions(options);

  qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(max_wsr_);
  const qpOASES::returnValue ret = qp_solver.init(
      H_qp.data(), g_qp.data(), A_ptr,
      lb_qp.data(), ub_qp.data(), lbA_ptr, ubA_ptr, nWSR);

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

  // Extract full solution
  full_solution_.resize(nV);
  std::vector<qpOASES::real_t> xOpt(nV, 0.0);
  qp_solver.getPrimalSolution(xOpt.data());
  for (int i = 0; i < nV; ++i) {
    full_solution_(i) = static_cast<double>(xOpt[i]);
  }

  // Return delay_idx-th control [F_gx, F_gy, M_gz]
  const int nu = 3;
  const int steps = static_cast<int>(full_solution_.size()) / nu;
  const int idx = std::max(0, std::min(delay_idx, steps - 1));
  out_u = full_solution_.segment<3>(idx * nu);

  return true;
}

}  // namespace minco_controller
