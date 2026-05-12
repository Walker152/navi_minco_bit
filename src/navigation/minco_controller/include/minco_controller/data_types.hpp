#pragma once

#include <Eigen/Core>

namespace minco_controller {

// 6-DOF state in map frame: [px, py, yaw, vx_global, vy_global, omega]^T
struct State
{
  double px{0.0};
  double py{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};

  Eigen::Matrix<double, 6, 1> asVector() const
  {
    Eigen::Matrix<double, 6, 1> v;
    v << px, py, yaw, vx, vy, omega;
    return v;
  }

  static State fromVector(const Eigen::Matrix<double, 6, 1>& v)
  {
    return {v(0), v(1), v(2), v(3), v(4), v(5)};
  }
};

// 3-DOF control in map frame: [F_gx, F_gy, M_gz]^T
struct Control
{
  double F_gx{0.0};
  double F_gy{0.0};
  double M_gz{0.0};

  Eigen::Vector3d asVector() const
  {
    return Eigen::Vector3d(F_gx, F_gy, M_gz);
  }
};

// Reference point from minco trajectory with feedforward acceleration
struct ReferencePoint
{
  Eigen::Vector2d pos{0.0, 0.0};   // map-frame position
  Eigen::Vector2d vel{0.0, 0.0};   // map-frame velocity (feedforward)
  Eigen::Vector2d acc{0.0, 0.0};   // map-frame acceleration (feedforward)
  double yaw{0.0};                  // reference heading
  double yaw_rate{0.0};             // reference angular velocity (feedforward)
  double yaw_acc{0.0};              // reference angular acceleration (feedforward)
};

// Attitude in body frame (roll, pitch) + global yaw for gravity projection
struct Attitude
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

// Physical and solver configuration
struct ModelConfig
{
  // Time
  double dt{0.05};
  double planner_freq{20.0};
  int horizon{10};

  // Physical parameters
  double mass{20.0};
  double inertia_z{1.0};
  double g{9.81};

  // Friction coefficients
  double mu_c{0.1};    // Coulomb friction
  double C_v{0.5};     // Viscous friction

  // Force/torque limits
  double f_max{50.0};            // max force per motor (N)
  double chassis_radius{0.25};   // chassis radius for Mz coupling (m)
  double M_max{5.0};             // max yaw torque (N·m)
  double P_limit{100.0};         // max translational power (W)

  // Cost weights: Q (6D state), R (3D control)
  Eigen::Matrix<double, 6, 1> Q{{10.0, 10.0, 2.0, 1.0, 1.0, 1.0}};
  // P1: Cross-track decomposition — rotated to align with reference heading.
  // Higher cross-track weight enforces precision on curves and narrow passages.
  double q_along_p{3.0};   // position along-track
  double q_cross_p{12.0};  // position cross-track (4x → precision)
  double q_along_v{1.0};   // velocity along-track
  double q_cross_v{3.0};   // velocity cross-track (3x)
  Eigen::Matrix<double, 3, 1> R{{1.0, 1.0, 0.5}};

  // qpOASES solver parameters
  int max_wsr{200};
  double eps_reg{1e-9};
};

// QP problem container: ModelBuilder output, MpcSolver input
struct QPProblem
{
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::MatrixXd A_con;
  Eigen::VectorXd lbA;
  Eigen::VectorXd ubA;
};

}  // namespace minco_controller
