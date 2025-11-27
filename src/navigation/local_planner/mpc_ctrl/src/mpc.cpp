#include "mpc.hpp"

namespace dreamchaser_mpc_ctrl
{
  MPCSolver::MPCSolver(const MPCParameters& params)
    : params_(params)
  {
    initializeSystemMatrices();
  }

  /**
   * @brief 初始化系统矩阵A和B
   * @details 全向轮机器人的线性离散化模型
   */
  void MPCSolver::initializeSystemMatrices()
  {
    double dt = params_.prediction_dt;

    // 状态维度：6 [px, py, θ, vx, vy, ω]
    // 控制维度：3 [ax, ay, α]
    A_ = Eigen::MatrixXd::Identity(6, 6);
    B_ = Eigen::MatrixXd::Zero(6, 3);

    // 位置积分：p(k+1) = p(k) + v(k)*dt
    A_(0, 3) = dt;  // px = px + vx*dt
    A_(1, 4) = dt;  // py = py + vy*dt
    A_(2, 5) = dt;  // θ = θ + ω*dt

    // 速度积分：v(k+1) = v(k) + a(k)*dt
    B_(3, 0) = dt;  // vx = vx + ax*dt
    B_(4, 1) = dt;  // vy = vy + ay*dt
    B_(5, 2) = dt;  // ω = ω + α*dt
  }

  /**
   * @brief MPC求解函数（简化版本）
   * @details 使用无约束QP的解析解
   */
  bool MPCSolver::solve(const Eigen::VectorXd& current_state,
                        const Eigen::MatrixXd& reference_traj,
                        Eigen::VectorXd& optimal_control)
  {
    int N = params_.prediction_horizon;
    int nx = 6;  // 状态维度
    int nu = 3;  // 控制维度

    // 构造权重矩阵
    Eigen::MatrixXd Q = params_.state_weights.asDiagonal();
    Eigen::MatrixXd R = params_.control_weights.asDiagonal();
    Eigen::MatrixXd Qf = params_.terminal_weights.asDiagonal();

    // 使用简化的解析解方法（无约束情况）
    return solveQPAnalytical(current_state, reference_traj, optimal_control);
  }

  /**
   * @brief 无约束QP的解析解求解
   * @details 对于无约束的线性MPC，存在闭式解
   */
  bool MPCSolver::solveQPAnalytical(const Eigen::VectorXd& x0, const Eigen::MatrixXd& ref, Eigen::VectorXd& u_opt)
  {
    int N = params_.prediction_horizon;
    int nx = A_.rows();
    int nu = B_.cols();

    // 构造预测矩阵
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(nx * (N + 1), nx);
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(nx * (N + 1), nu * N);

    // Phi矩阵：状态预测矩阵
    Phi.block(0, 0, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(nx, nx);

    for(int i = 1; i <= N; ++i)
    {
      A_power = A_ * A_power;
      Phi.block(i * nx, 0, nx, nx) = A_power;
    }

    // Gamma矩阵：控制影响矩阵
    Eigen::MatrixXd A_temp = Eigen::MatrixXd::Identity(nx, nx);
    for(int i = 1; i <= N; ++i)
    {
      for(int j = 0; j < i; ++j)
      {
        if(j == i - 1)
        {
          Gamma.block(i * nx, j * nu, nx, nu) = B_;
        }
        else
        {
          Gamma.block(i * nx, j * nu, nx, nu) = A_temp * B_;
          A_temp = A_ * A_temp;
        }
      }
      A_temp = Eigen::MatrixXd::Identity(nx, nx);
    }

    // 构造权重矩阵
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nx * (N + 1), nx * (N + 1));
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nu * N, nu * N);

    Eigen::MatrixXd Q = params_.state_weights.asDiagonal();
    Eigen::MatrixXd R = params_.control_weights.asDiagonal();
    Eigen::MatrixXd Qf = params_.terminal_weights.asDiagonal();

    for(int i = 0; i < N; ++i)
    {
      Q_bar.block(i * nx, i * nx, nx, nx) = Q;
      R_bar.block(i * nu, i * nu, nu, nu) = R;
    }
    Q_bar.block(N * nx, N * nx, nx, nx) = Qf;  // 终端权重

    // 构造参考轨迹向量
    Eigen::VectorXd ref_vec(nx * (N + 1));
    for(int i = 0; i <= N; ++i)
    {
      if(i < ref.rows())
      {
        ref_vec.segment(i * nx, nx) = ref.row(i).transpose();
      }
      else
      {
        ref_vec.segment(i * nx, nx) = ref.row(ref.rows() - 1).transpose();
      }
    }

    // 解析解：u* = -(R_bar + Gamma^T * Q_bar * Gamma)^(-1) * Gamma^T * Q_bar * (Phi * x0 - ref)
    Eigen::MatrixXd H = Gamma.transpose() * Q_bar * Gamma + R_bar;
    Eigen::VectorXd g = Gamma.transpose() * Q_bar * (Phi * x0 - ref_vec);

    // 求解线性方程组
    u_opt = -H.ldlt().solve(g);

    // 检查求解是否成功
    if(u_opt.hasNaN())
    {
      return false;
    }

    return true;
  }
}  // namespace dreamchaser_mpc_ctrl