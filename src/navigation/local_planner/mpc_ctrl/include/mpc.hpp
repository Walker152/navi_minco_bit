#pragma once
#define EIGEN_DONT_VECTORIZE
#include <Eigen/Dense>
#include <memory>
//
namespace dreamchaser_mpc_ctrl
{
  // MPC参数结构体
  struct MPCParameters
  {
    int prediction_horizon;    ///< 预测范围N
    double control_frequency;  ///< 控制频率
    double prediction_dt;      ///< 预测时间步长

    // 权重矩阵
    Eigen::VectorXd state_weights;     ///< 状态权重 [px,py,θ,vx,vy,ω]
    Eigen::VectorXd control_weights;   ///< 控制权重 [ax,ay,α]
    Eigen::VectorXd terminal_weights;  ///< 终端权重

    // 约束参数
    double max_linear_vel;   ///< 最大线性速度
    double max_angular_vel;  ///< 最大角速度
    double max_linear_acc;   ///< 最大线性加速度
    double max_angular_acc;  ///< 最大角加速度
  };

  // MPC求解器实现
  class MPCSolver
  {
  public:
    /**
     * @brief 构造函数
     * @param params MPC参数
     */
    explicit MPCSolver(const MPCParameters& params);

    /**
     * @brief 求解MPC优化问题
     * @param current_state 当前状态 [px,py,θ,vx,vy,ω]
     * @param reference_traj 参考轨迹 (N+1) x 6
     * @param optimal_control 输出的最优控制序列
     * @return 求解是否成功
     */
    bool solve(const Eigen::VectorXd& current_state,
               const Eigen::MatrixXd& reference_traj,
               Eigen::VectorXd& optimal_control);

  private:
    MPCParameters params_;
    Eigen::MatrixXd A_;  ///< 系统矩阵
    Eigen::MatrixXd B_;  ///< 控制矩阵

    /**
     * @brief 初始化系统矩阵
     */
    void initializeSystemMatrices();

    /**
     * @brief 简化的QP求解（解析解方法）
     */
    bool solveQPAnalytical(const Eigen::VectorXd& x0, const Eigen::MatrixXd& ref, Eigen::VectorXd& u_opt);
  };

}  // namespace dreamchaser_mpc_ctrl