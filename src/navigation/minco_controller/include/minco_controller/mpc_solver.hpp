#pragma once

#include <vector>

#include <Eigen/Core>

#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// 纯 C++ MPC 求解器：不依赖 ROS，仅依赖 Eigen 与 qpOASES。
class MpcSolver
{
public:
  explicit MpcSolver(const MPCConfig & config);

  void setConfig(const MPCConfig & config);

  // @brief 求解一次 MPC。
  // @param curr 当前状态 (map)
  // @param ref_traj 参考轨迹 (horizon 个点，若不足由插件层补齐)
  // @param out_u 输出控制 (map)
  // @param out_pred (可选) 输出预测轨迹
  // @return 是否求解成功
  bool solve(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Control & out_u,
    std::vector<State> * out_pred = nullptr);

  Eigen::Vector3d getLastControl() const { return last_u_global_; }

private:
  // 构建离散线性模型：x_{k+1} = A x_k + B_k u_k
  // 状态为 [x,y,yaw]，控制变量 u_k 为 map/global 系 [vx, vy, omega]
  void buildStepModel(double yaw_ref, Eigen::Matrix3d & A, Eigen::Matrix<double, 3, 3> & B) const;

  // 组装 MPC 的 condensed QP：0.5*U^T H U + g^T U
  // 其中 U 堆叠了 horizon 个控制量 [vx, vy, omega] (map/global)
  bool buildCondensedQP(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & g,
    Eigen::VectorXd & lb,
    Eigen::VectorXd & ub,
    Eigen::MatrixXd & A_con,
    Eigen::VectorXd & lbA,
    Eigen::VectorXd & ubA);

private:
  MPCConfig config_;

  // 用于加速度约束的上一时刻 global 控制量
  Eigen::Vector3d last_u_global_{0.0, 0.0, 0.0};
  bool has_last_u_{false};
};

}  // namespace minco_controller
