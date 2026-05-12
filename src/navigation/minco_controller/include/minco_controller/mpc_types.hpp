#pragma once

#include <vector>

#include <Eigen/Core>

namespace minco_controller {

// 状态：全局系 (map) 下的位姿
struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

// 控制量：速度命令 (vx, vy, omega)
// 约定：对外输出为全局系 (map) 下速度；插件层再转换到 base_link。
struct Control
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

// 参考点：来自 Minco 轨迹的高精度前馈信息
struct ReferencePoint
{
  Eigen::Vector2d pos{0.0, 0.0};  // map系位置
  Eigen::Vector2d vel{0.0, 0.0};  // map系速度(前馈)
  double yaw{0.0};                // 参考航向
  double yaw_rate{0.0};           // 参考角速度(前馈)
};

// MPC 配置
struct MPCConfig
{
  // 采样周期与预测步长
  double dt{0.05};
  double planner_freq{20.0};
  double lookahead_time{0.5};

  int horizon{10};

  // 代价权重：Q(状态误差) / R(控制误差)
  // 状态采用 [x, y, yaw]，控制采用 [vx, vy, omega]
  // Q_along/cross are rotated to align with the reference heading at each horizon step.
  Eigen::Vector3d Q{5.0, 5.0, 2.0};
  double q_along{3.0};   // weight along heading (lower → timing flexibility)
  double q_cross{12.0};  // weight cross-track (higher → precision)
  Eigen::Vector3d R{1.0, 1.0, 0.5};

  // 速度约束（作用在 map/global 速度变量上）
  double vx_min{-1.0};
  double vx_max{1.0};
  double vy_min{-1.0};
  double vy_max{1.0};
  double omega_min{-2.0};
  double omega_max{2.0};

  // 加速度约束（可选）
  bool use_acc_constraints{false};
  double ax_min{-2.0};
  double ax_max{2.0};
  double ay_min{-2.0};
  double ay_max{2.0};
  double alpha_min{-4.0};
  double alpha_max{4.0};

  // qpOASES 求解器参数
  int max_working_set_recalculations{200};
  int max_iterations{200};
  double eps_regularization{1e-9};
};

}  // namespace minco_controller
