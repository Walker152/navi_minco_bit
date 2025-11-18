
#include "dreamchaser_mpc_ctrl.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2/utils.h>
namespace dreamchaser_mpc_ctrl
{

  /**
   * @brief MPC控制器计算速度命令的核心实现
   */
  geometry_msgs::msg::TwistStamped MPController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                         const geometry_msgs::msg::Twist& velocity,
                                                                         nav2_core::GoalChecker* /*goal_checker*/)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 步骤1：状态向量构造
    current_state_ = rosToStateVector(pose, velocity);

    // 步骤2：生成参考轨迹
    Eigen::MatrixXd reference_trajectory = generateReferenceTrajectory(pose, velocity);

    // 步骤3：MPC求解
    Eigen::VectorXd optimal_control_sequence;
    bool solve_success = mpc_solver_->solve(current_state_, reference_trajectory, optimal_control_sequence);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;

    if(solve_success && optimal_control_sequence.size() >= 3)
    {
      // 步骤4：提取第一个控制量（MPC的递归可行性原理）
      Eigen::VectorXd first_control = optimal_control_sequence.head(3);

      // 步骤5：转换为速度增量并积分
      // 注意：这里简化为直接使用加速度作为速度增量
      double dt = mpc_params_.prediction_dt;
      cmd_vel.twist.linear.x = velocity.linear.x + first_control(0) * dt;
      cmd_vel.twist.linear.y = velocity.linear.y + first_control(1) * dt;
      cmd_vel.twist.angular.z = velocity.angular.z + first_control(2) * dt;

      // 步骤6：应用速度约束
      cmd_vel.twist.linear.x =
          std::clamp(cmd_vel.twist.linear.x, -mpc_params_.max_linear_vel, mpc_params_.max_linear_vel);
      cmd_vel.twist.linear.y =
          std::clamp(cmd_vel.twist.linear.y, -mpc_params_.max_linear_vel, mpc_params_.max_linear_vel);
      cmd_vel.twist.angular.z =
          std::clamp(cmd_vel.twist.angular.z, -mpc_params_.max_angular_vel, mpc_params_.max_angular_vel);
    }
    else
    {
      // MPC求解失败，输出零速度
      RCLCPP_WARN(logger_, "MPC solver failed, outputting zero velocity");
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
    }

    return cmd_vel;
  }

  /**
   * @brief 生成参考轨迹
   * @details 从全局路径中提取未来N+1个点作为参考轨迹
   */
  Eigen::MatrixXd MPController::generateReferenceTrajectory(const geometry_msgs::msg::PoseStamped& current_pose,
                                                            const geometry_msgs::msg::Twist& current_vel)
  {
    int N = mpc_params_.prediction_horizon;
    Eigen::MatrixXd reference(N + 1, 6);  // (N+1) x 6 矩阵

    if(global_plan_.poses.empty())
    {
      // 如果没有路径，生成静止参考
      for(int i = 0; i <= N; ++i)
      {
        reference.row(i) << current_pose.pose.position.x, current_pose.pose.position.y,
            tf2::getYaw(current_pose.pose.orientation), 0.0, 0.0, 0.0;  // 目标速度为0
      }
      return reference;
    }

    // 简化实现：沿路径等间距采样
    double dt = mpc_params_.prediction_dt;
    double sampling_distance = std::hypot(current_vel.linear.x, current_vel.linear.y) * dt;

    if(sampling_distance < 0.1)
    {
      sampling_distance = 0.1;  // 最小采样距离
    }

    // 找到最近的路径点
    int closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for(size_t i = 0; i < global_plan_.poses.size(); ++i)
    {
      double dist = std::hypot(global_plan_.poses[i].pose.position.x - current_pose.pose.position.x,
                               global_plan_.poses[i].pose.position.y - current_pose.pose.position.y);
      if(dist < min_dist)
      {
        min_dist = dist;
        closest_idx = i;
      }
    }

    // 沿路径采样参考点
    for(int i = 0; i <= N; ++i)
    {
      int path_idx = std::min(closest_idx + static_cast<int>(i * sampling_distance / 0.05),  // 假设路径分辨率为5cm
                              static_cast<int>(global_plan_.poses.size() - 1));

      auto& ref_pose = global_plan_.poses[path_idx];
      reference.row(i) << ref_pose.pose.position.x, ref_pose.pose.position.y, tf2::getYaw(ref_pose.pose.orientation),
          0.5,  // 期望线性速度x
          0.0,  // 期望线性速度y
          0.0;  // 期望角速度
    }

    return reference;
  }

  /**
   * @brief ROS消息转状态向量
   */
  Eigen::VectorXd MPController::rosToStateVector(const geometry_msgs::msg::PoseStamped& pose,
                                                 const geometry_msgs::msg::Twist& velocity)
  {
    Eigen::VectorXd state(6);
    state << pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation), velocity.linear.x,
        velocity.linear.y, velocity.angular.z;
    return state;
  }

  /**
   * @brief 控制向量转ROS消息
   */
  geometry_msgs::msg::Twist MPController::controlToRosTwist(const Eigen::VectorXd& control)
  {
    geometry_msgs::msg::Twist twist;
    if(control.size() >= 3)
    {
      twist.linear.x = control(0);   // ax -> vx (需要积分)
      twist.linear.y = control(1);   // ay -> vy
      twist.angular.z = control(2);  // α -> ω
    }
    return twist;
  }

  /**
   * @brief 配置函数示例
   */
  void MPController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                               std::string name,
                               std::shared_ptr<tf2_ros::Buffer> tf,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    auto node = parent.lock();
    if(!node)
    {
      throw std::runtime_error("Unable to lock node!");
    }

    node_ = parent;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    plugin_name_ = name;

    // 初始化MPC参数（从ROS参数服务器读取）
    mpc_params_.prediction_horizon = 10;
    mpc_params_.control_frequency = 20.0;
    mpc_params_.prediction_dt = 1.0 / mpc_params_.control_frequency;

    // 权重参数
    mpc_params_.state_weights.resize(6);
    mpc_params_.state_weights << 10.0, 10.0, 1.0, 1.0, 1.0, 0.1;  // [px,py,θ,vx,vy,ω]

    mpc_params_.control_weights.resize(3);
    mpc_params_.control_weights << 1.0, 1.0, 0.1;  // [ax,ay,α]

    mpc_params_.terminal_weights.resize(6);
    mpc_params_.terminal_weights << 20.0, 20.0, 2.0, 0.0, 0.0, 0.0;

    // 约束参数
    mpc_params_.max_linear_vel = 2.0;
    mpc_params_.max_angular_vel = 2.0;
    mpc_params_.max_linear_acc = 3.0;
    mpc_params_.max_angular_acc = 3.0;

    // 创建MPC求解器
    mpc_solver_ = std::make_unique<MPCSolver>(mpc_params_);

    // 创建发布者
    local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

    RCLCPP_INFO(logger_, "MPC Controller configured successfully");
  }

}  // namespace dreamchaser_mpc_ctrl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dreamchaser_mpc_ctrl::MPController, nav2_core::Controller)
/**
 * 使用示例和调试技巧：
 *
 * 1. 参数调整建议：
 *    - 增加Q矩阵的位置权重可以提高跟踪精度
 *    - 增加R矩阵的控制权重可以让控制更平滑
 *    - 预测范围N通常取10-20之间
 *
 * 2. 常见问题排查：
 *    - 如果控制器振荡，尝试增加R权重或减少控制频率
 *    - 如果跟踪误差大，检查系统模型是否准确
 *    - 如果求解失败，检查约束是否合理
 *
 * 3. 性能优化：
 *    - 使用专业的QP求解器（如OSQP、qpOASES）
 *    - 预计算系统矩阵和权重矩阵
 *    - 使用热启动加速求解
 *
 * 4. 扩展功能：
 *    - 添加障碍物约束
 *    - 实现自适应权重调整
 *    - 集成状态估计和滤波
 */