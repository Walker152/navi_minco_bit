/**
 * @file omni_mpc_controller.hpp
 * @brief 全向轮机器人MPC控制器插件头文件
 * @details 基于模型预测控制(MPC)的全向轮机器人路径跟踪控制器
 * @author Yuchen Fan
 * @date 2025
 */

#ifndef PB_OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_
#define PB_OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_

// C++
#include <memory>
#include <mutex>

// ROS
#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#define EIGEN_DONT_VECTORIZE
// MPC
#include "mpc.hpp"

namespace dreamchaser_mpc_ctrl
{



/**
 * @class OmniMpcController
 * @brief 全向轮机器人MPC控制器主类
 */
class MPController : public nav2_core::Controller
{
public:

  MPController() = default;
  ~MPController() override = default;

  // lifecycle node cfg
  // init 
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  // 核心功能
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief 生成参考轨迹
   * @param current_pose 当前位姿
   * @param current_vel 当前速度
   * @return 参考轨迹矩阵
   */
  Eigen::MatrixXd generateReferenceTrajectory(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Twist& current_vel
  );
  
  /**
   * @brief 将ROS速度消息转换为状态向量
   */
  Eigen::VectorXd rosToStateVector(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity
  );
  
  /**
   * @brief 将控制向量转换为ROS速度消息
   */
  geometry_msgs::msg::Twist controlToRosTwist(
    const Eigen::VectorXd& control
  );
  
  /**
   * @brief 路径坐标变换
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose
  );

private:
  // ROS2相关
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPController")};
  
  // MPC组件
  std::unique_ptr<MPCSolver> mpc_solver_;
  MPCParameters mpc_params_;
  
  // 路径和状态
  nav_msgs::msg::Path global_plan_;
  Eigen::VectorXd current_state_;
  
  // 线程安全
  std::mutex mutex_;
  
  // 发布者（用于调试和可视化）
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr predicted_path_pub_;
};

} // namespace pb_omni_mpc_controller

#endif // PB_OMNI_MPC_CONTROLLER__OMNI_MPC_CONTROLLER_HPP_