#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <cstdint>
#include <vector>

#include "nav2_core/controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "ros_interfaces/msg/mpc_position_command.hpp"

#include "minco_controller/mpc_solver.hpp"
#include "log.hpp"

namespace minco_controller
{

class MincoMpcController : public nav2_core::Controller
{
public:
  MincoMpcController() = default;
  ~MincoMpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 从缓存轨迹里找最近点并提取 horizon 参考序列
  bool buildReferenceFromOptPath(
    const State & curr,
    std::vector<ReferencePoint> & out_ref) const;

  bool transformPathToOdom(
    const ros_interfaces::msg::MpcPositionCommand::SharedPtr & opt,
    std::vector<ros_interfaces::msg::PositionCommand> & out_cmds) const;

  static double normalizeYaw(double yaw);
  inline static double interpolateYaw(double yaw1, double yaw2, double alpha) {
    double diff = std::atan2(std::sin(yaw2 - yaw1), std::cos(yaw2 - yaw1));
    return yaw1 + diff * alpha;
  }

  inline static double interpolate(double v1, double v2, double alpha) {
    return v1 + (v2 - v1) * alpha;
  }

  inline static Eigen::Vector2d interpolate(
    const Eigen::Vector2d & v1,
    const Eigen::Vector2d & v2,
    double alpha)
  {
    return v1 + (v2 - v1) * alpha;
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoMpcController")};
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // 订阅：Minco 优化轨迹 / 里程计
  rclcpp::Subscription<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 发布：可视化路径
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predict_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_real_path_pub_;
  // 记录实际行走路径
  std::vector<geometry_msgs::msg::PoseStamped> real_path_history_;
  rclcpp::Time last_real_path_pub_time_;
  
  // 用于可视化发布封装
  void publishVisualization(const std::vector<State> & pred_path, const State & curr_state);

  // 缓存：最新轨迹/里程计
  mutable std::mutex data_mtx_;
  ros_interfaces::msg::MpcPositionCommand::SharedPtr latest_opt_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  // 轨迹参考索引跟踪（用于轨迹未更新时按时间前推，避免回追起点）
  mutable double tracked_ref_idx_{0.0};
  mutable rclcpp::Time tracked_ref_time_;
  mutable uint32_t tracked_opt_traj_id_{0};
  mutable bool has_tracked_ref_{false};

  // Nav2 setPlan 缓存（fallback）
  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mtx_;

  // MPC
  MPCConfig mpc_config_;
  std::unique_ptr<MpcSolver> solver_;

  // 动态限速（Nav2 setSpeedLimit）
  double speed_limit_{0.0};
  bool speed_limit_percentage_{false};

  double fixed_wz_{0.0};
};

}  // namespace minco_controller
