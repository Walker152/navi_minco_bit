#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "ros_interfaces/msg/mpc_position_command.hpp"

#include "log.hpp"
#include "minco_controller/data_types.hpp"
#include "minco_controller/model_builder.hpp"
#include "minco_controller/mpc_solver.hpp"

namespace minco_controller {

class MincoMpcController : public nav2_core::Controller
{
public:
  MincoMpcController() = default;
  ~MincoMpcController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;

  void setPlan(const nav_msgs::msg::Path& path) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
  // Parameter callback for runtime hot-reload
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter>& parameters);

  // Callbacks
  void onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // State fusion: extract attitude (roll, pitch, yaw) from odometry
  void extractAttitudeFromOdom(const nav_msgs::msg::Odometry::SharedPtr& odom,
                               Attitude& attitude) const;

  // Reference trajectory building from planner output
  bool buildReferenceFromOptPath(const State& curr,
                                 std::vector<ReferencePoint>& out_ref) const;

  // Coordinate transform for planner path
  bool transformPathToOdom(const ros_interfaces::msg::MpcPositionCommand::SharedPtr& opt,
                           std::vector<ros_interfaces::msg::PositionCommand>& out_cmds) const;

  // Interpolation utilities
  static double normalizeAngle(double angle);
  static double interpolateYaw(double yaw1, double yaw2, double alpha);
  static double interpolate(double v1, double v2, double alpha);
  static Eigen::Vector2d interpolate(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, double alpha);

  // Visualization
  void publishVisualization(const State& curr_state);

  // Sync model_builder_ with model_config_ after parameter change
  void syncModelConfig();

  // === ROS 2 Interfaces ===
  rclcpp::Subscription<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predict_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_real_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_mpc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_force_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // === TF & Costmap & Frames ===
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoMpcController")};
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // === State Variables & Caches ===
  std::vector<geometry_msgs::msg::PoseStamped> real_path_history_;
  rclcpp::Time last_real_path_pub_time_;

  mutable std::mutex data_mtx_;
  ros_interfaces::msg::MpcPositionCommand::SharedPtr latest_opt_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  mutable double tracked_ref_idx_{0.0};
  mutable rclcpp::Time tracked_ref_time_;
  mutable uint32_t tracked_opt_traj_id_{0};
  mutable bool has_tracked_ref_{false};

  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mtx_;

  // === Core Modules ===
  ModelConfig model_config_;
  std::unique_ptr<ModelBuilder> model_builder_;
  std::unique_ptr<MpcSolver> solver_;

  // === Controller Parameters ===
  double speed_limit_{0.0};
  bool speed_limit_percentage_{false};

  double lookahead_time_{0.5};
  double V_max_{5.0};
  double fixed_wz_{0.0};
  double deadzone_speed_threshold_{0.02};
  double control_delay_compensation_{0.25};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  double lidar_roll_offset_{0.0};
  bool use_small_gyro_mode_{true};
};

}  // namespace minco_controller
