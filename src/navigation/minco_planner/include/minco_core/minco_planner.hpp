#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include "minco_core/header.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "ros_interfaces/msg/position_command.hpp"

#include "minco_core/astar.hpp"
#include "minco_core/corridor_generator.hpp"
#include "minco_core/recovery_behaivor.hpp"
#include "minco_core/planner_profiler.hpp"
#include "smac_search/smac_planner_2d_simple.hpp"
#include "small_rog_map/hybrid_esdf_map.hpp"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "traj_opt/minco_optimizer.hpp"
#include "traj_opt/yaw_traj_opt.h"

#include "utils/header/color_text.hpp"
namespace minco_planner {

class Visualizer;
class MincoFsm;

class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  MincoPlanner();
  ~MincoPlanner();

  void configure(const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  // === Core Planning Interfaces ===
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

  bool PlanGlobalPath(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

  bool ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose);
  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  // === Callbacks ===
  void safetyTimerCallback();

  // === Utility & Helper Functions ===
  bool checkCollision();
  bool checkCollision(const geometry_utils::Trajectory & traj);

  // Accessors for FSM
  bool isTrajSafe() const { return is_traj_safe_.load(); }
  double nowSeconds() const;
  double getTrajectoryRemainTime() const;
  bool isTrajectoryTimeExpired(double now_s) const;
  double getLookaheadDist() const { return lookahead_dist_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose);
  bool consumePendingGoal(geometry_msgs::msg::PoseStamped & goal_out);
  void cancelGoal();
  Eigen::Vector3d getCurrentSpeed() const;
  double getCurrentYawFromOdom() const;

  // Query ESDF distance at the given position.
  double getEsdfDistance(const Eigen::Vector3d & pos) const;

  void publishEscapeCommand(
    const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel);

  void clearRecoveryDebugVisualization();

  void publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose);

  traj_opt::Trajectory generateBackupTraj(const Eigen::Matrix3d & start_state);
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d & cur_pos);

private:
  // === Internal Types ===
  enum class PlanningState
  {
    COLD_START,     // Full replanning with zero initial velocity/acceleration.
    HOT_START,      // Replanning with inherited velocity/acceleration.
    EMERGENCY_STOP  // Immediate backup braking with safety priority.
  };

  // === Utility & Helper Functions ===
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state,
    const std::vector<Eigen::Vector3d> & sparse_path);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state);

  void PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
    const Eigen::Matrix3d & start_state,
    bool stop_at_local_end,
    PlanningState state,
    bool has_shifted_seed,
    const vec_Vec3f & shifted_waypoints,
    const VecDf & shifted_durations,
    vec_Vec3f & init_ps,
    VecDf & init_ts,
    VecDf & local_vmaxs) const;

  bool validateTrajectory(const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos);

  bool optimizeYaw(const Eigen::Matrix3d & start_state,
    const traj_opt::Trajectory & pos_traj,
    traj_opt::Trajectory & out_yaw_traj,
    PlanningState state,
    const geometry_msgs::msg::Pose & current_pose,
    double goal_yaw);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr backup_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // === TF & Costmap & Frames ===
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;

  // === Configurations & Parameters ===
  double tolerance_;
  bool allow_unknown_;
  bool use_smac_;
  bool use_yaw_opt_{true};
  double opt_freq_;
  double lookahead_dist_;
  double traj_goal_tolerance_{0.5};
  std::string esdf_pcd_path_;
  double esdf_resolution_;
  bool publish_esdf_{true};
  MincoOptimizer::Config minco_config;
  RecoverServer::Config recovery_server_config_{};

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  std::unique_ptr<Astar> astar_planner_;
  std::unique_ptr<minco_planner::smac::SmacPlanner2DSimple> smac_planner_;
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  std::unique_ptr<traj_opt::BackupTrajOpt> backup_opt_;
  std::unique_ptr<traj_opt::YawTrajOpt> yaw_opt_;
  std::unique_ptr<MincoFsm> fsm_;
  small_rog_map::HybridESDFMap::Ptr esdf_map_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  RecoverServer::Ptr recovery_server_;
  Ptr planner_handle_;

  // === State Variables & Caches ===
  uint32_t opt_trajectory_id_{0};
  uint32_t backup_trajectory_id_{0};
  geometry_utils::Trajectory last_traj_;
  geometry_utils::Trajectory last_yaw_traj_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PoseStamped pending_goal_;

  bool has_last_traj_ = false;
  bool has_last_yaw_traj_ = false;
  bool has_pending_goal_{false};
  bool has_latest_odom_{false};
  std::atomic_bool is_traj_safe_{true};

  std::mutex path_mutex_;
  std::mutex goal_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex mutex_;

  std::unique_ptr<Visualizer> visualizer_;
  std::unique_ptr<PlannerProfiler> profiler_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
