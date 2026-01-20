#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "std_msgs/msg/header.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "visualization_msgs/msg/marker.hpp"

#include "ros_interfaces/msg/position_command.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"

#include "minco_core/astar.hpp"
#include "small_rog_map/hybrid_esdf_map.hpp"
#include "minco_core/corridor_generator.hpp"
#include "traj_opt/minco_optimizer.hpp"
#include "traj_opt/backup_traj_optimizer_s4.h"

#include "utils/header/color_text.hpp"
namespace minco_planner
{

class Visualizer;

class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  MincoPlanner();
  ~MincoPlanner();

  void configure(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  traj_opt::Trajectory generateBackupTraj(const Eigen::Matrix3d& start_state);
  
  // 路径抽稀
  std::vector<Eigen::Vector3d> getSparseWaypoints(const std::vector<Eigen::Vector3d>& path); 
  bool isLineFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  void mapToWorld(double mx, double my, double & wx, double & wy);
  void clearRobotCell(unsigned int wx, unsigned int wy);

  void optimizationTimerCallback();
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d& cur_pos);

private:
  enum class PlanningState {
    COLD_START,     // 完全重规划 (Zero V/A)
    HOT_START,      // 继承重规划 (Inherit V/A)
    EMERGENCY_STOP  // 立即触发备份刹停（安全优先）
  };

  // State Machine Logic
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose,
    const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(
    const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose,
    double t_dur,
    Eigen::Matrix3d & start_state);

  void publishOptimizedTrajectory(
    const traj_opt::Trajectory & opt_traj,
    const std_msgs::msg::Header & header,
    int steps,
    double t_step);

  void publishBackupTrajectory(
    const traj_opt::Trajectory & backup_traj,
    const std_msgs::msg::Header & header,
    int steps,
    double t_step);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  // A* Planner
  std::unique_ptr<Astar> astar_planner_;
  
  // Minco Optimizer
  std::unique_ptr<MincoOptimizer> minco_optimizer_;

  // Backup Trajectory Optimizer
  std::unique_ptr<traj_opt::BackupTrajOpt> backup_opt_;
  
  // Static ESDF Map
  small_rog_map::HybridESDFMap::Ptr esdf_map_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  std::string esdf_pcd_path_;
  double esdf_resolution_;
  // Parameters
  double tolerance_;
  bool use_astar_;
  bool allow_unknown_;
  double opt_freq_;
  double lookahead_dist_;

  MincoOptimizer::Config minco_config;
  
  rclcpp::TimerBase::SharedPtr opt_timer_;
  
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  std::mutex path_mutex_;

  // Command Publishers
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr backup_path_pub_;
  uint32_t opt_trajectory_id_{0};
  uint32_t backup_trajectory_id_{0};

  geometry_utils::Trajectory last_traj_;
  bool has_last_traj_ = false;

  // Visualization helper (includes vis publishers + timers + ESDF timer)
    std::unique_ptr<Visualizer> visualizer_;
  
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  std::mutex mutex_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
