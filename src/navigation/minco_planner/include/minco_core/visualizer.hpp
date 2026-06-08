#ifndef MINCO_PLANNER__MINCO_PLANNER_VISUALIZER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_VISUALIZER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class Visualizer
{
public:
  Visualizer() = default;
  ~Visualizer() = default;

  void configure(const nav2_util::LifecycleNode::WeakPtr & parent,
    const std::string & global_frame,
    const small_rog_map::HybridESDFMap::Ptr & esdf_map,
    bool enable_esdf_timer);

  void cleanup();

  void update(const std::vector<Eigen::Vector3d> & control_points,
    const traj_opt::Trajectory & backup_traj,
    const traj_opt::Trajectory & opt_traj,
    double opt_time_seconds,
    const nav_msgs::msg::Path & astar_path);

  void publishRecoveryDebug(const geometry_msgs::msg::PoseStamped & current_pose,
    const Eigen::Vector2d & escape_vel,
    double preview_sec = 0.5);

  void clearRecoveryDebug();

private:
  void visualTimerCallback();
  nav_msgs::msg::Path convertTrajectoryToPath(const traj_opt::Trajectory & traj,
    const std_msgs::msg::Header & header,
    int steps,
    double t_step) const;

  void publishEsdfCloud(const std_msgs::msg::Header & header);
  void publishGlobalEsdfCloud(const std_msgs::msg::Header & header);

  nav2_util::LifecycleNode::WeakPtr node_;
  std::string global_frame_;

  // ESDF
  small_rog_map::HybridESDFMap::Ptr esdf_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_esdf_cloud_pub_;
  rclcpp::TimerBase::SharedPtr esdf_timer_;

  // Visualization publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr backup_path_vis_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_vis_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr astar_path_vis_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recover_path_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr recover_goal_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_vis_pub_;

  rclcpp::TimerBase::SharedPtr visual_timer_;

  // Cache
  std::mutex vis_mutex_;
  traj_opt::Trajectory vis_opt_traj_;
  traj_opt::Trajectory vis_backup_traj_;
  std::vector<Eigen::Vector3d> vis_control_points_;
  nav_msgs::msg::Path vis_astar_path_;
  double vis_opt_time_ = -1.0;
  bool has_vis_opt_traj_ = false;
  bool has_vis_backup_traj_ = false;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_VISUALIZER_HPP_
