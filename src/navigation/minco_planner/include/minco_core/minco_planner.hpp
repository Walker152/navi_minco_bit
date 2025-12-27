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

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros_interfaces/msg/position_command.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"

#include "minco_core/astar.hpp"
#include "minco_core/static_esdf_map.hpp"
#include "traj_opt/minco_optimizer.hpp"
namespace minco_planner
{

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
  
  // 路径抽稀
  std::vector<Eigen::Vector3d> getSparseWaypoints(const std::vector<Eigen::Vector3d>& path); 
  bool isLineFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  void mapToWorld(double mx, double my, double & wx, double & wy);
  void clearRobotCell(unsigned int wx, unsigned int wy);
private:
  void publishOptimizedTrajectory(
    const traj_opt::Trajectory & opt_traj,
    const std_msgs::msg::Header & header,
    int steps,
    double t_step);

  void publishEsdfCloud(const std_msgs::msg::Header & header);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  
  // A* Planner
  std::unique_ptr<Astar> astar_planner_;
  
  // Minco Optimizer
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  
  // Static ESDF Map
  StaticESDFMap::Ptr esdf_map_;
  std::string esdf_pcd_path_;
  double esdf_resolution_;
  // Parameters
  double tolerance_;
  bool use_astar_;
  bool allow_unknown_;
  MincoOptimizer::Config minco_config;
  
  rclcpp::Publisher<ros_interfaces::msg::PositionCommand>::SharedPtr traj_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_cloud_pub_;
  rclcpp::TimerBase::SharedPtr esdf_timer_;
  uint32_t opt_trajectory_id_{0};

  geometry_utils::Trajectory last_traj_;
  bool has_last_traj_ = false;
  
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  std::mutex mutex_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
