#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "minco_core/astar.hpp"
#include "traj_opt/exp_traj_optimizer_s4.h"

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
    nav_msgs::msg::Path & plan);
  
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  void mapToWorld(double mx, double my, double & wx, double & wy);
  void clearRobotCell(unsigned int wx, unsigned int wy);
private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  
  // A* Planner
  std::unique_ptr<Astar> astar_planner_;
  
  // Minco Optimizer
  std::shared_ptr<traj_opt::ExpTrajOpt> minco_optimizer_;
  
  // Parameters
  double tolerance_;
  bool use_astar_;
  bool allow_unknown_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
