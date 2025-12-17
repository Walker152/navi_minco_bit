#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace minco_planner
{

MincoPlanner::MincoPlanner()
: costmap_(nullptr), tf_(nullptr)
{
}

MincoPlanner::~MincoPlanner()
{
}

void MincoPlanner::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = parent.lock();
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_astar", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_astar", use_astar_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  astar_planner_ = std::make_unique<Astar>(
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    
  // Initialize Minco Optimizer
  traj_opt::Config minco_config;
  // Set config values here if needed
  minco_optimizer_ = std::make_shared<traj_opt::ExpTrajOpt>(minco_config, nullptr); // ros_ptr is null for now
}

void MincoPlanner::cleanup()
{
  astar_planner_.reset();
  minco_optimizer_.reset();
}

void MincoPlanner::activate()
{
}

void MincoPlanner::deactivate()
{
}

nav_msgs::msg::Path MincoPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = global_frame_;

  if (!astar_planner_) {
    return path;
  }

  // Update A* size if costmap changed
  astar_planner_->setSize(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
  astar_planner_->setCostmap(costmap_->getCharMap());

  // Convert start/goal to map coordinates
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    return path;
  }

  int start_idx[2] = {(int)mx_start, (int)my_start};
  int goal_idx[2] = {(int)mx_goal, (int)my_goal};

  astar_planner_->setStart(start_idx);
  astar_planner_->setGoal(goal_idx);

  if (astar_planner_->calcPath(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY())) {
    // Get A* path
    float * path_x = astar_planner_->getPathX();
    float * path_y = astar_planner_->getPathY();
    int len = astar_planner_->getPathLen();

    // Convert to world coordinates and fill path
    for (int i = len - 1; i >= 0; i--) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      double wx, wy;
      costmap_->mapToWorld(path_x[i], path_y[i], wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    
    // TODO: Call Minco Optimizer here
    // minco_optimizer_->optimize(...)
    // For now, just return A* path
  }

  return path;
}

}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
