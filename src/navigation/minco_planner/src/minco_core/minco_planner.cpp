#include "minco_core/minco_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>
#include <stdexcept>

namespace minco_planner
{

MincoPlanner::MincoPlanner()
: tf_(nullptr), costmap_(nullptr)
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

  if (!astar_planner_ || !costmap_) {
    throw std::runtime_error("MincoPlanner: planner is not properly configured");
  }

  // 将起点和终点转换到地图坐标系
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw std::runtime_error(
      "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
      std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw std::runtime_error(
      "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (tolerance_ == 0.0 &&
    costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    throw std::runtime_error(
      "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  // 特殊情况：起点和终点重合，直接返回单点路径
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = start.pose;
    path.poses.push_back(pose);
    return path;
  }

  if (!makePlan(start.pose, goal.pose, path)) {
    throw std::runtime_error(
            "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
  }
  return path;
}
bool MincoPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start, my_start;
  worldToMap(wx, wy, mx_start, my_start);
  clearRobotCell(mx_start, my_start);

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal, my_goal;
  worldToMap(wx, wy, mx_goal, my_goal);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  unsigned int nx = costmap_->getSizeInCellsX();
  unsigned int ny = costmap_->getSizeInCellsY();
  astar_planner_->setSize(nx, ny);

  // 1. 设置起点终点 (必须在 setupNavFn 之前，因为 setupNavFn 可能依赖 goal)
  astar_planner_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
  astar_planner_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));

  // 2. 初始化/重置 (重置 potarr, pending 等)
  astar_planner_->setupNavFn(true);

  // 3. 设置代价地图
  astar_planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
  lock.unlock();

  // 4. 传播波前
  // 循环调用 propNavFnAstar 直到找到 Start 或队列为空
  // 给定一个足够大的总循环次数上限，防止死循环
  int max_total_cycles = nx * ny; 
  int cycles_per_step = std::max(nx * ny / 20, nx + ny);
  
  while (max_total_cycles > 0) {
    if (!astar_planner_->propNavFnAstar(cycles_per_step)) {
      break;
    }
    max_total_cycles -= cycles_per_step;
  }

  // 5. 提取路径
  if (!astar_planner_->calcPath(nx * ny / 2) || astar_planner_->getPathLen() < 2) {
    // 如果路径长度小于 2，说明寻路失败（可能只找到了 Start 自己）
    throw std::runtime_error("Failed to create A* path (len < 2)");
  }

  // 取出 A* 路径（地图坐标），转换到世界坐标并构造导航路径
  float * path_x = astar_planner_->getPathX();
  float * path_y = astar_planner_->getPathY();
  const int len = astar_planner_->getPathLen();

  std::vector<Eigen::Vector3d> guide_path;
  guide_path.reserve(len);

  for (int i = 0; i < len; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;

    double wx, wy;
    costmap_->mapToWorld(path_x[i], path_y[i], wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    plan.poses.push_back(pose);

    // 为 Minco 构造引导路径（世界坐标系，z 置 0）
    guide_path.emplace_back(wx, wy, 0.0);
  }

  // 使用 Minco 对路径进行时间分配 / 平滑
  if (minco_optimizer_ && guide_path.size() >= 2) {
    Eigen::Vector3d start_pos(start.position.x, start.position.y, 0.0);
    Eigen::Vector3d goal_pos(goal.position.x, goal.position.y, 0.0);
    Eigen::Vector3d zero_vel(0.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> optimized;
    if (minco_optimizer_->optimize(start_pos, zero_vel, goal_pos, zero_vel,
        guide_path, optimized))
    {
      nav_msgs::msg::Path opt_path;
      opt_path.header = plan.header;

      for (const auto & p : optimized) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = opt_path.header;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        opt_path.poses.push_back(pose);
      }

      if (!opt_path.poses.empty()) {
        plan = opt_path;
        return true;
      }
    }
  }

  // Minco 失败或未启用时，退回原始 A* 路径
  return !plan.poses.empty();
}

bool MincoPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }
  return false;
}

void MincoPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void MincoPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}
}  // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
