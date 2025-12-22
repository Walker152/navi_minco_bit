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
  logger_ = node->get_logger();
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_astar", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".use_astar", use_astar_);
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.max_velocity", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".minco_optimizer.max_velocity", minco_config.max_vel);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.max_acceleration", rclcpp::ParameterValue(4.0));
  node->get_parameter(name + ".minco_optimizer.max_acceleration", minco_config.max_acc);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minco_optimizer.time_allocation_iters", rclcpp::ParameterValue(15));
  node->get_parameter(name + ".minco_optimizer.time_allocation_iters", minco_config.time_allocation_iters);

  astar_planner_ = std::make_unique<Astar>(
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    
  // Initialize Minco Optimizer
  
  // Set config values here if needed
  minco_optimizer_ = std::make_unique<minco_planner::MincoOptimizer>(minco_config); 
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

  // In ROS 2 Humble, the GlobalPlanner interface does not provide a cancel_checker.
  // We use rclcpp::ok() as a basic check.
  auto cancel_checker = []() {
    return !rclcpp::ok();
  };

  if (!astar_planner_ || !costmap_) {
    RCLCPP_ERROR(logger_, "MincoPlanner: planner is not properly configured");
    throw std::runtime_error("MincoPlanner: planner is not properly configured");
  }

  if (!minco_optimizer_) {
    RCLCPP_ERROR(logger_, "MincoPlanner: minco_optimizer is not initialized");
    throw std::runtime_error("MincoPlanner: minco_optimizer is not initialized");
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

  if (!makePlan(start.pose, goal.pose, tolerance_, cancel_checker, path)) {
    throw std::runtime_error(
            "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
  }
  return path;
}

bool MincoPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  (void)tolerance;
  // 1. Prepare the plan
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = global_frame_;

  double plan_start_time = rclcpp::Clock().now().seconds() + 0.03;
  Eigen::Matrix3d start_state, end_state;
  start_state.setZero();
  end_state.setZero();

  bool hot_start = false;
  if (has_last_traj_)
  {
    double t_dur = plan_start_time - last_traj_.start_WT;
    if (t_dur > 0.0 && t_dur < last_traj_.getTotalDuration())
    {
      hot_start = true;
      start_state.col(0) = last_traj_.getPos(t_dur);
      start_state.col(1) = last_traj_.getVel(t_dur);
      start_state.col(2) = last_traj_.getAcc(t_dur);
    }
  }
  
  if (!hot_start)
  {
    start_state.col(0) = Eigen::Vector3d(start.position.x, start.position.y, 0.0);
  }

  // 2. 使用 A* 算法找到路径
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
  astar_planner_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
  astar_planner_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
  astar_planner_->setupNavFn(true);
  astar_planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  // 传播波前
  // 循环调用 propNavFnAstar 直到找到 Start 或队列为空
  // 给定一个足够大的总循环次数上限，防止死循环
  int max_total_cycles = nx * ny; 
  int cycles_per_step = std::max(nx * ny / 20, nx + ny);
  
  while (max_total_cycles > 0) {
    if (cancel_checker && cancel_checker()) {
      return false;
    }
    if (!astar_planner_->propNavFnAstar(cycles_per_step, cancel_checker)) {
      break;
    }
    max_total_cycles -= cycles_per_step;
  }

  // 提取路径
  if (!astar_planner_->calcPath(nx * ny / 2) || astar_planner_->getPathLen() < 2) {
    return false;
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
    guide_path.emplace_back(wx, wy, 0.0);
    plan.poses.push_back(pose);
  }

  std::vector<Eigen::Vector3d> sparse_path = getSparseWaypoints(guide_path);
  lock.unlock();
  // 3. 使用 Minco 优化器优化路径
  traj_opt::Trajectory opt_traj;
  end_state.col(0) = sparse_path.back();
  if (!minco_optimizer_->optimize(sparse_path, start_state, end_state, opt_traj))
  {
    RCLCPP_WARN(node_.lock()->get_logger(), "Minco optimization failed");
    return false;
  } 

  // 4. 将优化后的轨迹转换为导航路径
  double t_start = plan_start_time;
  double t_end = t_start + opt_traj.getTotalDuration();
  
  if (opt_traj.getTotalDuration() <= 1e-3) {
      t_end = t_start + 1e-3; // Ensure non-zero duration
  }

  // Re-sample based on fixed resolution to ensure path quality
  double resolution = costmap_->getResolution();
  int steps = std::max(2, static_cast<int>(opt_traj.getTotalDuration() / 0.1)); // Min 2 points, or 10Hz
  // Or better, use spatial resolution
  if (len < steps) {
      steps = std::max(len, static_cast<int>(opt_traj.getTotalDuration() / 0.05));
  } else {
      steps = len;
  }
  
  // Resize plan
  plan.poses.resize(steps);
  for (int i = 0; i < steps; ++i) {
      plan.poses[i].header = plan.header;
  }

  double t_step = (t_end - t_start) / (steps - 1);
  for (int i = 0; i < steps; ++i) {
    double t = i * t_step;
    Eigen::Vector3d pos = opt_traj.getPos(t);

    plan.poses[i].pose.position.x = pos(0);
    plan.poses[i].pose.position.y = pos(1);
    plan.poses[i].pose.position.z = 0.0;
    plan.poses[i].pose.orientation.x = 0.0;
    plan.poses[i].pose.orientation.y = 0.0;
    plan.poses[i].pose.orientation.z = 0.0;
    plan.poses[i].pose.orientation.w = 1.0;
  }

  RCLCPP_INFO(logger_, "MincoPlanner: Successfully created plan with %d poses, duration %f", steps, opt_traj.getTotalDuration());

  // 5. 保存优化后的轨迹
  last_traj_ = opt_traj;
  last_traj_.start_WT = t_start;
  has_last_traj_ = true;

  return !plan.poses.empty();
}

std::vector<Eigen::Vector3d> MincoPlanner::getSparseWaypoints(const std::vector<Eigen::Vector3d>& path) {
    std::vector<Eigen::Vector3d> sparse;
    if (path.empty()) return sparse;
    
    sparse.push_back(path.front());
    if (path.size() < 3) {
        sparse.push_back(path.back());
        return sparse;
    }
    
    const int lookahead_step = 0.5 / costmap_->getResolution();
    size_t current_idx = 0;

    while (current_idx < path.size() - 1) {
      size_t next_idx = current_idx + 1;
      Eigen::Vector3d curr_pt = path[current_idx];
      size_t max_lookahead = std::min(static_cast<size_t>(lookahead_step), path.size() - 1 - current_idx);
      for (size_t i = 1; i < max_lookahead; i++) {
        if (!isLineFree(curr_pt, path[current_idx + i])) {
          break;
        }
        next_idx = current_idx + i;
      }
      sparse.push_back(path[next_idx]);
      current_idx = next_idx;
    }
    
    if (sparse.back() != path.back()) {
      sparse.push_back(path.back());
    }
    return sparse;
}

bool MincoPlanner::isLineFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    if (!costmap_) return true; 
    unsigned int mx, my;
    double dist = (p2 - p1).norm();
    int steps = std::ceil(dist / costmap_->getResolution()); 
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d p = p1 + (p2 - p1) * t;
        if (costmap_->worldToMap(p.x(), p.y(), mx, my)) {
            if (costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { 
                return false; 
            }
        }
    }
    return true;
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
