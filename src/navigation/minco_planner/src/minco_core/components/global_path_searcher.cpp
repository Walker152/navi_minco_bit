#include "minco_core/components/global_path_searcher.hpp"

#include <queue>

namespace minco_planner {

namespace {

bool projectStartToFreeCell(
  const std::shared_ptr<rog_map::MapQueryInterface> & map, unsigned int & mx, unsigned int & my)
{
  if (!map || map->isFree(mx, my)) {
    return true;
  }
  const unsigned int nx = map->sizeX();
  const unsigned int ny = map->sizeY();
  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    int sx = static_cast<int>(mx) + dx4[k];
    int sy = static_cast<int>(my) + dy4[k];
    if (sx < 0 || sy < 0 || sx >= static_cast<int>(nx) || sy >= static_cast<int>(ny)) {
      continue;
    }
    if (map->isFree(static_cast<unsigned int>(sx), static_cast<unsigned int>(sy))) {
      mx = static_cast<unsigned int>(sx);
      my = static_cast<unsigned int>(sy);
      return true;
    }
  }
  constexpr int kMaxRadius = 50;
  for (int r = 1; r <= kMaxRadius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) != r && std::abs(dy) != r) {
          continue;
        }
        int cx = static_cast<int>(mx) + dx;
        int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(nx) || cy >= static_cast<int>(ny)) {
          continue;
        }
        if (map->isFree(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy))) {
          mx = static_cast<unsigned int>(cx);
          my = static_cast<unsigned int>(cy);
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

void GlobalPathSearcher::configure(std::shared_ptr<tf2_ros::Buffer> tf,
  Astar * astar,
  smac::SmacPlanner2DSimple * smac,
  bool use_smac,
  bool allow_unknown,
  double tolerance,
  rclcpp::Logger logger)
{
  tf_ = std::move(tf);
  astar_ = astar;
  smac_ = smac;
  use_smac_ = use_smac;
  allow_unknown_ = allow_unknown;
  tolerance_ = tolerance;
  logger_ = logger;
}

void GlobalPathSearcher::setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query)
{
  global_query_ = global_query;
  if (astar_) {
    astar_->setMap(global_query_);
  }
  if (smac_ && global_query_) {
    smac_->setMap(global_query_);
  }
}

bool GlobalPathSearcher::normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
  const std::string & fallback_frame,
  const std::string & target_frame,
  const std::string & context,
  geometry_msgs::msg::PoseStamped & out) const
{
  out = in;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = fallback_frame;
    RCLCPP_WARN(logger_,
      "[MincoPlanner] %s pose frame is empty, treating it as %s.",
      context.c_str(),
      fallback_frame.c_str());
  }

  if (out.header.frame_id == target_frame) {
    out.header.frame_id = target_frame;
    return true;
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Cannot transform %s pose from %s to %s: TF buffer is null.",
      context.c_str(),
      out.header.frame_id.c_str(),
      target_frame.c_str());
    return false;
  }

  try {
    out = tf_->transform(out, target_frame);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to transform %s pose from %s to %s: %s",
      context.c_str(),
      in.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }
}

bool GlobalPathSearcher::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  if (mode_context.mode() == PlannerMode::PRIORMAP) {
    return planPriorMap(start, goal, mode_context, latest_global_path);
  }
  return planExploration(start, goal, mode_context, latest_global_path);
}

bool GlobalPathSearcher::planPriorMap(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  if (!astar_ || !mode_context.globalQuery()) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Nav2 costmap global search query is unavailable.");
    return false;
  }

  geometry_msgs::msg::PoseStamped start_map;
  geometry_msgs::msg::PoseStamped goal_map;
  if (!normalizePoseToFrame(
        start, mode_context.mapFrame(), mode_context.mapFrame(), "PRIORMAP start", start_map) ||
      !normalizePoseToFrame(
        goal, mode_context.mapFrame(), mode_context.mapFrame(), "PRIORMAP goal", goal_map)) {
    return false;
  }

  nav_msgs::msg::Path dummy;
  dummy.header.stamp = rclcpp::Clock().now();
  dummy.header.frame_id = mode_context.outputFrame();

  std::function<bool()> cancel_checker = []() {
    return !rclcpp::ok();
  };

  return makePlanOnQuery(start_map.pose,
    goal_map.pose,
    mode_context.globalQuery(),
    mode_context.outputFrame(),
    "Nav2 costmap",
    tolerance_,
    cancel_checker,
    dummy,
    latest_global_path);
}

bool GlobalPathSearcher::planExploration(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  const auto query = mode_context.globalQuery();
  if (!astar_ || !query) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap global search query is unavailable.");
    return false;
  }

  geometry_msgs::msg::PoseStamped start_rog;
  geometry_msgs::msg::PoseStamped goal_rog;
  if (!normalizePoseToFrame(
        start, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION start", start_rog) ||
      !normalizePoseToFrame(
        goal, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION goal", goal_rog)) {
    return false;
  }

  std::function<bool()> cancel_checker = []() {
    return !rclcpp::ok();
  };

  unsigned int sx = 0;
  unsigned int sy = 0;
  if (!query->worldToMap(start_rog.pose.position.x, start_rog.pose.position.y, sx, sy)) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] ROGMap boundary check failed: EXPLORATION start (%.2f, %.2f) is outside ROGMap.",
      start_rog.pose.position.x,
      start_rog.pose.position.y);
    return false;
  }

  unsigned int gx = 0;
  unsigned int gy = 0;
  const bool goal_inside = query->worldToMap(goal_rog.pose.position.x, goal_rog.pose.position.y, gx, gy);
  bool goal_traversable = false;
  if (goal_inside) {
    const unsigned char goal_cost = query->value(gx, gy);
    goal_traversable = goal_cost == nav2_costmap_2d::NO_INFORMATION
                         ? !mode_context.explorationUnknownAsOccupied()
                         : goal_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
  if (goal_traversable) {
    nav_msgs::msg::Path direct_plan;
    direct_plan.header.stamp = rclcpp::Clock().now();
    direct_plan.header.frame_id = mode_context.outputFrame();
    if (makePlanOnQuery(start_rog.pose,
          goal_rog.pose,
          query,
          mode_context.outputFrame(),
          "ROGMap",
          tolerance_,
          cancel_checker,
          direct_plan,
          latest_global_path)) {
      return true;
    }
    RCLCPP_WARN(logger_,
      "[MincoPlanner] Exploration goal is inside ROGMap and traversable but unreachable; "
      "fallback to reachable boundary search.");
  }

  const unsigned int nx = query->sizeX();
  const unsigned int ny = query->sizeY();
  if (nx == 0U || ny == 0U) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap boundary search failed: empty ROGMap.");
    return false;
  }

  std::vector<unsigned char> costs;
  const size_t map_size = static_cast<size_t>(nx) * static_cast<size_t>(ny);
  if (!query->copyValues(costs) || costs.size() != map_size) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap boundary search failed: cannot copy ROGMap values.");
    return false;
  }

  const auto index_of = [nx](unsigned int x, unsigned int y) {
    return static_cast<size_t>(y) * static_cast<size_t>(nx) + static_cast<size_t>(x);
  };

  const bool allow_unknown = !mode_context.explorationUnknownAsOccupied();
  const auto traversable = [&costs, allow_unknown](size_t idx) {
    const unsigned char cost = costs[idx];
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return allow_unknown;
    }
    return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  const size_t start_idx = index_of(sx, sy);
  if (!traversable(start_idx)) {
    projectStartToFreeCell(query, sx, sy);
  }

  const double resolution = std::max(1e-6, query->resolution());
  const int margin_cells =
    std::max(1, static_cast<int>(std::ceil(mode_context.explorationBoundaryMargin() / resolution)));

  struct QueueNode
  {
    size_t idx{0};
    double cost{0.0};
    bool operator>(const QueueNode & other) const { return cost > other.cost; }
  };

  std::vector<double> dist(map_size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(map_size, -1);
  std::vector<uint8_t> closed(map_size, 0U);
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  const size_t updated_start_idx = index_of(sx, sy);
  if (!traversable(updated_start_idx)) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] EXPLORATION start cell is not traversable in ROGMap.");
    return false;
  }
  dist[updated_start_idx] = 0.0;
  open.push(QueueNode{updated_start_idx, 0.0});

  const Eigen::Vector2d start_xy(start_rog.pose.position.x, start_rog.pose.position.y);
  const Eigen::Vector2d goal_dir =
    (Eigen::Vector2d(goal_rog.pose.position.x, goal_rog.pose.position.y) - start_xy);
  Eigen::Vector2d goal_dir_norm = Eigen::Vector2d::Zero();
  if (goal_dir.norm() > 1e-6) {
    goal_dir_norm = goal_dir.normalized();
  }

  size_t best_idx = map_size;
  double best_score = std::numeric_limits<double>::infinity();
  const int offsets[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};

  while (!open.empty()) {
    if (cancel_checker && cancel_checker()) {
      return false;
    }
    const QueueNode current = open.top();
    open.pop();
    if (closed[current.idx]) {
      continue;
    }
    closed[current.idx] = 1U;

    const unsigned int cx = static_cast<unsigned int>(current.idx % nx);
    const unsigned int cy = static_cast<unsigned int>(current.idx / nx);
    const int min_edge_dist = std::min({static_cast<int>(cx),
      static_cast<int>(cy),
      static_cast<int>(nx - 1U - cx),
      static_cast<int>(ny - 1U - cy)});
    if (min_edge_dist <= margin_cells) {
      double wx = 0.0;
      double wy = 0.0;
      query->mapToWorld(cx, cy, wx, wy);
      double score = current.cost;
      if (mode_context.explorationPreferGoalDirection() && goal_dir_norm.norm() > 1e-6) {
        Eigen::Vector2d candidate_dir(wx - start_xy.x(), wy - start_xy.y());
        if (candidate_dir.norm() > 1e-6) {
          const double align = candidate_dir.normalized().dot(goal_dir_norm);
          score += (1.0 - align) * std::max(1.0, mode_context.explorationBoundaryMargin());
        }
      }
      if (score < best_score) {
        best_score = score;
        best_idx = current.idx;
      }
    }

    for (const auto & offset : offsets) {
      const int nx_i = static_cast<int>(cx) + offset[0];
      const int ny_i = static_cast<int>(cy) + offset[1];
      if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<int>(nx) || ny_i >= static_cast<int>(ny)) {
        continue;
      }
      const unsigned int nbx = static_cast<unsigned int>(nx_i);
      const unsigned int nby = static_cast<unsigned int>(ny_i);
      const size_t nb_idx = index_of(nbx, nby);
      if (closed[nb_idx] || !traversable(nb_idx)) {
        continue;
      }
      const bool diagonal = std::abs(offset[0]) + std::abs(offset[1]) == 2;
      const double step = (diagonal ? 1.41421356237 : 1.0) * resolution;
      const double next_dist = current.cost + step;
      if (next_dist < dist[nb_idx]) {
        dist[nb_idx] = next_dist;
        parent[nb_idx] = static_cast<int>(current.idx);
        open.push(QueueNode{nb_idx, next_dist});
      }
    }
  }

  if (best_idx >= map_size) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] EXPLORATION cannot find a reachable ROGMap boundary candidate.");
    return false;
  }

  std::vector<size_t> reversed;
  for (size_t idx = best_idx;;) {
    reversed.push_back(idx);
    if (idx == updated_start_idx) {
      break;
    }
    const int p = parent[idx];
    if (p < 0) {
      RCLCPP_ERROR(logger_, "[MincoPlanner] EXPLORATION boundary path reconstruction failed.");
      return false;
    }
    idx = static_cast<size_t>(p);
  }

  latest_global_path.clear();
  latest_global_path.reserve(reversed.size());
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    const unsigned int x = static_cast<unsigned int>(*it % nx);
    const unsigned int y = static_cast<unsigned int>(*it / nx);
    double wx = 0.0;
    double wy = 0.0;
    query->mapToWorld(x, y, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = mode_context.outputFrame();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    latest_global_path.push_back(pose);
  }
  return latest_global_path.size() >= 2U;
}

bool GlobalPathSearcher::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const PlannerModeContext & mode_context,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  std::vector<geometry_msgs::msg::PoseStamped> ignored_path;
  return makePlanOnQuery(start,
    goal,
    mode_context.globalQuery(),
    mode_context.outputFrame(),
    mode_context.mode() == PlannerMode::PRIORMAP ? "Nav2 costmap" : "ROGMap",
    tolerance,
    cancel_checker,
    plan,
    ignored_path);
}

bool GlobalPathSearcher::makePlanOnQuery(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & output_frame,
  const std::string & failure_source,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  (void)tolerance;

  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = output_frame;
  if (!query) {
    RCLCPP_ERROR(
      logger_, "[MincoPlanner] %s query is not available for global planning.", failure_source.c_str());
    return false;
  }

  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start = 0;
  unsigned int my_start = 0;
  if (!query->worldToMap(wx, wy, mx_start, my_start)) {
    RCLCPP_ERROR(logger_,
      "%s worldToMap failed for start world coordinates (%.2f, %.2f)",
      failure_source.c_str(),
      wx,
      wy);
    return false;
  }

  if (!projectStartToFreeCell(query, mx_start, my_start)) {
    RCLCPP_ERROR(logger_,
      "%s failed to project start cell to a traversable free cell near world coordinates (%.2f, %.2f)",
      failure_source.c_str(),
      wx,
      wy);
    return false;
  }

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal = 0;
  unsigned int my_goal = 0;
  if (!query->worldToMap(wx, wy, mx_goal, my_goal)) {
    RCLCPP_ERROR(logger_,
      "%s worldToMap failed for goal world coordinates (%.2f, %.2f)",
      failure_source.c_str(),
      wx,
      wy);
    return false;
  }
  unsigned int nx = query->sizeX();
  unsigned int ny = query->sizeY();
  if (use_smac_ && smac_) {
    smac::SmacPlanner2DSimple::CoordinateVector smac_path;
    bool smac_success = smac_->createPath(mx_start, my_start, mx_goal, my_goal, smac_path, cancel_checker);

    if (!smac_success || smac_path.size() < 2) {
      RCLCPP_ERROR(logger_, "SMAC 2D: Failed to find path");
      return false;
    }

    latest_global_path.clear();
    latest_global_path.reserve(smac_path.size());
    plan.poses.reserve(smac_path.size());

    for (auto it = smac_path.rbegin(); it != smac_path.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double path_wx = 0.0;
      double path_wy = 0.0;
      query->mapToWorld(
        static_cast<unsigned int>(it->x), static_cast<unsigned int>(it->y), path_wx, path_wy);

      pose.pose.position.x = path_wx;
      pose.pose.position.y = path_wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path.push_back(pose);
      plan.poses.push_back(pose);
    }
  } else {
    if (!astar_) {
      return false;
    }
    astar_->setSize(nx, ny);
    astar_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
    astar_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
    astar_->setupNavFn(true);
    std::vector<unsigned char> query_costmap_copy;
    const size_t map_size = static_cast<size_t>(nx) * static_cast<size_t>(ny);
    if (!query->copyValues(query_costmap_copy) || query_costmap_copy.size() != map_size) {
      return false;
    }
    astar_->setCostmap(query_costmap_copy.data(), true, allow_unknown_);

    int max_total_cycles = static_cast<int>(nx * ny) * 9999;
    int cycles_per_step = std::max(static_cast<int>(nx * ny / 20), static_cast<int>(nx + ny));
    while (max_total_cycles > 0) {
      if (cancel_checker && cancel_checker()) {
        return false;
      }
      if (!astar_->propNavFnAstar(cycles_per_step, cancel_checker)) {
        break;
      }
      max_total_cycles -= cycles_per_step;
    }

    if (!astar_->calcPath(nx * ny / 2) || astar_->getPathLen() < 2) {
      return false;
    }

    float * path_x = astar_->getPathX();
    float * path_y = astar_->getPathY();
    const int len = astar_->getPathLen();

    latest_global_path.clear();
    latest_global_path.reserve(len);
    plan.poses.reserve(len);

    for (int i = 0; i < len; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double path_wx = 0.0;
      double path_wy = 0.0;
      query->mapToWorld(
        static_cast<unsigned int>(path_x[i]), static_cast<unsigned int>(path_y[i]), path_wx, path_wy);

      pose.pose.position.x = path_wx;
      pose.pose.position.y = path_wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path.push_back(pose);
      plan.poses.push_back(pose);
    }
  }

  if (!plan.poses.empty()) {
    plan.poses.back().pose.position.x = goal.position.x;
    plan.poses.back().pose.position.y = goal.position.y;
    plan.poses.back().pose.position.z = goal.position.z;
    plan.poses.back().pose.orientation = goal.orientation;
    latest_global_path.back().pose.position.x = goal.position.x;
    latest_global_path.back().pose.position.y = goal.position.y;
    latest_global_path.back().pose.position.z = goal.position.z;
    latest_global_path.back().pose.orientation = goal.orientation;
  }

  return !plan.poses.empty();
}

}  // namespace minco_planner
