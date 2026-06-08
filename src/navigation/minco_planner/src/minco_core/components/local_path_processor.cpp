#include "minco_core/components/local_path_processor.hpp"

namespace minco_planner {

namespace {

bool isLineFree(const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  if (!map || map->resolution() <= 0.0) {
    return true;
  }
  const double dist = (p2 - p1).norm();
  const int steps = static_cast<int>(std::ceil(dist / map->resolution()));
  if (steps <= 0) {
    return true;
  }
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector3d p = p1 + (p2 - p1) * t;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!map->worldToMap(p.x(), p.y(), mx, my) || !map->isFree(mx, my)) {
      return false;
    }
  }
  return true;
}

}  // namespace

void LocalPathProcessor::configure(
  double lookahead_dist, double max_vel, double max_acc, double traj_goal_tolerance, rclcpp::Logger logger)
{
  lookahead_dist_ = lookahead_dist;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
  logger_ = logger;
}

void LocalPathProcessor::updateLimits(double max_vel, double max_acc, double traj_goal_tolerance)
{
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
}

LocalPathSeed LocalPathProcessor::buildSeed(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const PlannerModeContext & mode_context) const
{
  LocalPathSeed seed;
  if (global_path.empty()) {
    return seed;
  }

  Eigen::Vector3d global_goal(global_path.back().pose.position.x, global_path.back().pose.position.y, 0.0);
  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  seed.dense_path = extractLocalPath(global_path, cur_pos);
  const bool clip_required =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  const bool clip_ok = clipLocalPathByRogBoundary(seed.dense_path, mode_context);
  if (clip_required && (!clip_ok || seed.dense_path.size() < 2U)) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] Local seed path is outside ROGMap boundary or too short after clipping.");
    return seed;
  }
  if (seed.dense_path.size() < 2U) {
    return seed;
  }

  seed.local_end_is_goal = (global_goal - seed.dense_path.back()).head<2>().norm() <= traj_goal_tolerance_;
  seed.sparse_waypoints = utils::getSparseWaypoints(seed.dense_path,
    max_vel_,
    max_acc_,
    seed.local_end_is_goal,
    [&mode_context](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return isLineFree(mode_context.sparsifyQuery(), a, b);
    });
  seed.valid = seed.sparse_waypoints.size() >= 2U;
  return seed;
}

std::vector<Eigen::Vector3d> LocalPathProcessor::extractLocalPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path, const Eigen::Vector3d & cur_pos) const
{
  std::vector<Eigen::Vector3d> local_segment;
  if (global_path.empty()) {
    return local_segment;
  }

  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_path.size(); ++i) {
    const auto & pt = global_path[i].pose.position;
    double dist_sq =
      (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  double accum_dist = 0.0;
  local_segment.push_back(
    Eigen::Vector3d(global_path[start_idx].pose.position.x, global_path[start_idx].pose.position.y, 0.0));

  for (size_t i = start_idx + 1; i < global_path.size(); ++i) {
    const auto & p1 = global_path[i - 1].pose.position;
    const auto & p2 = global_path[i].pose.position;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    accum_dist += dist;

    local_segment.push_back(Eigen::Vector3d(p2.x, p2.y, 0.0));

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

bool LocalPathProcessor::clipLocalPathByRogBoundary(
  std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const
{
  if (path.empty()) {
    return false;
  }

  const bool enable_clip =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  if (!enable_clip) {
    return path.size() >= 2U;
  }

  const auto query = mode_context.dynamicQuery();
  if (!query) {
    return false;
  }

  const double margin = mode_context.mode() == PlannerMode::PRIORMAP
                          ? mode_context.rogBoundaryMargin()
                          : mode_context.explorationBoundaryMargin();
  const int margin_cells =
    std::max(0, static_cast<int>(std::ceil(margin / std::max(1e-6, query->resolution()))));
  const int max_x = static_cast<int>(query->sizeX());
  const int max_y = static_cast<int>(query->sizeY());
  if (max_x <= 0 || max_y <= 0) {
    return false;
  }

  const double sample_step = mode_context.mode() == PlannerMode::PRIORMAP
                               ? mode_context.rogBoundarySampleStep()
                               : mode_context.explorationBoundarySampleStep();
  const double step = std::max(query->resolution(), std::max(1e-3, sample_step));

  auto inside_boundary = [&query, margin_cells, max_x, max_y](const Eigen::Vector3d & p) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!query->worldToMap(p.x(), p.y(), mx, my)) {
      return false;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    return ix >= margin_cells && iy >= margin_cells && ix < max_x - margin_cells &&
           iy < max_y - margin_cells;
  };

  std::vector<Eigen::Vector3d> clipped;
  clipped.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    if (!inside_boundary(path[i])) {
      if (i == 0U) {
        path.clear();
        RCLCPP_WARN_THROTTLE(logger_,
          *rclcpp::Clock::make_shared(),
          2000,
          "[MincoPlanner] Local seed path starts outside ROGMap boundary; reject local replan seed.");
        return false;
      }
      break;
    }
    if (i > 0U) {
      const Eigen::Vector3d delta = path[i] - path[i - 1U];
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
      for (int s = 1; s <= samples; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        if (!inside_boundary(path[i - 1U] + ratio * delta)) {
          path.swap(clipped);
          return path.size() >= 2U;
        }
      }
    }
    clipped.push_back(path[i]);
  }

  path.swap(clipped);
  return path.size() >= 2U;
}

}  // namespace minco_planner
