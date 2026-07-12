// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "smac_search/smac_planner_2d_simple.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>

namespace minco_planner {
namespace smac {

SmacPlanner2DSimple::SmacPlanner2DSimple()
: allow_unknown_(true), max_iterations_(5000000), tolerance_(0.125), size_x_(0), size_y_(0),
  motion_model_(MotionModel::TWOD)
{
  search_info_.cost_penalty = 2.0;
}

SmacPlanner2DSimple::~SmacPlanner2DSimple()
{
}

void SmacPlanner2DSimple::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  map_ = map;
  if (!map_) {
    return;
  }
  costmap_origin_x_ = map_->originX();
  costmap_origin_y_ = map_->originY();
  costmap_resolution_ = map_->resolution();
  size_x_ = map_->sizeX();
  size_y_ = map_->sizeY();
  planning_id_ = 0u;
  ensureSearchBuffers();
}

void SmacPlanner2DSimple::setESDFQuery(
  const std::shared_ptr<rog_map::MapQueryInterface> & query)
{
  esdf_query_ = query;
  planning_id_ = 0u;
}

void SmacPlanner2DSimple::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  configure(node, costmap_ros, std::string{});
}

void SmacPlanner2DSimple::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const std::string & param_prefix)
{
  node_ = node;
  costmap_ros_ = costmap_ros;

  // Optional distance-field biasing (default off), queried from MapQueryInterface.
  auto full_key = [&param_prefix](const std::string & key) {
    if (param_prefix.empty()) {
      return key;
    }
    return param_prefix + key;
  };

  auto declare_or_get = [this, &full_key](const std::string & key, const auto & default_value) {
    using T = std::decay_t<decltype(default_value)>;
    const std::string name = full_key(key);
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<T>(name, default_value);
    }
    T value;
    node_->get_parameter(name, value);
    return value;
  };

  use_esdf_cost_ = declare_or_get("smac_2d.use_esdf_cost", false);
  esdf_weight_ = static_cast<float>(declare_or_get("smac_2d.esdf_weight", 1.0));
  esdf_decay_ = static_cast<float>(declare_or_get("smac_2d.esdf_decay", 0.5));
  esdf_max_cost_ = static_cast<float>(declare_or_get("smac_2d.esdf_max_cost", 5.0));

  if (esdf_decay_ <= 1e-3f) {
    esdf_decay_ = 1e-3f;
  }

  ensureSearchBuffers();
}

void SmacPlanner2DSimple::setParameters(bool allow_unknown, int max_iterations, float tolerance)
{
  allow_unknown_ = allow_unknown;
  max_iterations_ = max_iterations;
  tolerance_ = tolerance;
}

void SmacPlanner2DSimple::ensureSearchBuffers()
{
  grid_size_ = static_cast<uint64_t>(size_x_) * static_cast<uint64_t>(size_y_);
  const size_t size = static_cast<size_t>(grid_size_);
  if (size == 0u) {
    return;
  }

  if (g_score_.size() != size) {
    g_score_.assign(size, std::numeric_limits<float>::infinity());
    parent_.assign(size, -1);
    visited_.assign(size, 0u);
    closed_.assign(size, 0u);

    esdf_cost_cache_.assign(size, 0.0f);
    esdf_cost_cache_id_.assign(size, 0u);
  }
}

void SmacPlanner2DSimple::logFailure(const std::string & reason,
  unsigned int start_x,
  unsigned int start_y,
  unsigned int goal_x,
  unsigned int goal_y,
  int iterations) const
{
  const auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("SmacPlanner2DSimple");
  RCLCPP_WARN(logger,
    "[SMAC 2D] %s | start=(%u,%u) goal=(%u,%u) size=%ux%u origin=(%.3f,%.3f) res=%.3f "
    "allow_unknown=%s tolerance=%.3f max_iterations=%d iterations=%d",
    reason.c_str(),
    start_x,
    start_y,
    goal_x,
    goal_y,
    size_x_,
    size_y_,
    costmap_origin_x_,
    costmap_origin_y_,
    costmap_resolution_,
    allow_unknown_ ? "true" : "false",
    tolerance_,
    max_iterations_,
    iterations);
}

float SmacPlanner2DSimple::getESDFPotentialCost(unsigned int mx, unsigned int my)
{
  if (!use_esdf_cost_ || !map_ || !esdf_query_) {
    return 0.0f;
  }

  if (mx >= size_x_ || my >= size_y_) {
    return 0.0f;
  }

  const uint64_t index =
    static_cast<uint64_t>(my) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(mx);

  const size_t idx = static_cast<size_t>(index);
  if (esdf_cost_cache_id_[idx] == planning_id_) {
    return esdf_cost_cache_[idx];
  }

  // Convert through the prior search map; dynamicQuery handles any further frame conversion.
  double wx = 0.0;
  double wy = 0.0;
  map_->mapToWorld(mx, my, wx, wy);

  const auto result = esdf_query_->query(Eigen::Vector3d(wx, wy, 0.0));
  if (!result.ok || !std::isfinite(result.distance)) {
    esdf_cost_cache_[idx] = 0.0f;
    esdf_cost_cache_id_[idx] = planning_id_;
    return 0.0f;
  }

  const double dist = std::max(0.0, result.distance);

  const float normalized_potential =
    static_cast<float>(std::exp(-dist / static_cast<double>(esdf_decay_)));
  float weighted_potential = esdf_weight_ * normalized_potential;
  if (esdf_max_cost_ > 0.0f) {
    weighted_potential = std::min(weighted_potential, esdf_max_cost_);
  }

  esdf_cost_cache_[idx] = weighted_potential;
  esdf_cost_cache_id_[idx] = planning_id_;
  return esdf_cost_cache_[idx];
}

float SmacPlanner2DSimple::evaluateInflationCost(unsigned char cell_cost)
{
  if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
    return 1.0f;
  }

  const float normalized_cost = static_cast<float>(cell_cost) / MAX_NON_OBSTACLE_COST;

  if (cell_cost >= 253u) {
    return 50.0f;
  }

  if (cell_cost > 128u) {
    return 1.0f + 20.0f * (normalized_cost * normalized_cost * normalized_cost);
  }

  return 1.0f + search_info_.cost_penalty * (normalized_cost * normalized_cost);
}

bool SmacPlanner2DSimple::createPath(const unsigned int & start_x,
  const unsigned int & start_y,
  const unsigned int & goal_x,
  const unsigned int & goal_y,
  CoordinateVector & path,
  std::function<bool()> cancel_checker)
{
  path.clear();

  if (!map_) {
    logFailure("map query is null", start_x, start_y, goal_x, goal_y);
    return false;
  }

  // Refresh cached metadata (avoid per-cell getters in the search loop).
  costmap_origin_x_ = map_->originX();
  costmap_origin_y_ = map_->originY();
  costmap_resolution_ = map_->resolution();
  size_x_ = map_->sizeX();
  size_y_ = map_->sizeY();
  ensureSearchBuffers();

  if (size_x_ == 0u || size_y_ == 0u) {
    logFailure("map query has zero size", start_x, start_y, goal_x, goal_y);
    return false;
  }

  if (start_x >= size_x_ || start_y >= size_y_ || goal_x >= size_x_ || goal_y >= size_y_) {
    logFailure("start or goal is outside map bounds", start_x, start_y, goal_x, goal_y);
    return false;
  }

  ++planning_id_;
  if (planning_id_ == 0u) {
    std::fill(visited_.begin(), visited_.end(), 0u);
    std::fill(closed_.begin(), closed_.end(), 0u);
    std::fill(esdf_cost_cache_id_.begin(), esdf_cost_cache_id_.end(), 0u);
    planning_id_ = 1u;
  }

  const uint64_t start_index =
    static_cast<uint64_t>(start_y) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(start_x);
  const uint64_t goal_index =
    static_cast<uint64_t>(goal_y) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(goal_x);

  const size_t map_size = static_cast<size_t>(size_x_) * static_cast<size_t>(size_y_);
  std::vector<unsigned char> map_charmap;
  if (!map_->copyValues(map_charmap) || map_charmap.size() != map_size) {
    logFailure("failed to copy map values or copied map size mismatched",
      start_x,
      start_y,
      goal_x,
      goal_y);
    return false;
  }
  const auto * charmap = map_charmap.data();
  if (!charmap) {
    logFailure("copied map data is null", start_x, start_y, goal_x, goal_y);
    return false;
  }

  const auto is_traversable = [this, charmap](const uint64_t index) -> bool {
    const unsigned char cost = charmap[static_cast<size_t>(index)];
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return allow_unknown_;
    }
    return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  };

  if (!is_traversable(goal_index)) {
    const unsigned char goal_cost = charmap[static_cast<size_t>(goal_index)];
    logFailure("goal cell is not traversable, cost=" + std::to_string(static_cast<unsigned int>(goal_cost)),
      start_x,
      start_y,
      goal_x,
      goal_y);
    return false;
  }

  const float step_cardinal = static_cast<float>(costmap_resolution_);
  const float step_diagonal = step_cardinal * 1.41421356237f;
  const float D = step_cardinal;
  const float D2 = step_diagonal;

  const auto heuristic = [D, D2, goal_x, goal_y](const unsigned int x, const unsigned int y) {
    const unsigned int dx = (x > goal_x) ? (x - goal_x) : (goal_x - x);
    const unsigned int dy = (y > goal_y) ? (y - goal_y) : (goal_y - y);
    const unsigned int min_d = (dx < dy) ? dx : dy;
    return D * static_cast<float>(dx + dy) + (D2 - 2.0f * D) * static_cast<float>(min_d);
  };

  const float tol_cells =
    (costmap_resolution_ > 1e-9) ? static_cast<float>(tolerance_ / costmap_resolution_) : tolerance_;
  const float tol_sq = tol_cells * tol_cells;

  // Min-heap open list
  std::priority_queue<NodeMin, std::vector<NodeMin>, std::greater<NodeMin>> open;

  visited_[static_cast<size_t>(start_index)] = planning_id_;
  closed_[static_cast<size_t>(start_index)] = 0u;
  g_score_[static_cast<size_t>(start_index)] = 0.0f;
  parent_[static_cast<size_t>(start_index)] = -1;
  open.push(NodeMin{start_index, heuristic(start_x, start_y)});

  uint64_t goal_reached_index = goal_index;
  bool goal_reached = false;
  int iterations = 0;

  while (!open.empty() && iterations < max_iterations_) {
    if (cancel_checker && cancel_checker()) {
      logFailure("planning canceled", start_x, start_y, goal_x, goal_y, iterations);
      return false;
    }

    const NodeMin current = open.top();
    open.pop();

    const uint64_t current_index = current.index;
    const size_t current_i = static_cast<size_t>(current_index);
    if (closed_[current_i] == planning_id_) {
      continue;
    }
    closed_[current_i] = planning_id_;
    ++iterations;

    const uint64_t cy = current_index / static_cast<uint64_t>(size_x_);
    const uint64_t cx = current_index - cy * static_cast<uint64_t>(size_x_);
    const unsigned int cx_u = static_cast<unsigned int>(cx);
    const unsigned int cy_u = static_cast<unsigned int>(cy);

    const float dx = static_cast<float>((cx_u > goal_x) ? (cx_u - goal_x) : (goal_x - cx_u));
    const float dy = static_cast<float>((cy_u > goal_y) ? (cy_u - goal_y) : (goal_y - cy_u));
    if (dx * dx + dy * dy <= tol_sq) {
      goal_reached = true;
      goal_reached_index = current_index;
      break;
    }

    const float g_current = g_score_[current_i];

    // 8-connected neighbors: (dx,dy,cost)
    // No division/mod in the inner neighbor expansion loop.
    const int offsets[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};

    for (int k = 0; k < 8; ++k) {
      const int nx_i = static_cast<int>(cx_u) + offsets[k][0];
      const int ny_i = static_cast<int>(cy_u) + offsets[k][1];
      if (nx_i < 0 || ny_i < 0) {
        continue;
      }
      const unsigned int nx = static_cast<unsigned int>(nx_i);
      const unsigned int ny = static_cast<unsigned int>(ny_i);
      if (nx >= size_x_ || ny >= size_y_) {
        continue;
      }

      const uint64_t neighbor_index =
        static_cast<uint64_t>(ny) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(nx);
      const size_t neighbor_i = static_cast<size_t>(neighbor_index);
      if (closed_[neighbor_i] == planning_id_) {
        continue;
      }

      if (!is_traversable(neighbor_index)) {
        continue;
      }

      const bool diagonal = (k >= 4);
      const float step_cost = diagonal ? step_diagonal : step_cardinal;

      const unsigned char cell_cost = charmap[neighbor_i];
      const float traversal_factor = evaluateInflationCost(cell_cost);

      float tentative_g = g_current + step_cost * traversal_factor;
      if (use_esdf_cost_) {
        tentative_g += step_cost * getESDFPotentialCost(nx, ny);
      }

      if (visited_[neighbor_i] != planning_id_ || tentative_g < g_score_[neighbor_i]) {
        visited_[neighbor_i] = planning_id_;
        g_score_[neighbor_i] = tentative_g;
        parent_[neighbor_i] = static_cast<int>(current_index);

        const float f = tentative_g + heuristic(nx, ny);
        open.push(NodeMin{neighbor_index, f});
      }
    }
  }

  if (!goal_reached && closed_[static_cast<size_t>(goal_index)] != planning_id_) {
    logFailure(iterations >= max_iterations_ ? "max iterations reached before goal" : "open set exhausted before goal",
      start_x,
      start_y,
      goal_x,
      goal_y,
      iterations);
    return false;
  }

  // Backtrace (goal -> start). Caller expects reversed ordering.
  uint64_t index = goal_reached ? goal_reached_index : goal_index;
  for (;;) {
    const uint64_t y = index / static_cast<uint64_t>(size_x_);
    const uint64_t x = index - y * static_cast<uint64_t>(size_x_);
    path.emplace_back(static_cast<float>(x), static_cast<float>(y));

    if (index == start_index) {
      break;
    }

    const int p = parent_[static_cast<size_t>(index)];
    if (p < 0) {
      logFailure("path backtrace failed: missing parent", start_x, start_y, goal_x, goal_y, iterations);
      return false;
    }
    index = static_cast<uint64_t>(p);
  }

  if (path.size() < 2u) {
    logFailure("path has fewer than 2 poses", start_x, start_y, goal_x, goal_y, iterations);
    return false;
  }

  return true;
}

}  // namespace smac
}  // namespace minco_planner
