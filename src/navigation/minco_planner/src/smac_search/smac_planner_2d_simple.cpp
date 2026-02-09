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
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>
#include <type_traits>
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <Eigen/Core>

#include "small_rog_map/hybrid_esdf_map.hpp"

namespace minco_planner
{
namespace smac
{

SmacPlanner2DSimple::SmacPlanner2DSimple()
: allow_unknown_(true),
  max_iterations_(1000000),
  tolerance_(0.125),
  costmap_(nullptr),
  size_x_(0),
  size_y_(0),
  motion_model_(MotionModel::TWOD)
{
  search_info_.cost_penalty = 2.0;
}

SmacPlanner2DSimple::~SmacPlanner2DSimple()
{
}

void SmacPlanner2DSimple::setESDFMap(
  const std::shared_ptr<small_rog_map::HybridESDFMap> & esdf_map)
{
  esdf_map_ = esdf_map;
  esdf_cost_cache_.clear();
}

void SmacPlanner2DSimple::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  configure(node, costmap_ros, std::string{});
}

void SmacPlanner2DSimple::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const std::string & param_prefix)
{
  node_ = node;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // Optional ESDF biasing (default off). ESDF map instance is injected via setESDFMap().
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

  // Calculate circumscribed cost for collision checking optimization
  double possible_collision_cost = 0.0;
  nav2_costmap_2d::InflationLayer * inflation_layer = nullptr;
  
  // Find inflation layer
  auto layers = costmap_ros_->getLayeredCostmap()->getPlugins();
  for (auto layer = layers->begin(); layer != layers->end(); ++layer) {
    auto inflation = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (inflation) {
      inflation_layer = inflation.get();
      break;
    }
  }
  
  if (inflation_layer != nullptr) {
    double circum_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    double resolution = costmap_->getResolution();
    
    // Try to get inflation_radius from parameters
    double inflation_radius = 0.0;
    try {
      rclcpp::Parameter param = node_->get_parameter("global_costmap.inflation_layer.inflation_radius");
      inflation_radius = param.as_double();
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Could not get inflation_radius from parameters, using circumscribed_radius");
      inflation_radius = circum_radius;  // Use circumscribed_radius as fallback
    }
    
    if (inflation_radius >= circum_radius) {
      possible_collision_cost = static_cast<double>(
        inflation_layer->computeCost(circum_radius / resolution));
      RCLCPP_INFO(
        node_->get_logger(),
        "SMAC: Circumscribed cost computed as %.2f (inflation_radius=%.2f, circum_radius=%.2f)",
        possible_collision_cost, inflation_radius, circum_radius);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "SMAC: Inflation radius (%.2f) < circumscribed radius (%.2f). "
        "Setting possible_collision_cost to 0. This may impact performance.",
        inflation_radius, circum_radius);
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "SMAC: No inflation layer found. Using possible_collision_cost=0. "
      "This may impact performance.");
  }

  // Initialize collision checker
  collision_checker_ = std::make_unique<GridCollisionChecker>(
    costmap_ros_, 1, node_);
  collision_checker_->setFootprint(
    costmap_ros_->getRobotFootprint(),
    true,  // Use radius
    possible_collision_cost);

  // Initialize motion model
  unsigned int dummy_angle = 1;
  Node2D::initMotionModel(motion_model_, size_x_, size_y_, dummy_angle, search_info_);
}

void SmacPlanner2DSimple::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;

  const unsigned int new_size_x = costmap_->getSizeInCellsX();
  const unsigned int new_size_y = costmap_->getSizeInCellsY();
  if (new_size_x != size_x_ || new_size_y != size_y_) {
    size_x_ = new_size_x;
    size_y_ = new_size_y;
    clearNodePool();

    unsigned int dummy_angle = 1;
    Node2D::initMotionModel(motion_model_, size_x_, size_y_, dummy_angle, search_info_);
  }

  esdf_cost_cache_.clear();
}

void SmacPlanner2DSimple::setParameters(
  bool allow_unknown,
  int max_iterations,
  float tolerance)
{
  allow_unknown_ = allow_unknown;
  max_iterations_ = max_iterations;
  tolerance_ = tolerance;
}

void SmacPlanner2DSimple::clear()
{
  for (auto * node : touched_nodes_) {
    if (node) {
      node->reset();
    }
  }
  touched_nodes_.clear();
  esdf_cost_cache_.clear();

  // Bump search id used for touch tracking.
  ++search_id_;
  if (search_id_ == 0u) {
    search_id_ = 1u;
    for (auto & node_ptr : graph_) {
      node_ptr->setLastSearchId(0u);
    }
  }
}

void SmacPlanner2DSimple::clearNodePool()
{
  graph_.clear();
  graph_lookup_.clear();
  touched_nodes_.clear();
  esdf_cost_cache_.clear();
}

float SmacPlanner2DSimple::getESDFPotentialCost(const uint64_t & index)
{
  if (!use_esdf_cost_ || !esdf_map_ || !costmap_) {
    return 0.0f;
  }

  auto it = esdf_cost_cache_.find(index);
  if (it != esdf_cost_cache_.end()) {
    return it->second;
  }

  const auto coords = Node2D::getCoords(index, size_x_, 1);
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(
    static_cast<unsigned int>(coords.x),
    static_cast<unsigned int>(coords.y),
    wx, wy);

  double dist = 0.0;
  Eigen::Vector3d grad(0.0, 0.0, 0.0);
  esdf_map_->evaluate(Eigen::Vector3d(wx, wy, 0.0), dist, grad);

  if (!std::isfinite(dist) || dist < 0.0) {
    dist = 0.0;
  }

  // Potential field: higher cost near obstacles, decays with distance.
  float cost = static_cast<float>(esdf_weight_ * std::exp(-dist / static_cast<double>(esdf_decay_)));
  if (esdf_max_cost_ > 0.0f && cost > esdf_max_cost_) {
    cost = esdf_max_cost_;
  }

  esdf_cost_cache_.emplace(index, cost);
  return cost;
}

Node2D * SmacPlanner2DSimple::getOrAddNode(const uint64_t & index)
{
  auto it = graph_lookup_.find(index);
  if (it != graph_lookup_.end()) {
    NodePtr node = it->second;
    if (node->getLastSearchId() != search_id_) {
      node->setLastSearchId(search_id_);
      touched_nodes_.push_back(node);
    }
    return node;
  }

  graph_.emplace_back(std::make_unique<Node2D>(index));
  NodePtr node = graph_.back().get();
  graph_lookup_.emplace(index, node);
  node->setLastSearchId(search_id_);
  touched_nodes_.push_back(node);
  return node;
}

float SmacPlanner2DSimple::computeHeuristic(const NodePtr & node, const NodePtr & goal)
{
  auto node_coords = Node2D::getCoords(node->getIndex(), size_x_, 1);
  auto goal_coords = Node2D::getCoords(goal->getIndex(), size_x_, 1);
  
  float dx = goal_coords.x - node_coords.x;
  float dy = goal_coords.y - node_coords.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool SmacPlanner2DSimple::createPath(
  const unsigned int & start_x,
  const unsigned int & start_y,
  const unsigned int & goal_x,
  const unsigned int & goal_y,
  CoordinateVector & path,
  std::function<bool()> cancel_checker)
{
  clear();
  path.clear();

  // Check if costmap is valid
  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "Costmap not set!");
    return false;
  }
  
  // Update costmap size in case it changed (e.g., rolling window)
  unsigned int new_size_x = costmap_->getSizeInCellsX();
  unsigned int new_size_y = costmap_->getSizeInCellsY();
  
  // Reinitialize motion model if costmap size changed
  if (new_size_x != size_x_ || new_size_y != size_y_) {
    RCLCPP_INFO(node_->get_logger(),
      "SMAC 2D: Costmap size changed from %ux%u to %ux%u, reinitializing motion model",
      size_x_, size_y_, new_size_x, new_size_y);
    
    size_x_ = new_size_x;
    size_y_ = new_size_y;

    // Node indices depend on size_x_, so clear the pool on size changes.
    clearNodePool();
    
    unsigned int dummy_angle = 1;
    Node2D::initMotionModel(motion_model_, size_x_, size_y_, dummy_angle, search_info_);
  }
  
  // Validate start and goal are within costmap bounds
  if (start_x >= size_x_ || start_y >= size_y_) {
    RCLCPP_ERROR(node_->get_logger(),
      "SMAC 2D: Start position (%u, %u) is outside costmap bounds (%u, %u)",
      start_x, start_y, size_x_, size_y_);
    return false;
  }
  
  if (goal_x >= size_x_ || goal_y >= size_y_) {
    RCLCPP_ERROR(node_->get_logger(),
      "SMAC 2D: Goal position (%u, %u) is outside costmap bounds (%u, %u)",
      goal_x, goal_y, size_x_, size_y_);
    return false;
  }

  // Get start and goal nodes
  uint64_t start_index = Node2D::getIndex(start_x, start_y, size_x_);
  uint64_t goal_index = Node2D::getIndex(goal_x, goal_y, size_x_);
  
  RCLCPP_INFO(node_->get_logger(), 
    "SMAC 2D: Planning from (%u, %u) to (%u, %u), costmap size: %ux%u",
    start_x, start_y, goal_x, goal_y, size_x_, size_y_);
  
  // Check start and goal validity
  unsigned char start_cost = costmap_->getCost(start_x, start_y);
  unsigned char goal_cost = costmap_->getCost(goal_x, goal_y);
  
  if (start_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_ERROR(node_->get_logger(), "SMAC 2D: Start position is in collision!");
    return false;
  }
  
  if (goal_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && !allow_unknown_) {
    RCLCPP_ERROR(node_->get_logger(), "SMAC 2D: Goal position is in collision!");
    return false;
  }

  NodePtr start_node = getOrAddNode(start_index);
  NodePtr goal_node = getOrAddNode(goal_index);

  start_node->setAccumulatedCost(0.0);
  start_node->setPose(Coordinates(start_x, start_y));
  goal_node->setPose(Coordinates(goal_x, goal_y));

  // Priority queue for A*
  typedef std::pair<float, NodePtr> QueueElement;
  auto compare = [](const QueueElement & a, const QueueElement & b) {
      return a.first > b.first;
    };
  std::priority_queue<QueueElement, std::vector<QueueElement>, decltype(compare)> open_list(
    compare);

  // Add start to open list
  float start_h = computeHeuristic(start_node, goal_node);
  open_list.push({start_h, start_node});
  start_node->queued();

  int iterations = 0;
  Node2D::NodeVector neighbors;

  const uint64_t grid_size =
    static_cast<uint64_t>(size_x_) * static_cast<uint64_t>(size_y_);

  std::function<bool(const uint64_t &, NodePtr &)> neighbor_getter = 
    [this, grid_size](const uint64_t & index, NodePtr & neighbor_out) -> bool {
      if (index >= grid_size) {
        return false;
      }
      neighbor_out = getOrAddNode(index);
      return true;
    };

  while (!open_list.empty() && iterations < max_iterations_) {
    // Check for cancellation
    if (cancel_checker && iterations % 100 == 0 && cancel_checker()) {
      return false;
    }

    // Get best node
    auto current = open_list.top();
    open_list.pop();
    NodePtr current_node = current.second;

    // Skip if already visited
    if (current_node->wasVisited()) {
      continue;
    }

    current_node->visited();
    iterations++;
    
    // Check if goal reached
    if (current_node->getIndex() == goal_index) {
      return current_node->backtracePath(path);
    }

    // Expand neighbors
    neighbors.clear();
    current_node->getNeighbors(neighbor_getter, collision_checker_.get(), allow_unknown_, neighbors);

    for (auto * neighbor : neighbors) {
      float tentative_g = current_node->getAccumulatedCost() +
                          current_node->getTraversalCost(neighbor) +
                          getESDFPotentialCost(neighbor->getIndex());

      if (tentative_g < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(tentative_g);
        neighbor->parent = current_node;

        // Reinsert with updated priority (allow duplicates; visited() check filters them).
        float f = tentative_g + computeHeuristic(neighbor, goal_node);
        open_list.push({f, neighbor});
        neighbor->queued();
      }
    }
  }

  // Path not found
  if (iterations >= max_iterations_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SMAC 2D: Max iterations (%d) reached without finding path", max_iterations_);
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SMAC 2D: Open list exhausted after %d iterations, no path exists", iterations);
  }
  return false;
}

}  // namespace smac
}  // namespace minco_planner
