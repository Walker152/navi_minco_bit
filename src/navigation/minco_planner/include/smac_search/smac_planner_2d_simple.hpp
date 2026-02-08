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

#ifndef MINCO_PLANNER__SMAC_SEARCH__SMAC_PLANNER_2D_SIMPLE_HPP_
#define MINCO_PLANNER__SMAC_SEARCH__SMAC_PLANNER_2D_SIMPLE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <functional>

#include "smac_search/node_2d.hpp"
#include "smac_search/collision_checker.hpp"
#include "smac_search/constants.hpp"
#include "smac_search/types.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace minco_planner
{
namespace smac
{

/**
 * @class minco_planner::smac::SmacPlanner2DSimple
 * @brief A simplified SMAC 2D planner for integration with minco_planner
 */
class SmacPlanner2DSimple
{
public:
  typedef Node2D::NodePtr NodePtr;
  typedef Node2D::Coordinates Coordinates;
  typedef Node2D::CoordinateVector CoordinateVector;

  /**
   * @brief Constructor
   */
  SmacPlanner2DSimple();

  /**
   * @brief Destructor
   */
  ~SmacPlanner2DSimple();

  /**
   * @brief Configure the planner
   * @param node Lifecycle node
   * @param costmap_ros Costmap ROS wrapper
   */
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Set the costmap directly (alternative to costmap_ros)
   * @param costmap Costmap2D pointer
   */
  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  /**
   * @brief Create a path from start to goal
   * @param start_x Start X in map coordinates
   * @param start_y Start Y in map coordinates
   * @param goal_x Goal X in map coordinates
   * @param goal_y Goal Y in map coordinates
   * @param path Output path coordinates
   * @param cancel_checker Function to check if planning should be canceled
   * @return true if path found
   */
  bool createPath(
    const unsigned int & start_x,
    const unsigned int & start_y,
    const unsigned int & goal_x,
    const unsigned int & goal_y,
    CoordinateVector & path,
    std::function<bool()> cancel_checker = nullptr);

  /**
   * @brief Set parameters
   * @param allow_unknown If we allow traversing unknown space
   * @param max_iterations Maximum iterations
   * @param tolerance Tolerance for goal reaching
   */
  void setParameters(
    bool allow_unknown,
    int max_iterations,
    float tolerance);

private:
  /**
   * @brief Initialize graph for search
   */
  void initializeGraph();

  /**
   * @brief Get or add node to graph
   * @param index Node index
   * @return Node pointer
   */
  NodePtr getOrAddNode(const uint64_t & index);

  /**
   * @brief Compute heuristic cost
   * @param node Current node
   * @param goal Goal node
   * @return Heuristic cost
   */
  float computeHeuristic(const NodePtr & node, const NodePtr & goal);

  /**
   * @brief Clear all data structures for new search
   */
  void clear();

  // Parameters
  bool allow_unknown_;
  int max_iterations_;
  float tolerance_;
  
  // Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  unsigned int size_x_;
  unsigned int size_y_;

  // Collision checker
  std::unique_ptr<GridCollisionChecker> collision_checker_;

  // Graph
  std::vector<std::unique_ptr<Node2D>> graph_;
  
  // Search info
  SearchInfo search_info_;
  MotionModel motion_model_;

  // Node for logging
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace smac
}  // namespace minco_planner

#endif  // MINCO_PLANNER__SMAC_SEARCH__SMAC_PLANNER_2D_SIMPLE_HPP_
