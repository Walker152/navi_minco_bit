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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "smac_search/constants.hpp"
#include "smac_search/types.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rog_map/map_query_interface.hpp"

namespace minco_planner {
namespace smac {

/**
 * @class minco_planner::smac::SmacPlanner2DSimple
 * @brief A simplified SMAC 2D planner for integration with minco_planner
 */
class SmacPlanner2DSimple
{
public:
  struct Coordinates
  {
    Coordinates() = default;
    Coordinates(float x_in, float y_in) : x(x_in), y(y_in) {}

    float x{0.0f};
    float y{0.0f};
  };

  using CoordinateVector = std::vector<Coordinates>;

  struct NodeMin
  {
    uint64_t index{0u};
    float f_score{0.0f};
    bool operator>(const NodeMin & other) const { return f_score > other.f_score; }
  };

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
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Configure the planner with parameter prefix for namespacing
   * @param node Lifecycle node
   * @param costmap_ros Costmap ROS wrapper
   * @param param_prefix Prefix for parameters, e.g. "<plugin_name>." (can be empty)
   */
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const std::string & param_prefix);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);
  void setESDFQuery(const std::shared_ptr<rog_map::MapQueryInterface> & query);

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
  bool createPath(const unsigned int & start_x,
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
  void setParameters(bool allow_unknown, int max_iterations, float tolerance);

private:
  void ensureSearchBuffers();
  void logFailure(const std::string & reason,
    unsigned int start_x,
    unsigned int start_y,
    unsigned int goal_x,
    unsigned int goal_y,
    int iterations = -1) const;

  /**
   * @brief Compute ESDF-based potential cost for a grid cell index
   */
  float getESDFPotentialCost(unsigned int mx, unsigned int my);

  /**
   * @brief Evaluate inflation traversal factor from costmap cell cost
   */
  float evaluateInflationCost(unsigned char cell_cost);

  // Parameters
  bool allow_unknown_;
  int max_iterations_;
  float tolerance_;

  // Costmap
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> esdf_query_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  unsigned int size_x_;
  unsigned int size_y_;

  // Cached costmap metadata (updated in configure()/createPath() to avoid per-cell getters).
  double costmap_origin_x_{0.0};
  double costmap_origin_y_{0.0};
  double costmap_resolution_{0.0};

  // SoA search buffers (lazy reset via planning_id_)
  std::vector<float> g_score_;
  std::vector<int> parent_;
  std::vector<uint32_t> visited_;
  std::vector<uint32_t> closed_;
  uint32_t planning_id_{0u};
  uint64_t grid_size_{0u};

  // Optional distance-field biasing queried through MapQueryInterface.
  bool use_esdf_cost_{false};
  float esdf_weight_{1.0f};
  float esdf_decay_{0.5f};
  float esdf_max_cost_{5.0f};

  // Per-planning-iteration cache to avoid allocations in the search loop.
  std::vector<float> esdf_cost_cache_;
  std::vector<uint32_t> esdf_cost_cache_id_;

  // Search info
  SearchInfo search_info_;
  MotionModel motion_model_;

  // Node for logging
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace smac
}  // namespace minco_planner

#endif  // MINCO_PLANNER__SMAC_SEARCH__SMAC_PLANNER_2D_SIMPLE_HPP_
