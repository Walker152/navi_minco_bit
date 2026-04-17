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

#ifndef MINCO_PLANNER__SMAC_SEARCH__COLLISION_CHECKER_HPP_
#define MINCO_PLANNER__SMAC_SEARCH__COLLISION_CHECKER_HPP_

#include <memory>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "smac_search/constants.hpp"

namespace minco_planner {
namespace smac {

/**
 * @class minco_planner::smac::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for minco_planner::smac::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * @param node Node to extract clock and logger from
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    unsigned int num_quantizations,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(const nav2_costmap_2d::Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(const float & x, const float & y, const float & theta, const bool & traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(const unsigned int & i, const bool & traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Set the footprint from the footprint topic
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::vector<nav2_costmap_2d::Footprint> oriented_footprints_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  float center_cost_;
  bool footprint_is_radius_{false};
  std::vector<float> angles_;
  float possible_collision_cost_{-1};
  rclcpp::Logger logger_{rclcpp::get_logger("SmacPlannerCollisionChecker")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace smac
}  // namespace minco_planner

#endif  // MINCO_PLANNER__SMAC_SEARCH__COLLISION_CHECKER_HPP_
