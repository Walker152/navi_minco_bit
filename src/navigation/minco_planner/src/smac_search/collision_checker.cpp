// Copyright (c) 2021, Samsung Research America
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

#include "smac_search/collision_checker.hpp"

namespace minco_planner {
namespace smac {

GridCollisionChecker::GridCollisionChecker(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint, const bool & radius, const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(logger_,
      *clock_,
      1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint new_footprint;
    new_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      new_footprint.push_back(new_pt);
    }

    oriented_footprints_.push_back(new_footprint);
  }

  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(
  const float & x, const float & y, const float & angle_bin, const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) || outsideRange(costmap_->getSizeInCellsY(), y)) {
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(
    costmap_->getCost(static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    // if possible inscribed, need to check actual footprint
    if (center_cost_ > possible_collision_cost_) {
      if (angle_bin >= oriented_footprints_.size()) {
        throw std::runtime_error("Angle bin is out of bounds. Check angle quantizations.");
      }
      return footprintCost(oriented_footprints_[static_cast<unsigned int>(angle_bin)]) >=
             nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    // If low enough cost, no need to check footprint
    if (!traverse_unknown) {
      return center_cost_ >= nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    // If traversing unknown, need to check actual footprint for unknown costs
    return center_cost_ == UNKNOWN_COST &&
           footprintCost(oriented_footprints_[static_cast<unsigned int>(angle_bin)]) >=
             nav2_costmap_2d::LETHAL_OBSTACLE;
  } else {
    // If using a radius footprint model
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= nav2_costmap_2d::LETHAL_OBSTACLE;
  }
}

bool GridCollisionChecker::inCollision(const unsigned int & i, const bool & traverse_unknown)
{
  // Check if index is within bounds
  if (i >= costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY()) {
    // Index out of bounds, consider as collision
    center_cost_ = nav2_costmap_2d::LETHAL_OBSTACLE;
    return true;
  }

  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  // 允许在膨胀层内规划以逃离死锁，仅拦截绝对障碍物 (254)
  return center_cost_ >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace smac
}  // namespace minco_planner
