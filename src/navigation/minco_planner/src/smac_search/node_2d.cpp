// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include "smac_search/node_2d.hpp"

#include <limits>
#include <vector>

namespace minco_planner {
namespace smac {

// defining static member for all instance to share
std::vector<int> Node2D::_neighbors_grid_offsets;
float Node2D::cost_travel_multiplier = 2.0;
unsigned int Node2D::_size_x = 0;

Node2D::Node2D(const uint64_t index)
: parent(nullptr), _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()), _index(index), _in_collision(false)
{
}

Node2D::~Node2D()
{
  parent = nullptr;
}

void Node2D::reset()
{
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
}

bool Node2D::isNodeValid(const bool & traverse_unknown, GridCollisionChecker * collision_checker)
{
  // Already found, we can return the result
  if (!std::isnan(_cell_cost)) {
    return !_in_collision;
  }

  _in_collision = collision_checker->inCollision(this->getIndex(), traverse_unknown);
  _cell_cost = collision_checker->getCost();
  return !_in_collision;
}

float Node2D::getTraversalCost(const NodePtr & child)
{
  float normalized_cost = child->getCost() / 252.0;
  const Coordinates A = getCoords(child->getIndex());
  const Coordinates B = getCoords(this->getIndex());
  const float & dx = A.x - B.x;
  const float & dy = A.y - B.y;
  static float sqrt_2 = sqrt(2);

  // If a diagonal move, travel cost is sqrt(2) not 1.0.
  if ((dx * dx + dy * dy) > 1.05) {
    return sqrt_2 * (1.0 + cost_travel_multiplier * normalized_cost);
  }

  // Length = 1.0
  return 1.0 + cost_travel_multiplier * normalized_cost;
}

float Node2D::getHeuristicCost(const Coordinates & node_coords, const CoordinateVector & goals_coords)
{
  // Using Moore distance as it more accurately represents the distances
  // even a Van Neumann neighborhood robot can navigate.
  auto dx = goals_coords[0].x - node_coords.x;
  auto dy = goals_coords[0].y - node_coords.y;
  return std::sqrt(dx * dx + dy * dy);
}

void Node2D::initMotionModel(const MotionModel & motion_model,
  unsigned int & x_size_uint,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  if (motion_model != MotionModel::TWOD) {
    throw std::runtime_error("Invalid motion model for 2D node.");
  }

  int x_size = static_cast<int>(x_size_uint);
  cost_travel_multiplier = search_info.cost_penalty;
  _size_x = x_size_uint;
  _neighbors_grid_offsets = {-1, +1, -x_size, +x_size, -x_size - 1, -x_size + 1, +x_size - 1, +x_size + 1};
}

void Node2D::getNeighbors(
  std::function<bool(const uint64_t &, minco_planner::smac::Node2D *&)> & NeighborGetter,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  const uint64_t & iter,
  NodeVector & neighbors)
{
  uint64_t index;
  NodePtr neighbor;
  uint64_t node_i = this->getIndex();
  const Coordinates coord_parent = getCoords(this->getIndex());
  Coordinates child;

  for (unsigned int i = 0; i != _neighbors_grid_offsets.size(); ++i) {
    index = node_i + _neighbors_grid_offsets[i];

    // Check for wrap around conditions
    child = getCoords(index);
    if (fabs(coord_parent.x - child.x) > 1 || fabs(coord_parent.y - child.y) > 1) {
      continue;
    }

    if (NeighborGetter(index, neighbor)) {
      if (neighbor->isNodeValid(traverse_unknown, collision_checker) && !neighbor->wasVisited(iter)) {
        neighbors.push_back(neighbor);
      }
    }
  }
}

bool Node2D::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    path.push_back(Node2D::getCoords(current_node->getIndex()));
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(Node2D::getCoords(current_node->getIndex()));

  return true;
}

}  // namespace smac
}  // namespace minco_planner
