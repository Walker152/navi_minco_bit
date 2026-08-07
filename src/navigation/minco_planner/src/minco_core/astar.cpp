#include "minco_core/astar.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

namespace minco_planner {

#define COST_UNKNOWN_ROS 255
#define COST_OBS 254
#define COST_OBS_ROS 253
#define COST_NEUTRAL 10
#define COST_FACTOR 3.0

Astar::Astar(unsigned int nx, unsigned int ny) : nx(nx), ny(ny), ns(nx * ny)
{
  potarr = new float[ns];
  pending = new bool[ns];
  gradx = new int[ns];
  grady = new int[ns];
  pathx = new float[ns];
  pathy = new float[ns];

  pb1 = new float[ns];
  pb2 = new float[ns];
  pb3 = new float[ns];

  curP = new int[ns];
  nextP = new int[ns];
  overP = new int[ns];

  costarr = NULL;
  start[0] = 0;
  start[1] = 0;
  goal[0] = 0;
  goal[1] = 0;
  npath = 0;
  allow_unknown = true;
}

Astar::~Astar()
{
  delete[] potarr;
  delete[] pending;
  delete[] gradx;
  delete[] grady;
  delete[] pathx;
  delete[] pathy;
  delete[] pb1;
  delete[] pb2;
  delete[] pb3;
  delete[] curP;
  delete[] nextP;
  delete[] overP;
}

void Astar::setCostmap(const unsigned char * costmap, bool /*isROS*/, bool allow_unknown)
{
  costarr = costmap;
  this->allow_unknown = allow_unknown;
}

void Astar::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  map_ = map;
}

void Astar::setStart(int x, int y)
{
  start[0] = x;
  start[1] = y;
}

void Astar::setGoal(int x, int y)
{
  goal[0] = x;
  goal[1] = y;
}

void Astar::setSize(int nx, int ny)
{
  if (this->nx == nx && this->ny == ny)
    return;

  this->nx = nx;
  this->ny = ny;
  this->ns = nx * ny;

  delete[] potarr;
  potarr = new float[ns];
  delete[] pending;
  pending = new bool[ns];
  delete[] gradx;
  gradx = new int[ns];
  delete[] grady;
  grady = new int[ns];
  delete[] pathx;
  pathx = new float[ns];
  delete[] pathy;
  pathy = new float[ns];

  delete[] pb1;
  pb1 = new float[ns];
  delete[] pb2;
  pb2 = new float[ns];
  delete[] pb3;
  pb3 = new float[ns];

  delete[] curP;
  curP = new int[ns];
  delete[] nextP;
  nextP = new int[ns];
  delete[] overP;
  overP = new int[ns];
}

void Astar::setupNavFn(bool /*keepit*/)
{
  npath = 0;
  for (int i = 0; i < ns; i++) {
    potarr[i] = std::numeric_limits<float>::max();
    gradx[i] = 0;
    grady[i] = 0;
    pending[i] = false;
  }

  curPe = 0;
  nextPe = 0;
  overPe = 0;

  int goal_idx = goal[1] * nx + goal[0];
  potarr[goal_idx] = 0.0;

  curP[curPe++] = goal_idx;
  pending[goal_idx] = true;
}

bool Astar::calcPath(int /*nplan*/)
{
  // Trace back
  npath = 0;
  int c = start[1] * nx + start[0];
  int goal_c = goal[1] * nx + goal[0];

  if (potarr[c] >= std::numeric_limits<float>::max()) {
    return false;
  }

  pathx[npath] = c % nx;
  pathy[npath] = c / nx;
  npath++;

  while (c != goal_c) {
    int min_c = c;
    float min_pot = potarr[c];

    // 8-connected gradient descent
    int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};

    for (int i = 0; i < 8; i++) {
      int nc = c + dy[i] * nx + dx[i];
      if (nc >= 0 && nc < ns && potarr[nc] < min_pot) {
        min_pot = potarr[nc];
        min_c = nc;
      }
    }

    if (min_c == c)
      break;

    if (npath >= ns - 1) {
      break;
    }

    c = min_c;
    pathx[npath] = c % nx;
    pathy[npath] = c / nx;
    npath++;
  }

  // Only append the goal if the line from the last path point is free of obstacles.
  if (npath < ns && (pathx[npath - 1] != goal[0] || pathy[npath - 1] != goal[1])) {
    int x0 = static_cast<int>(pathx[npath - 1]);
    int y0 = static_cast<int>(pathy[npath - 1]);
    int x1 = goal[0], y1 = goal[1];
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    bool line_clear = true;
    while (x0 != x1 || y0 != y1) {
      int idx = y0 * static_cast<int>(nx) + x0;
      if (idx >= 0 && idx < static_cast<int>(ns) && costarr[idx] >= COST_OBS_ROS &&
          !(allow_unknown && costarr[idx] == 255)) {
        line_clear = false;
        break;
      }
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
    if (line_clear) {
      pathx[npath] = goal[0];
      pathy[npath] = goal[1];
      npath++;
    }
  }

  return true;
}

bool Astar::propNavFnAstar(int cycles, std::function<bool()> cancelChecker)
{
  int ncycl = 0;
  int ncycl_check = 0;
  int start_idx = start[1] * nx + start[0];

  while (ncycl < cycles) {
    if (cancelChecker && ncycl_check > 1000) {
      ncycl_check = 0;
      if (cancelChecker()) {
        return false;
      }
    }
    ncycl_check++;

    if (curPe == 0 && nextPe == 0 && overPe == 0)
      return false;  // No more to propagate

    // Process current buffer
    for (int i = 0; i < curPe; i++) {
      int c = curP[i];
      pending[c] = false;

      if (c == start_idx)
        return false;  // Found start

      // Neighbors (8-connected)
      int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
      int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
      float dist[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

      for (int k = 0; k < 8; k++) {
        int nc = c + dy[k] * nx + dx[k];
        if (nc >= 0 && nc < ns) {
          // 增加距离权重：对角线移动代价更高
          float new_pot = potarr[c] + COST_NEUTRAL * dist[k] + costarr[nc] * COST_FACTOR * dist[k];

          // 检查障碍物：如果是 UNKNOWN (255) 且不允许 unknown，则跳过
          // 如果是 LETHAL (254) 或 INSCRIBED (253)，则跳过
          if (costarr[nc] >= COST_OBS_ROS && !(allow_unknown && costarr[nc] == 255))
            continue;

          if (new_pot < potarr[nc]) {
            potarr[nc] = new_pot;
            if (!pending[nc]) {
              nextP[nextPe++] = nc;
              pending[nc] = true;
            }
          }
        }
      }
    }

    curPe = 0;
    // Swap buffers
    int * tmp = curP;
    curP = nextP;
    nextP = tmp;
    int tmp_e = curPe;
    curPe = nextPe;
    nextPe = tmp_e;

    ncycl++;
  }
  return true;
}

}  // namespace minco_planner
