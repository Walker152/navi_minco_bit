#include "minco_core/astar.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>
#include <iostream>

namespace minco_planner
{

#define COST_UNKNOWN_ROS 255
#define COST_OBS 254
#define COST_OBS_ROS 253
#define COST_NEUTRAL 50
#define COST_FACTOR 0.8

Astar::Astar(unsigned int nx, unsigned int ny)
: nx(nx), ny(ny), ns(nx * ny)
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
  start = NULL;
  goal = NULL;
  npath = 0;
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

void Astar::setCostmap(const unsigned int * costmap)
{
  costarr = costmap;
}

void Astar::setStart(int * start)
{
  this->start = start;
}

void Astar::setGoal(int * goal)
{
  this->goal = goal;
}

void Astar::setSize(int nx, int ny)
{
  if (this->nx == nx && this->ny == ny) return;
  
  this->nx = nx;
  this->ny = ny;
  this->ns = nx * ny;
  
  delete[] potarr; potarr = new float[ns];
  delete[] pending; pending = new bool[ns];
  delete[] gradx; gradx = new int[ns];
  delete[] grady; grady = new int[ns];
  delete[] pathx; pathx = new float[ns];
  delete[] pathy; pathy = new float[ns];
  
  delete[] pb1; pb1 = new float[ns];
  delete[] pb2; pb2 = new float[ns];
  delete[] pb3; pb3 = new float[ns];
  
  delete[] curP; curP = new int[ns];
  delete[] nextP; nextP = new int[ns];
  delete[] overP; overP = new int[ns];
}

void Astar::setupNavFn(bool keepit)
{
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

bool Astar::calcPath(int nplan)
{
  setupNavFn();
  
  // Propagate
  int cycle = 0;
  while (propNavFnAstar(std::max(nx * ny / 20, nx + ny))) {
    cycle++;
    if (cycle > 10000) break; // Safety break
  }
  
  // Trace back
  int start_idx = start[1] * nx + start[0];
  if (potarr[start_idx] >= std::numeric_limits<float>::max()) {
    return false;
  }
  
  // Simple gradient descent for path
  int c = start_idx;
  npath = 0;
  pathx[npath] = c % nx;
  pathy[npath] = c / nx;
  npath++;
  
  while (c != goal[1] * nx + goal[0] && npath < ns) {
    int min_c = c;
    float min_pot = potarr[c];
    
    int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    
    for (int i = 0; i < 8; i++) {
      int nc = c + dy[i] * nx + dx[i];
      if (nc >= 0 && nc < ns && potarr[nc] < min_pot) {
        min_pot = potarr[nc];
        min_c = nc;
      }
    }
    
    if (min_c == c) break;
    c = min_c;
    pathx[npath] = c % nx;
    pathy[npath] = c / nx;
    npath++;
  }
  
  return true;
}

bool Astar::propNavFnAstar(int cycles)
{
  int ncycl = 0;
  int start_idx = start[1] * nx + start[0];
  
  while (ncycl < cycles) {
    if (curPe == 0 && nextPe == 0 && overPe == 0) return false; // No more to propagate
    
    // Process current buffer
    for (int i = 0; i < curPe; i++) {
      int c = curP[i];
      pending[c] = false;
      
      if (c == start_idx) return false; // Found start
      
      // Neighbors
      int dx[4] = {1, -1, 0, 0};
      int dy[4] = {0, 0, 1, -1};
      
      for (int k = 0; k < 4; k++) {
        int nc = c + dy[k] * nx + dx[k];
        if (nc >= 0 && nc < ns) {
          float new_pot = potarr[c] + COST_NEUTRAL + costarr[nc];
          if (costarr[nc] >= COST_OBS_ROS) continue; // Obstacle
          
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
    int * tmp = curP; curP = nextP; nextP = tmp;
    int tmp_e = curPe; curPe = nextPe; nextPe = tmp_e;
    
    ncycl++;
  }
  return true;
}

}  // namespace minco_planner
