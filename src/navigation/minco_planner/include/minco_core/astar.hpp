#ifndef MINCO_PLANNER__ASTAR_HPP_
#define MINCO_PLANNER__ASTAR_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace minco_planner
{

class Astar
{
public:
  Astar(unsigned int nx, unsigned int ny);
  ~Astar();

  void setCostmap(const unsigned int * costmap);
  void setStart(int * start);
  void setGoal(int * goal);
  bool calcPath(int nplan);
  
  float * getPathX() { return pathx; }
  float * getPathY() { return pathy; }
  int getPathLen() { return npath; }

  void setSize(int nx, int ny);

private:
  void setupNavFn(bool keepit = false);
  bool propNavFnAstar(int cycles);
  void updateCell(int n);
  void updateCellAstar(int n);

  int nx, ny, ns;
  const unsigned int * costarr;
  float * potarr;
  bool * pending;
  int * current_width;
  int * next_width;
  int * gradx, * grady;
  float * pathx, * pathy;
  int npath;
  int * start;
  int * goal;
  
  float curT;
  float priInc;
  
  // Priority queue related
  float * pb1, * pb2, * pb3;
  int * curP, * nextP, * overP;
  int curPe, nextPe, overPe;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__ASTAR_HPP_
