#ifndef MINCO_PLANNER__ASTAR_HPP_
#define MINCO_PLANNER__ASTAR_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "rog_map/map_query_interface.hpp"

namespace minco_planner {

class Astar
{
public:
  Astar(unsigned int nx, unsigned int ny);
  ~Astar();

  void setCostmap(const unsigned char * costmap, bool isROS = true, bool allow_unknown = true);
  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);
  void setStart(int x, int y);
  void setGoal(int x, int y);
  bool calcPath(int nplan);

  void setupNavFn(bool keepit = false);
  bool propNavFnAstar(int cycles, std::function<bool()> cancelChecker = nullptr);

  float * getPathX() { return pathx; }
  float * getPathY() { return pathy; }
  int getPathLen() { return npath; }

  void setSize(int nx, int ny);

private:
  void updateCell(int n);
  void updateCellAstar(int n);

  int nx, ny, ns;
  const unsigned char * costarr;
  float * potarr;
  bool * pending;
  int * current_width;
  int * next_width;
  int *gradx, *grady;
  float *pathx, *pathy;
  int npath;
  int start[2];
  int goal[2];
  bool allow_unknown;
  std::shared_ptr<rog_map::MapQueryInterface> map_;

  float curT;
  float priInc;

  // Priority queue related
  float *pb1, *pb2, *pb3;
  int *curP, *nextP, *overP;
  int curPe, nextPe, overPe;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__ASTAR_HPP_
