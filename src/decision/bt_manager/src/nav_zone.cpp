#include "nav_zone.hpp"

namespace Sentry_BT
{
  std::vector<Point2D> nav_points = {
      {0.0, -6.25},  //HOME
      {5.5, 3.5},  // BONUS
      //{-3, 5.5}
      {12.2, 0.65}  // OUTPOST
      //{0.0, 0.0}//test
      //{1.0, 3.2} //TEST  -
  };

  std::vector<Point2D> patrol_points_normal = {
      {13.2, -4.6},
      {13.9, -2.6},
      {13.9, 0.18},
      {12.7, 1.85},
      {14.0, 4.91},
      {19.2, 5.10},
      {14.1, 4.77},
      {8.9, -0.2},
      {10.7, -4.8}
      
  };

  std::vector<int> patrol_points_milliseconds = {
      10000,
      500,
      500,
      10000,
      500,
      12000,
      500,
      12000,
      500
  };

  std::vector<Point2D> patrol_points_attack = {
      {11.2, 4.5},
      // {12.8, -0.1},
      // {11.5, -3.8}
  };

  std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
  std::vector<std::string> mode_names = {"PATROL", "ATTACK", "RETREAT", "RESPONSE"};
  std::vector<std::string> position_names = {"MOVE", "ATTACK", "DEFEND"}; 
}  // namespace Sentry_BT