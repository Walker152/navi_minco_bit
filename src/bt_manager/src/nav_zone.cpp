#include "nav_zone.hpp"

namespace Sentry_BT
{
  std::vector<Point2D> nav_points = {
      //{-1.8, -4.7},   // HOME
      {0.0, -6.0},  //HOME
      {5.5, 3.5},  // BONUS
      //{-3, 5.5}
      {10.3, 1.5}  // OUTPOST
      //{1.0, 3.2} //TEST  -
  };

  std::vector<Point2D> patrol_points_normal = {
      //{1.0, 3.2} //TEST  
      //{-2.5, -5.5},
      {2.5, -3.5},
      {10.8, 2.5},
      {11.0, -5.5},
      {13.5, 4.2}
      
      // {12.5, 1.0},
      // {12.4, -2.4},
      // {7.4, -1.0},
      // {8.4, 3.5}
      // {}
  };

  std::vector<int> patrol_points_milliseconds = {
      8000,
      8000,
      7000,
      4000

      // 8000,
      // 8000,
      // 8000
  };

  std::vector<Point2D> patrol_points_attack = {
      {11.2, 4.5},
      // {12.8, -0.1},
      // {11.5, -3.8}
  };

  std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
  std::vector<std::string> mode_names = {"PATROL", "ATTACK", "RETREAT", "RESPONSE"};
}  // namespace Sentry_BT