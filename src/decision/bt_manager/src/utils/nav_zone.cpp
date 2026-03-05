#include "bt_manager/utils/nav_zone.hpp"

namespace Sentry_BT
{
  std::vector<Point2D> nav_points = {
      {3.0, 3.0},  //HOME
      {8.5, 8.5},  // BONUS
      //{-3, 5.5}
      {15.7, 11.0}  // OUTPOST
      //{0.0, 0.0}//test
      //{1.0, 3.2} //TEST  -
  };

  std::vector<Point2D> patrol_points_normal = {
    // for
      {14.2, 11.5},
      {17.5, 8.2},
      {16.0, 4.2},
      {12.4, 5.1},
      {12.0, 8.6},
      {14.0, 12.0}
  };

  std::vector<int> patrol_points_milliseconds = {
      10000,
      500,
      500,
      10000,
      500,
      12000,
  };

  std::vector<Point2D> patrol_points_attack = {
      {15.2, 11.5},
      // {12.8, -0.1},
      // {11.5, -3.8}
  };

  std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
  std::vector<std::string> mode_names = {"PATROL", "TRACING", "RETREAT", "RESPONSE"};
  std::vector<std::string> stance_names = {"MOVE", "ATTACK", "DEFEND"}; 
}  // namespace Sentry_BT