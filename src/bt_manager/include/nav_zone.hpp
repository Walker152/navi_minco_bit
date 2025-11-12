#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace Sentry_BT
{
  struct Point2D
  {
    double x;
    double y;
  };

  struct Area_Square
  {
    Point2D top_left;
    Point2D bottom_right;

    bool contains(const Point2D& point) const
    {
      double min_x = std::min(top_left.x, bottom_right.x);
      double max_x = std::max(top_left.x, bottom_right.x);
      double min_y = std::min(top_left.y, bottom_right.y);
      double max_y = std::max(top_left.y, bottom_right.y);
      return (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y);
    }
  };

  typedef enum _NavStatus
  {
    IDLE = 0,
    RUNNING = 1,
    SUCCESS = 2,
    FAILURE = 3,
    PENDING = 4
  } NavStatus;

  typedef enum _NavMode
  {
    PATROL = 0,
    ATTACK = 1,
    RETREAT = 2,
    RESPONSE = 3,
  } NavMode;

  typedef enum _NavGoal
  {
    HOME = 0,
    BONUS = 1,
    OUTPOST = 2,
  } NavGoal;

  typedef enum _RobotID
  {
    Hero = 1,
    Engineer = 2,
    Infantry1 = 3,
    Infantry2 = 4,
    Sentry = 5,
  } RobotID;

  extern std::vector<std::string> current_nav_status;
  extern std::vector<std::string> mode_names;
  extern std::vector<Point2D> nav_points;
  extern std::vector<Point2D> patrol_points_normal;
  extern std::vector<int> patrol_points_milliseconds;
  extern std::vector<Point2D> patrol_points_attack;
}  // namespace Sentry_BT