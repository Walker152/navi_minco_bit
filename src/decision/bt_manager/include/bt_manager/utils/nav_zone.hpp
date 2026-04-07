#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

//ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
namespace Sentry_BT
{
  struct Point2D
  {
    double x;
    double y;
    double yaw;
    Point2D() : x(0.0), y(0.0), yaw(0.0) {}
    Point2D(double x_, double y_, double yaw_ = 0.0) : x(x_), y(y_), yaw(yaw_) {}
  };
  struct Area_Square
  {
    Point2D top_left;
    Point2D bottom_right;

    Area_Square() : top_left(), bottom_right() {}
    Area_Square(const Point2D& top_left_, const Point2D& bottom_right_) : top_left(top_left_), bottom_right(bottom_right_) {}

    bool contains(const Point2D& point) const
    {
      double min_x = std::min(top_left.x, bottom_right.x);
      double max_x = std::max(top_left.x, bottom_right.x);
      double min_y = std::min(top_left.y, bottom_right.y);
      double max_y = std::max(top_left.y, bottom_right.y);
      return (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y);
    }
  };

  struct Area_Circle
  {
    Point2D center;
    double radius;

    Area_Circle() : center(), radius(0.0) {}
    Area_Circle(const Point2D& center_, double radius_) : center(center_), radius(radius_) {}
    
    bool contains(const Point2D& point) const
    {
      double dx = point.x - center.x;
      double dy = point.y - center.y;
      return (dx * dx + dy * dy) <= (radius * radius);
    }
  };

  struct PatrolPoint
  {
    Point2D position;
    int duration_ms; // 在该巡逻点停留的时间，单位为毫秒

    PatrolPoint() : position(), duration_ms(0) {}
    PatrolPoint(const Point2D& position_, int duration_ms_) : position(position_), duration_ms(duration_ms_) {}
  };
  
  typedef enum _TacticalMode
  {
    OFFENSIVE = 0,
    DEFENSIVE = 1,
    BALANCED = 2,
  } TacticalMode;

  typedef enum _NavMode
  {
    PATROL = 0,
    TRACING = 1,
    RETREAT = 2,
    RESPONSE = 3,
  } NavMode;

  typedef enum _NavGoal
  {
    HOME = 0,
    BONUS = 1,
    OUTPOST = 2,
  } NavGoal;

  typedef enum _SentryStance
  {
    MOVE = 0,
    ATTACK = 1,
    DEFEND = 2
  } SentryStance;

  typedef enum _RobotID
  {
    Hero = 1,
    Engineer = 2,
    Infantry1 = 3,
    Infantry2 = 4,
    Sentry = 5,
  } RobotID;

  inline std::vector<Point2D> nav_points = {
      {3.0, 3.0, 0.0},  //HOME
      {8.5, 8.5, 0.0},  // BONUS
      {15.7, 11.0, 0.0}  // OUTPOST

      // for test
      // {1.8, 5.6, 0.0},  //HOME
      // {5.6, 3.8, 0.0},  //BONUS
      // {7.2, 6.0, 0.0}   //OUTPOST

    // for rmul
      // {1.2, 7.2, 0.0},  //HOME
      // {6.4, 4.4, 0.0},  // BONUS
      // {7.5, 6.8, 0.0}  // OUTPOST
  };

  inline std::vector<PatrolPoint> patrol_points_normal = {
    // for rmuc
      {{14.2, 11.5, 0.0}, 5000},
      {{12.8, -0.1, 0.0}, 5000},
      {{11.5, -3.8, 0.0}, 6000},
      // {{12.4, 5.1, 0.0}, 5000},
      // {{12.0, 8.6, 0.0}, 5000},
      // {{14.0, 12.0, 0.0}, 6000}
    // for test
      // {{5.6, 6.0, 0.0}, 5000},
      // {{4.5, 5.1, 0.0}, 5000},
      // {{5.5, 3.5, 0.0}, 5000}

    // for rmul
     // {{7.2, 5.3, 0.0}, 5000},
      // {{6.4, 4.4, 0.0}, 5000},
      // {{5.5, 5.3, 0.0}, 5000}
      // {{7.3, 4.5, 0.0}, 5000}
  };

    inline std::vector<PatrolPoint> patrol_points_attack = {
      {{15.2, 11.5, 0.0}, 5000},
      {{12.8, -0.1, 0.0}, 5000},
      {{11.5, -3.8, 0.0}, 6000}

    // for test
    // {5.6, 6.0, 0.0},
    // {4.5, 5.1, 0.0},
    // {5.5, 3.5, 0.0}

    // for rmul
      // {7.2, 5.3, 0.0},
      // {6.4, 4.4, 0.0},
      // {5.5, 5.3, 0.0}
      // {7.3, 4.5, 0.0}
  };

  inline std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
  inline std::vector<std::string> mode_names = {"PATROL", "TRACING", "RETREAT", "RESPONSE"};
  inline std::vector<std::string> stance_names = {"MOVE", "ATTACK", "DEFEND"};

  struct AllyRobotInfo {
    int robot_id;  // 机器人ID
    int remain_hp;  // 剩余血量
    geometry_msgs::msg::Point position;  // 位置
  };

  struct EnemyRobotInfo {
    int robot_id;           // 机器人ID
    int remain_hp;          // 剩余血量
    int allowed_projectile; // 可打弹丸数
    geometry_msgs::msg::Point position;  // 位置
  };
}  // namespace Sentry_BT