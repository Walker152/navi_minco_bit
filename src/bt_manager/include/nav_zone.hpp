#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace Sentry_BT
{
    struct Point2D
    {
        double x;
        double y;

        // Point2D& operator =(const Point2D& other)
        // {
        //     this->x = other.x;
        //     this->y = other.y;
        //     return *this;
        // };
    };
    
    typedef enum _NavStatus {
        IDLE = 0,
        RUNNING = 1,
        SUCCESS = 2,
        FAILURE = 3,
        PENDING = 4
    } NavStatus;

    typedef enum _NavMode{
        PATROL = 0,
        ATTACK = 1,
        RETREAT = 2,
        RESPONSE = 3,
    } NavMode;

    typedef enum _NavGoal{
        HOME = 0,
        BONUS = 1,
        OUTPOST = 2,
    } NavGoal;

    extern std::vector<std::string> current_nav_status;
    extern std::vector<std::string> mode_names;
    extern std::vector<Point2D> nav_points;
    extern std::vector<Point2D> patrol_points_normal;
    extern std::vector<Point2D> patrol_points_attack;
}