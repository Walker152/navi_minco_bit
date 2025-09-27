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
    };
    
    struct Area_Square
    {
        Point2D top_left;
        Point2D bottom_right;

        bool contains(const Point2D & point) const
        {
            return (point.x >= top_left.x && point.x <= bottom_right.x &&
                    point.y >= top_left.y && point.y <= bottom_right.y);
        }
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

    typedef enum _RobotID{
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
    extern std::vector<Point2D> patrol_points_attack;
}