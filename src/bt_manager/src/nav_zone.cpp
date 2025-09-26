#include "nav_zone.hpp"

namespace Sentry_BT {
std::vector<Point2D> nav_points = {
    {0.0, 0.0},   // HOME
    {5.0, 5.0},   // BONUS
    {10.0, 10.0}  // OUTPOST
};

std::vector<Point2D> patrol_points_normal = {
    {0.0, 0.0},
    {2.0, 2.0},
    {3.0, 3.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0}
};

std::vector<Point2D> patrol_points_attack = {
    {6.0, 6.0},
    {7.0, 7.0},
    {8.0, 8.0}
};

std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
std::vector<std::string> mode_names = {"PATROL", "ATTACK", "RETREAT", "RESPONSE"};
}