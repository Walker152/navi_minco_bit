#include "bt_manager/utils/nav_zone.hpp"
#include "bt_manager/ros_interface.hpp"

namespace Sentry_BT
{
  Area_Square tunnel_zone{Point2D{10.5, 5.0}, Point2D{8.0, 1.3}}; // 假设这是隧道区域的坐标范围
  Area_Square stairs_zone{Point2D{9.3, 2.0}, Point2D{7.6, 0.18}}; // 假设这是楼梯区域的坐标范围
}
  