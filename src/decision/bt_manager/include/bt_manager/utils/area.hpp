#pragma once

#include "bt_manager/utils/nav_zone.hpp"

namespace Sentry_BT {
inline Area_Square transform_zone{Point2D{10.5, 5.0}, Point2D{8.0, 1.3}};  // 假设这是隧道区域的坐标范围
inline Area_Square tunnel_zone{Point2D{10.4, 3.6}, Point2D{9.3, 1.8}};  // 假设这是隧道区域的坐标范围
inline Area_Square stairs_zone{Point2D{9.4, 1.8}, Point2D{8.0, 0.2}};  // 假设这是楼梯区域的坐标范围
inline Area_Square stairs_lower_safe_zone{Point2D{9.4, 3.4}, Point2D{8.1, 2.2}};
inline Area_Square target_feasible_zone{Point2D{16.5, 12.0}, Point2D{2.0, 1.0}};
inline Area_Square highland_zone{Point2D{12.0, 10.0}, Point2D{9.0, 7.2}};
inline Area_Square own_defense_zone{Point2D{10.6, 13.1}, Point2D{0.6, 2.0}};
inline Area_Square enemy_defense_zone{Point2D{28.9, 12.6}, Point2D{19.4, 4.7}};
inline Area_Square enemy_outpost_watch_zone{Point2D{16.7, 12.5}, Point2D{14.0, 9.6}};
}  // namespace Sentry_BT
