#pragma once

#include "bt_manager/utils/nav_zone.hpp"

namespace Sentry_BT {
// for rmuc
inline Area_Square transform_zone{Point2D{10.5, 5.0}, Point2D{8.0, 1.3}};  // 假设这是隧道区域的坐标范围
inline std::array<Area_Square, 4> tunnel_zone = {
  Area_Square{Point2D{10.4, 3.6}, Point2D{9.3, 1.8}},
  Area_Square{Point2D{9.8, 3.6}, Point2D{9.3, 2.7}},
  Area_Square{Point2D{10.4, 2.7}, Point2D{9.8, 1.8}},
  Area_Square{Point2D{9.8, 2.7}, Point2D{9.3, 1.8}},
};
inline Area_Square stairs_zone{Point2D{9.4, 1.8}, Point2D{8.0, 0.2}};  // 假设这是楼梯区域的坐标范围
inline Area_Square stairs_lower_safe_zone{Point2D{9.4, 3.4}, Point2D{8.1, 2.2}};
inline Area_Square highland_zone{Point2D{12.0, 10.0}, Point2D{9.0, 7.2}};
inline Area_Square own_defense_zone{Point2D{10.6, 13.1}, Point2D{0.6, 2.0}};
inline Area_Square enemy_defense_zone{Point2D{28.9, 12.6}, Point2D{19.4, 4.7}};
inline Area_Square enemy_outpost_watch_zone{Point2D{16.7, 12.5}, Point2D{14.0, 9.6}};

inline Area_Square full_court_zone{Point2D{28.0, 15.0}, Point2D{0.0, 0.0}};

inline AreaPolygon<6, Point2D> own_highland_buff_zone{
  Point2D{12.9, 12.5, 0.0}, Point2D{12.9, 11.2, 0.0}, Point2D{11.7, 9.5, 0.0}, Point2D{11.7, 6.5, 0.0}, Point2D{10.7, 6.5, 0.0}, Point2D{10.7, 9.5, 0.0}};
inline AreaPolygon<4, Point2D> enemy_highland_buff_zone{
  Point2D{18.0, 9.8, 0.0}, Point2D{21.2, 9.8, 0.0}, Point2D{20.8, 7.0, 0.0}, Point2D{18.8, 7.0, 0.0}};

inline Area_Square own_base_buff_zone{Point2D{2.0, 13.8}, Point2D{5.2, 10.0}};
inline Area_Square enemy_base_buff_zone{Point2D{24.8, 13.8}, Point2D{28.0, 10.0}};
inline Area_Square own_outpost_buff_zone{Point2D{6.8, 9.0}, Point2D{9.8, 6.8}};
inline Area_Square enemy_outpost_buff_zone{Point2D{14.0, 9.8}, Point2D{17.0, 6.8}};

inline const std::unordered_map<TacticalMode, std::vector<Area_Square>> tracking_areas = {
  {TacticalMode::DEFENSIVE, {own_defense_zone, highland_zone}},
  {TacticalMode::BALANCED, {own_defense_zone, highland_zone}},
  {TacticalMode::OFFENSIVE, {own_defense_zone, highland_zone, enemy_defense_zone}},
};

// for test
// inline Area_Square transform_zone{Point2D{12.7, 7.3}, Point2D{9.5, 2.9}};  // 假设这是隧道区域的坐标范围
// inline Area_Square tunnel_zone{Point2D{12.5, 6.3}, Point2D{11.5, 4.8}};  // 假设这是隧道区域的坐标范围
// inline Area_Square stairs_zone{Point2D{11.6, 7.3}, Point2D{10.5, 6.0}};  // 假设这是楼梯区域的坐标范围
// inline Area_Square stairs_lower_safe_zone{Point2D{11.6, 6.0}, Point2D{10.5, 4.4}};
// inline Area_Square highland_zone{Point2D{11.0, 7.0}, Point2D{5.8, 6.4}};
// inline Area_Square own_defense_zone{Point2D{10.2, 4.3}, Point2D{5.6, 0.5}};
// inline Area_Square enemy_defense_zone{Point2D{15.6, 5.5}, Point2D{13.4, 0.5}};
// inline Area_Square enemy_outpost_watch_zone{Point2D{9.6, 7.0}, Point2D{7.6, 6.5}};
}  // namespace Sentry_BT
