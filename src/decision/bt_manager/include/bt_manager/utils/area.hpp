#pragma once
// #define RMUC_AREA
#define TEST_AREA
#include "bt_manager/utils/nav_zone.hpp"

#include <array>
#include <sstream>
#include <string>
#include <vector>
// clang-format off
namespace Sentry_BT {
#ifdef RMUC_AREA
// for rmuc
inline std::array<Area_Square, 4> transform_zone{
  Area_Square{Point2D{10.5, 5.0}, Point2D{8.0, 1.3}}, // Home Right Tunnel
  Area_Square{Point2D{21.0, 13.2}, Point2D{18.5, 10.0}}, // Enemy Right Tunnel
  Area_Square{Point2D{15.7, 13.8}, Point2D{10.4, 12.3}}, // Home Left Tunnel
  Area_Square{Point2D{18.6, 2.7}, Point2D{13.3, 1.2}}, // Enemy Left Tunnel
};
inline std::array<Area_Square, 2> bonus_zone = {
  Area_Square{Point2D{12.8, 5.5}, Point2D{13.8, 6.5}},
  Area_Square{Point2D{14.7, 11.0}, Point2D{15.7, 12.0}},
};  // 假设这是奖励区域的坐标范围
inline std::array<Area_Square, 4> tunnel_zone = {
  Area_Square{Point2D{10.4, 3.6}, Point2D{9.3, 1.8}},
  Area_Square{Point2D{14.1, 13.8}, Point2D{12.1, 13.0}},
  Area_Square{Point2D{19.7, 13.2}, Point2D{18.6, 11.4}},
  Area_Square{Point2D{16.9, 2.0}, Point2D{14.9, 1.2}},
};
// Per-tunnel recovery configuration, index-aligned with tunnel_zone.
inline std::array<TunnelRecoveryConfig, 4> tunnel_recovery_configs = {
  TunnelRecoveryConfig{-1.57f, Point2D{3.0, 3.0, 0.0}, Point2D{9.8, 2.8, 0.0}, 0.0f, 1.2f},
  TunnelRecoveryConfig{1.57f, Point2D{22.0, 7.5, 0.0}, Point2D{12.8, 13.3, 0.0}, 0.0f, 1.5f},
  TunnelRecoveryConfig{0.0f, Point2D{3.0, 3.0, 0.0}, Point2D{19.1, 12.3, 0.0}, 0.0f, 1.0f},
  TunnelRecoveryConfig{3.14f, Point2D{22.0, 7.5, 0.0}, Point2D{15.8, 1.6, 0.0}, 0.0f, 1.4f},
};
inline std::array<Area_Square, 2> stairs_zone{
  Area_Square{Point2D{9.4, 1.8}, Point2D{8.0, 0.2}},
  Area_Square{Point2D{19.6, 13.2}, Point2D{21.0, 14.8}},
};  // 假设这是楼梯区域的坐标范围
inline std::array<Area_Square, 2> stairs_lower_safe_zone{
  Area_Square{Point2D{9.4, 3.4}, Point2D{8.1, 2.2}},
  Area_Square{Point2D{19.6, 11.8}, Point2D{20.9, 12.8}},
};
inline AreaPolygon<8, Point2D> highland_zone{
  Point2D{13.2, 12.7}, 
  Point2D{10.9, 9.5},
  Point2D{10.9, 6.5},
  Point2D{13.5, 2.3},
  Point2D{15.8, 2.3},
  Point2D{18.1, 5.5},
  Point2D{18.1, 8.5},
  Point2D{15.5, 12.7},
};
inline AreaPolygon<8, Point2D> own_defense_zone{
  Point2D{0.3, 9.8}, 
  Point2D{8.3, 9.8},
  Point2D{11.0, 13.6},
  Point2D{12.9, 13.6},
  Point2D{9.6, 8.4},
  Point2D{9.6, 6.9},
  Point2D{10.6, 4.4},
  Point2D{0.3, 4.4},
};
inline AreaPolygon<8, Point2D> enemy_defense_zone{
  Point2D{28.7, 5.2}, 
  Point2D{20.7, 5.2},
  Point2D{18.0, 1.4},
  Point2D{16.1, 1.4},
  Point2D{19.4, 6.6},
  Point2D{19.4, 8.1},
  Point2D{18.4, 10.6},
  Point2D{28.7, 10.6},
};
inline Area_Square enemy_outpost_watch_zone{Point2D{16.7, 12.5}, Point2D{13.5, 10.1}};

inline AreaPolygon<6, Point2D> own_highland_buff_zone{
  Point2D{13.0, 12.3},
  Point2D{13.0, 11.1},
  Point2D{11.8, 9.5},
  Point2D{11.8, 6.5},
  Point2D{11.0, 6.5},
  Point2D{11.0, 9.5},
};
inline AreaPolygon<6, Point2D> enemy_highland_buff_zone{
  Point2D{16.0, 2.7},
  Point2D{16.0, 3.9},
  Point2D{17.2, 5.5},
  Point2D{17.2, 8.5},
  Point2D{18.0, 8.5},
  Point2D{18.0, 5.5},
};

inline AreaPolygon<6, Point2D> own_base_buff_zone{
  Point2D{2.9, 9.4}, 
  Point2D{4.4, 8.5},
  Point2D{4.4, 6.5},
  Point2D{2.9, 5.6},
  Point2D{1.6, 6.4},
  Point2D{1.6, 8.8},
};
inline AreaPolygon<6, Point2D> enemy_base_buff_zone{
  Point2D{26.1, 5.6},
  Point2D{24.6, 6.5},
  Point2D{24.6, 8.5},
  Point2D{26.1, 9.4},
  Point2D{27.4, 8.6},
  Point2D{27.4, 6.2},
};
inline AreaPolygon<6, Point2D> own_outpost_buff_zone{
  Point2D{10.9, 2.5},
  Point2D{11.7, 2.5},
  Point2D{12.2, 3.0},
  Point2D{12.2, 4.0},
  Point2D{11.7, 4.5},
  Point2D{10.9, 4.5},
};
inline AreaPolygon<6, Point2D> enemy_outpost_buff_zone{
  Point2D{18.1, 12.5},
  Point2D{17.3, 12.5},
  Point2D{16.8, 12.0},
  Point2D{16.8, 11.0},
  Point2D{17.3, 10.5},
  Point2D{18.1, 10.5},
};

inline std::vector<Point2D> nav_points = {
  {3.0, 3.0, 0.0},   // HOME
  {12.8, 5.5, 0.0},   // BONUS
  {15.7, 11.0, 0.0}, // OUTPOST
  {7.2, 7.5, 0.0},   // OWN_FORT
  {22.0, 7.5, 0.0}   // ENEMY_FORT

  // for rmul
  // {1.2, 7.2, 0.0},  //HOME
  // {6.4, 4.4, 0.0},  // BONUS
  // {7.5, 6.8, 0.0}  // OUTPOST
};

inline std::vector<PatrolPoint> patrol_points_normal = {
  // for rmuc
  {{16.0, 12.0, 0.0}, 5000},
  {{17.3, 7.9, 0.0}, 5000},
  {{15.3, 3.8, 0.0}, 6000},
};

inline std::vector<PatrolPoint> patrol_points_attack = {
  {{16.0, 12.0, 0.0}, 5000}, {{17.3, 7.9, 0.0}, 5000}, {{15.3, 3.8, 0.0}, 6000}
};
#endif
#ifdef TEST_AREA
// for test
inline std::array<Area_Square, 4> transform_zone{
  Area_Square{Point2D{12.6, 7.3}, Point2D{9.6, 2.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{9.6, 2.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{9.6, 2.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{9.6, 2.1}},
};
inline std::array<Area_Square, 2> bonus_zone = {
  Area_Square{Point2D{12.8, 5.5}, Point2D{13.8, 6.5}},
  Area_Square{Point2D{14.7, 11.0}, Point2D{15.7, 12.0}},
};  // 假设这是奖励区域的坐标范围
inline std::array<Area_Square, 4> tunnel_zone = {
  Area_Square{Point2D{12.6, 7.3}, Point2D{11.4, 4.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{11.4, 4.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{11.4, 4.1}},
  Area_Square{Point2D{12.6, 7.3}, Point2D{11.4, 4.1}},
};
// Per-tunnel recovery configuration, index-aligned with tunnel_zone.
inline std::array<TunnelRecoveryConfig, 4> tunnel_recovery_configs = {
  TunnelRecoveryConfig{1.57f, Point2D{6.4, 2.3, 0.0}, Point2D{11.8, 5.8, 0.0}, 0.0f, 1.0f},
  TunnelRecoveryConfig{1.57f, Point2D{13.7, 3.2, 0.0}, Point2D{11.8, 5.2, 0.0}, 0.0f, 1.2f},
  TunnelRecoveryConfig{1.57f, Point2D{6.4, 2.3, 0.0}, Point2D{11.8, 4.8, 0.0}, 0.0f, 1.4f},
  TunnelRecoveryConfig{1.57f, Point2D{13.7, 3.2, 0.0}, Point2D{11.8, 4.4, 0.0}, 0.0f, 1.1f},
};
inline std::array<Area_Square, 2> stairs_zone{
  Area_Square{Point2D{11.5, 7.1}, Point2D{10.3, 6.2}},
  Area_Square{Point2D{11.5, 7.1}, Point2D{10.3, 6.2}},
};  // 假设这是楼梯区域的坐标范围
inline std::array<Area_Square, 2> stairs_lower_safe_zone{
  Area_Square{Point2D{11.5, 6.2}, Point2D{10.3, 4.5}},
  Area_Square{Point2D{11.5, 6.2}, Point2D{10.3, 4.5}},
};
inline AreaPolygon<8, Point2D> highland_zone{
  Point2D{12.3, 7.3}, 
  Point2D{10.3, 7.3},
  Point2D{7.9, 7.3},
  Point2D{5.5, 7.3},
  Point2D{5.5, 6.2},
  Point2D{7.9, 6.2},
  Point2D{10.3, 6.2},
  Point2D{12.3, 6.2},
};
inline AreaPolygon<8, Point2D> own_defense_zone{
  Point2D{11.4, 4.5}, 
  Point2D{9.6, 4.5},
  Point2D{7.8, 4.5},
  Point2D{6.0, 4.5},
  Point2D{6.0, 0.5},
  Point2D{7.8, 0.5},
  Point2D{9.6, 0.5},
  Point2D{11.4, 0.5},
};
inline AreaPolygon<8, Point2D> enemy_defense_zone{
  Point2D{15.8, 0.7}, 
  Point2D{15.8, 3.1},
  Point2D{15.8, 4.5},
  Point2D{15.8, 5.7},
  Point2D{12.7, 5.7},
  Point2D{12.7, 4.5},
  Point2D{12.7, 3.1},
  Point2D{12.7, 0.7},
};
inline Area_Square enemy_outpost_watch_zone{Point2D{9.1, 7.2}, Point2D{7.3, 6.2}};

inline AreaPolygon<6, Point2D> own_highland_buff_zone{
  Point2D{9.1, 7.2},
  Point2D{9.1, 6.7},
  Point2D{9.1, 6.2},
  Point2D{7.3, 6.2},
  Point2D{7.3, 6.7},
  Point2D{7.3, 7.2},
};
inline AreaPolygon<6, Point2D> enemy_highland_buff_zone{
  Point2D{9.1, 7.2},
  Point2D{9.1, 6.7},
  Point2D{9.1, 6.2},
  Point2D{7.3, 6.2},
  Point2D{7.3, 6.7},
  Point2D{7.3, 7.2},
};

inline AreaPolygon<6, Point2D> own_base_buff_zone{
  Point2D{7.2, 4.3}, 
  Point2D{7.2, 2.7},
  Point2D{7.2, 1.2},
  Point2D{6.1, 1.2},
  Point2D{6.1, 2.7},
  Point2D{6.1, 4.3},
};
inline AreaPolygon<6, Point2D> enemy_base_buff_zone{
  Point2D{14.1, 5.9},
  Point2D{14.1, 4.8},
  Point2D{14.1, 3.8},
  Point2D{12.9, 3.8},
  Point2D{12.9, 4.8},
  Point2D{12.9, 5.9},
};
inline AreaPolygon<6, Point2D> own_outpost_buff_zone{
  Point2D{9.1, 7.2},
  Point2D{9.1, 6.7},
  Point2D{9.1, 6.2},
  Point2D{7.3, 6.2},
  Point2D{7.3, 6.7},
  Point2D{7.3, 7.2},
};
inline AreaPolygon<6, Point2D> enemy_outpost_buff_zone{
  Point2D{9.1, 7.2},
  Point2D{9.1, 6.7},
  Point2D{9.1, 6.2},
  Point2D{7.3, 6.2},
  Point2D{7.3, 6.7},
  Point2D{7.3, 7.2},
};

inline std::vector<Point2D> nav_points = {

  // for test
  {6.4, 2.3, 0.0},  // HOME
  {5.6, 3.8, 0.0},  // BONUS
  {10.6, 6.6, 0.0},  // OUTPOST
  {6.8, 3.5, 0.0},  // OWN_FORT
  {13.7, 3.2, 0.0}   // ENEMY_FORT
};

inline std::vector<PatrolPoint> patrol_points_normal = {
  // for test
  {{10.2, 6.8, 0.0}, 5000},
  {{12.6, 2.0, 0.0}, 5000},
  {{10.1, 2.6, 0.0}, 5000}
  };

inline std::vector<PatrolPoint> patrol_points_attack = {
  {{16.0, 12.0, 0.0}, 5000}, {{17.3, 7.9, 0.0}, 5000}, {{15.3, 3.8, 0.0}, 6000}
};
#endif
// =============== 战略模式：巡逻点与云台巡检区域映射表 ===============

using PatrolList = std::vector<PatrolPoint>;
using GimbalPatrolAreaList = std::vector<GimbalPatrolPoint>;

// 1. 底盘巡逻点映射表 (TacticalMode -> PatrolList)
inline std::unordered_map<TacticalMode, PatrolList> tactical_patrol_map = {
  {TacticalMode::OFFENSIVE, patrol_points_attack},
  {TacticalMode::DEFENSIVE, patrol_points_normal},
  {TacticalMode::BALANCED, patrol_points_normal}};

// 2. 云台巡检区域映射表 (TacticalMode -> GimbalPatrolAreaList)
inline std::unordered_map<TacticalMode, GimbalPatrolAreaList> tactical_gimbal_map = {
  {TacticalMode::OFFENSIVE,
    {
      {-180.0f, 180.0f, false},  // 直视前方扫射范围
      {-30.0f, 30.0f, true}      // 抬头重点区域
    }},
  {TacticalMode::DEFENSIVE,
    {
      {-180.0f, 180.0f, false},  // 360度全方位戒备
      {90.0f, 180.0f, true}      // 背后抬头观察高塔等
    }},
  {TacticalMode::BALANCED,
    {
      {-180.0f, 180.0f, false}  // 兼顾前后
    }}};

// clang-format on
inline const std::unordered_map<TacticalMode, std::vector<AreaPolygon<8, Point2D>>> tracking_areas = {
  {TacticalMode::DEFENSIVE, {own_defense_zone, highland_zone}},
  {TacticalMode::BALANCED, {own_defense_zone, highland_zone}},
  {TacticalMode::OFFENSIVE, {own_defense_zone, highland_zone, enemy_defense_zone}},
};

inline std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
inline std::vector<std::string> mode_names = {"PATROL", "TRACING", "RETREAT", "RESPONSE", "MANUAL"};
inline std::vector<std::string> stance_names = {"ATTACK", "DEFEND", "MOVE"};

inline void appendTrackingAreaVizConfigs(std::vector<PolygonVizConfig> & configs)
{
  for (const auto & mode_entry : tracking_areas) {
    const auto mode_id = static_cast<int>(mode_entry.first);
    for (std::size_t i = 0; i < mode_entry.second.size(); ++i) {
      std::ostringstream name;
      name << "_mode_" << mode_id << "_" << i;
      configs.push_back(makePolygonVizConfig(name.str(), mode_entry.second[i], {0.0F, 0.85F, 0.9F}));
    }
  }
}

inline std::vector<AreaVizConfig> getAreaVizConfigs()
{
  return {
    {"transform_zone", transform_zone[0], {1.0F, 0.2F, 0.2F}},
    {"transform_zone", transform_zone[1], {1.0F, 0.2F, 0.2F}},
    {"transform_zone", transform_zone[2], {1.0F, 0.2F, 0.2F}},
    {"transform_zone", transform_zone[3], {1.0F, 0.2F, 0.2F}},
    {"tunnel_zone", tunnel_zone[0], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[1], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[2], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[3], {1.0F, 0.6F, 0.0F}},
    {"stairs_zone", stairs_zone[0], {1.0F, 1.0F, 0.2F}},
    {"stairs_zone", stairs_zone[1], {1.0F, 1.0F, 0.2F}},
    {"stairs_lower_safe_zone", stairs_lower_safe_zone[0], {0.4F, 1.0F, 0.4F}},
    {"stairs_lower_safe_zone", stairs_lower_safe_zone[1], {0.4F, 1.0F, 0.4F}},
    {"enemy_outpost_watch_zone", enemy_outpost_watch_zone, {0.7F, 0.2F, 1.0F}},
  };
}

inline std::vector<PolygonVizConfig> getBasePolygonVizConfigs()
{
  return {
    makePolygonVizConfig("highland_zone", highland_zone, {0.2F, 0.6F, 1.0F}),
    makePolygonVizConfig("own_defense_zone", own_defense_zone, {0.6F, 0.4F, 1.0F}),
    makePolygonVizConfig("enemy_defense_zone", enemy_defense_zone, {1.0F, 0.2F, 1.0F}),
    makePolygonVizConfig("own_highland_buff_zone", own_highland_buff_zone, {0.3F, 0.9F, 0.3F}),
    makePolygonVizConfig("enemy_highland_buff_zone", enemy_highland_buff_zone, {1.0F, 0.4F, 0.2F}),
    makePolygonVizConfig("own_base_buff_zone", own_base_buff_zone, {0.3F, 0.9F, 0.3F}),
    makePolygonVizConfig("enemy_base_buff_zone", enemy_base_buff_zone, {0.9F, 0.3F, 0.3F}),
    makePolygonVizConfig("own_outpost_buff_zone", own_outpost_buff_zone, {0.5F, 0.9F, 0.3F}),
    makePolygonVizConfig("enemy_outpost_buff_zone", enemy_outpost_buff_zone, {0.9F, 0.5F, 0.3F}),
  };
}

inline std::vector<PolygonVizConfig> getTrackingPolygonVizConfigs()
{
  std::vector<PolygonVizConfig> configs;
  appendTrackingAreaVizConfigs(configs);
  return configs;
}

inline std::vector<PolygonVizConfig> getPolygonVizConfigs()
{
  auto configs = getBasePolygonVizConfigs();
  appendTrackingAreaVizConfigs(configs);
  return configs;
}
}  // namespace Sentry_BT
