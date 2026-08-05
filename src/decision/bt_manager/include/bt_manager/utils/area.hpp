#pragma once
#define RMUC_AREA
// #define TEST_AREA
#include "bt_manager/utils/nav_zone.hpp"

#include <array>
#include <sstream>
#include <string>
#include <vector>
// clang-format off
namespace Sentry_BT {
#ifdef RMUC_AREA
// for rmuc
inline std::array<AreaPolygon<6, Point2D>, 4> transform_zone{
  AreaPolygon<6, Point2D>{
    Point2D{8.5, 5.0}, Point2D{10.0, 5.0}, Point2D{11.0, 5.0},
    Point2D{11.0, 0.1}, Point2D{10.0, 0.1}, Point2D{8.5, 0.1}}, // Home Right Tunnel 
  AreaPolygon<6, Point2D>{
    Point2D{18.0, 15.0}, Point2D{18.75, 15.0}, Point2D{20.5, 15.0},
    Point2D{20.5, 10.0}, Point2D{18.75, 10.0}, Point2D{18.0, 10.0}}, // Enemy Right Tunnel 
  AreaPolygon<6, Point2D>{
    Point2D{9.5, 12.0}, Point2D{11.0, 14.2}, Point2D{17.0, 14.2},
    Point2D{17.0, 12.5}, Point2D{12.7, 12.5}, Point2D{12.3, 12.0}}, // Home Left Tunnel 
  AreaPolygon<6, Point2D>{
    Point2D{19.5, 3.0}, Point2D{18.0, 0.8}, Point2D{12.0, 0.8},
    Point2D{12.0, 2.5}, Point2D{16.3, 2.5}, Point2D{16.7, 3.0}}, // Enemy Left Tunnel 
};
inline std::array<Area_Square, 2> bonus_zone = {
  Area_Square{Point2D{12.8, 5.5}, Point2D{13.8, 6.5}},
  Area_Square{Point2D{14.7, 11.0}, Point2D{15.7, 12.0}},
};  // 假设这是奖励区域的坐标范围
inline std::array<Area_Square, 4> tunnel_zone = {
  Area_Square{Point2D{10.3, 3.6}, Point2D{9.4, 1.7}},     // Home Right Tunnel
  Area_Square{Point2D{19.7, 13.3}, Point2D{18.8, 11.4}},  // Enemy Right Tunnel
  Area_Square{Point2D{15.4, 14.0}, Point2D{12.1, 13.0}},  // Home Left Tunnel
  Area_Square{Point2D{17.2, 2.0}, Point2D{13.6, 1.0}},    // Enemy Left Tunnel
};
// Per-tunnel alignment and recovery velocity configuration, index-aligned with tunnel_zone.
inline std::array<TunnelRecoveryConfig, 4> tunnel_recovery_configs = {
  TunnelRecoveryConfig{-1.57f, 0.0f, 0.5f, false},   // Home Right Tunnel
  TunnelRecoveryConfig{1.57f, 0.0f, -0.5f, false},   // Enemy Right Tunnel
  TunnelRecoveryConfig{0.0f, -0.5f, 0.0f, true},    // Home Left Tunnel
  TunnelRecoveryConfig{3.14f, 0.5f, 0.0f, true},    // Enemy Left Tunnel
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
  Point2D{13.2, 12.8}, 
  Point2D{10.7, 9.8},
  Point2D{10.7, 4.8},
  Point2D{13.5, 2.2},
  Point2D{15.8, 2.2},
  Point2D{18.3, 5.2},
  Point2D{18.3, 10.2},
  Point2D{15.5, 12.8},
};
inline AreaPolygon<8, Point2D> own_defense_zone{
  Point2D{0.3, 10.1}, 
  Point2D{8.3, 10.1},
  Point2D{11.0, 13.8},
  Point2D{13.5, 13.8},
  Point2D{9.7, 8.4},
  Point2D{9.7, 6.9},
  Point2D{10.8, 4.1},
  Point2D{0.3, 4.1},
};
inline AreaPolygon<8, Point2D> enemy_defense_zone{
  Point2D{28.7, 4.9}, 
  Point2D{20.7, 4.9},
  Point2D{18.0, 1.2},
  Point2D{15.5, 1.2},
  Point2D{19.3, 6.6},
  Point2D{19.3, 8.1},
  Point2D{18.2, 10.9},
  Point2D{28.7, 10.9},
};
inline Area_Square enemy_outpost_watch_zone{Point2D{16.7, 12.5}, Point2D{13.5, 10.1}};

inline AreaPolygon<8, Point2D> engineering_zone{
  Point2D{16.7, 7.8},
  Point2D{16.7, 6.3},
  Point2D{15.7, 5.4},
  Point2D{14.3, 5.4},
  Point2D{12.5, 7.2},
  Point2D{12.5, 8.7},
  Point2D{13.5, 9.6},
  Point2D{14.7, 9.6},
};
inline Area_Circle enemy_fort_zone{Point2D{22.0, 7.5}, 1.0};
inline Area_Circle enemy_fort_engage_zone{Point2D{22.0, 7.5}, 0.4};

inline AreaPolygon<6, Point2D> own_highland_buff_zone{
  Point2D{13.0, 12.5},
  Point2D{13.0, 11.1},
  Point2D{11.8, 9.5},
  Point2D{11.8, 6.5},
  Point2D{10.8, 6.5},
  Point2D{10.8, 9.7},
};
inline AreaPolygon<6, Point2D> enemy_highland_buff_zone{
  Point2D{16.0, 2.5},
  Point2D{16.0, 3.9},
  Point2D{17.2, 5.5},
  Point2D{17.2, 8.5},
  Point2D{18.2, 8.5},
  Point2D{18.2, 5.3},
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
inline Area_Square own_supply_zone{Point2D{3.8, 4.4}, Point2D{1.5, 0.0}};
inline Area_Square enemy_supply_zone{Point2D{27.5, 15.0}, Point2D{25.2, 10.6}};
inline AreaPolygon<6, Point2D> own_outpost_buff_zone{
  Point2D{11.1, 2.5},
  Point2D{11.9, 2.5},
  Point2D{12.4, 3.0},
  Point2D{12.4, 4.0},
  Point2D{11.9, 4.5},
  Point2D{11.1, 4.5},
};
inline AreaPolygon<6, Point2D> enemy_outpost_buff_zone{
  Point2D{17.9, 12.5},
  Point2D{17.1, 12.5},
  Point2D{16.6, 12.0},
  Point2D{16.6, 11.0},
  Point2D{17.1, 10.5},
  Point2D{17.9, 10.5},
};

inline std::vector<Point2D> nav_points = {
  {3.0, 2.6, 0.0},   // HOME
  {12.8, 5.5, 0.0},  // BONUS
  {15.2, 11.2, 0.0}, // ENEMY_OUTPOST
  {7.2, 7.5, 0.0},   // OWN_FORT
  {22.0, 7.5, 0.0},  // ENEMY_FORT
  {12.1, 3.9, 0.0},  // OWN_OUTPOST
  {8.8, 13.6, 0.0}  // HERO_GUARD

  // for rmul
  // {1.2, 7.2, 0.0},  //HOME
  // {6.4, 4.4, 0.0},  // BONUS
  // {7.5, 6.8, 0.0}  // OUTPOST
};

using PatrolList = std::vector<PatrolPoint>; 
inline std::vector<PatrolList> normal_patrol_branches = {
    {  
        // {{20.2, 14.0, 0.0}, 5000},  // 雷霆大坐点位
        {{11.0, 12.2, 0.0}, 5000},
        {{9.4, 4.7, 0.0}, 5000},
    },
    {
        {{20.5, 7.3, 0.0}, 5000},
        {{24.0, 7.8, 0.0}, 5000},
        {{20.4, 8.0, 0.0}, 5000} // 冲家巡逻点
    },
    {
        {{16.1, 10.4, 0.0}, 5000},
        {{17.2, 8.3, 0.0}, 5000},
        // {{12.0, 10.4, 0.0}, 5000} // 高地巡逻点
    },
    {
        {{16.1, 10.4, 0.0}, 5000},
        {{17.2, 8.3, 0.0}, 5000},
        {{20.5, 7.3, 0.0}, 5000},
        {{24.0, 7.8, 0.0}, 5000}
        // {{12.0, 10.4, 0.0}, 5000} // 测试巡逻点
    }
    //可继续加
};

inline std::vector<PatrolList> attack_patrol_branches = {
    {   
        {{20.2, 14.0, 0.0}, 5000}
    },
    { 
        {{21.0, 9.7, 0.0}, 5000},
        {{24.0, 7.8, 0.0}, 5000},
        {{20.4, 6.0, 0.0}, 5000}
    },
    {
        {{16.1, 10.4, 0.0}, 5000},
        {{17.2, 8.3, 0.0}, 5000},
        // {{12.0, 10.4, 0.0}, 5000}
    },
    {
        {{16.1, 10.4, 0.0}, 5000},
        {{17.2, 8.3, 0.0}, 5000},
        {{20.5, 7.3, 0.0}, 5000},
        {{24.0, 7.8, 0.0}, 5000}
        // {{12.0, 10.4, 0.0}, 5000} // 测试巡逻点
    }
    //可继续加
};
#endif
#ifdef TEST_AREA
// for test
inline std::array<AreaPolygon<6, Point2D>, 4> transform_zone{
  AreaPolygon<6, Point2D>{
    Point2D{10.6, 7.3}, Point2D{11.1, 7.3}, Point2D{12.6, 7.3},
    Point2D{12.6, 3.0}, Point2D{11.1, 3.0}, Point2D{10.6, 3.0}}, // TODO: replace with measured 6-point polygon vertices
  AreaPolygon<6, Point2D>{
    Point2D{10.6, 7.3}, Point2D{11.1, 7.3}, Point2D{12.6, 7.3},
    Point2D{12.6, 3.0}, Point2D{11.1, 3.0}, Point2D{10.6, 3.0}}, // TODO: replace with measured 6-point polygon vertices
  AreaPolygon<6, Point2D>{
    Point2D{10.6, 7.3}, Point2D{11.1, 7.3}, Point2D{12.6, 7.3},
    Point2D{12.6, 3.0}, Point2D{11.1, 3.0}, Point2D{10.6, 3.0}}, // TODO: replace with measured 6-point polygon vertices
  AreaPolygon<6, Point2D>{
    Point2D{8.5, 4.0}, Point2D{5.7, 4.0}, Point2D{2.4, 4.0},
    Point2D{2.4, 1.4}, Point2D{5.7,1.4}, Point2D{8.5, 1.4}}, // TODO: replace with measured 6-point polygon vertices
};
inline std::array<Area_Square, 2> bonus_zone = {
  Area_Square{Point2D{12.8, 5.5}, Point2D{13.8, 6.5}},
  Area_Square{Point2D{14.7, 11.0}, Point2D{15.7, 12.0}},
};  // 假设这是奖励区域的坐标范围
inline std::array<Area_Square, 4> tunnel_zone = {
  Area_Square{Point2D{12.6, 7.2}, Point2D{11.4, 5.3}},
  Area_Square{Point2D{12.6, 7.2}, Point2D{11.4, 5.3}},
  Area_Square{Point2D{12.6, 7.2}, Point2D{11.4, 5.3}},
  Area_Square{Point2D{6.5, 3.6}, Point2D{2.4, 1.4}},
};
// Per-tunnel alignment and recovery velocity configuration, index-aligned with tunnel_zone.
inline std::array<TunnelRecoveryConfig, 4> tunnel_recovery_configs = {
  TunnelRecoveryConfig{1.57f, 0.0f, -0.5f, false},
  TunnelRecoveryConfig{1.57f, 0.0f, -0.5f, false},
  TunnelRecoveryConfig{1.57f, 0.0f, -0.5f, false},
  TunnelRecoveryConfig{3.14f, 0.5f, 0.0f, false},
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
  Point2D{6.4, 6.9}, 
  Point2D{6.4, 5.3},
  Point2D{6.4, 5.1},
  Point2D{5.5, 5.1},
  Point2D{2.4, 5.1},
  Point2D{2.4, 5.3},
  Point2D{2.4, 6.9},
  Point2D{5.3, 6.9},
};
inline AreaPolygon<8, Point2D> own_defense_zone{
  Point2D{12.4, 4.5}, 
  Point2D{9.6, 4.5},
  Point2D{7.8, 4.5},
  Point2D{5.0, 4.5},
  Point2D{5.0, 0.5},
  Point2D{7.8, 0.5},
  Point2D{9.6, 0.5},
  Point2D{12.4, 0.5},
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
inline Area_Square enemy_outpost_watch_zone{Point2D{4.6, 6.6}, Point2D{3.2,5.2 }};
inline Area_Circle enemy_fort_zone{Point2D{12.7, 3.3}, 0.4};
inline AreaPolygon<8, Point2D> engineering_zone{
  Point2D{12.0, 2.0},
  Point2D{11.0, 2.0},
  Point2D{10.0, 2.0},
  Point2D{10.0, 3.0},
  Point2D{10.0, 4.0},
  Point2D{11.0, 4.0},
  Point2D{12.0, 4.0},
  Point2D{12.0, 3.0},
};

inline AreaPolygon<6, Point2D> own_highland_buff_zone{
  Point2D{10.1, 7.2},
  Point2D{10.1, 6.7},
  Point2D{10.1, 6.2},
  Point2D{8.3, 6.2},
  Point2D{8.3, 6.7},
  Point2D{8.3, 7.2},
};
inline AreaPolygon<6, Point2D> enemy_highland_buff_zone{
  Point2D{10.1, 7.2},
  Point2D{10.1, 6.7},
  Point2D{10.1, 6.2},
  Point2D{8.3, 6.2},
  Point2D{8.3, 6.7},
  Point2D{8.3, 7.2},
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
inline Area_Square own_supply_zone{Point2D{0.0, 0.0}, Point2D{0.0, 0.0}};
inline Area_Square enemy_supply_zone{Point2D{0.0, 0.0}, Point2D{0.0, 0.0}};
inline AreaPolygon<6, Point2D> own_outpost_buff_zone{
  Point2D{10.1, 7.2},
  Point2D{10.1, 6.7},
  Point2D{10.1, 6.2},
  Point2D{8.3, 6.2},
  Point2D{8.3, 6.7},
  Point2D{8.3, 7.2},
};
inline AreaPolygon<6, Point2D> enemy_outpost_buff_zone{
  Point2D{10.1, 7.2},
  Point2D{10.1, 6.7},
  Point2D{10.1, 6.2},
  Point2D{8.3, 6.2},
  Point2D{8.3, 6.7},
  Point2D{8.3, 7.2},
};

inline std::vector<Point2D> nav_points = {

  // for test
  {8.1, 2.4, 0.0},  // HOME
  // {15.0, 4.4, 0.0},  // HOME
  {5.6, 3.8, 0.0},  // BONUS
  // {9.1, 6.5, 0.0},  // OUTPOST
  {3.9, 5.9, 0.0},  // OUTPOST
  {12.6, 3.0, 0.0},  // OWN_FORT
  {12.3, 3.3, 0.0},  // ENEMY_FORT
  {0.0, 0.0, 0.0},   // OWN_OUTPOST (test-area placeholder)
  {11.0, 10.9, 0.0}  // HERO_GUARD
};

using PatrolList = std::vector<PatrolPoint>; 
inline std::vector<PatrolList> normal_patrol_branches = {
  // for test
  {
    {{10.0, 2.5, 0.0}, 5000},
    // {{11.0, 3.1, 0.0}, 5000},
    {{15.0, 4.2, 0.0}, 5000},
    {{3.9, 4.9, 0.0}, 10000},
    // {{8.1, 6.4, 0.0}, 10000}
  },
  {
    {{3.9, 4.9, 0.0}, 10000},
    {{10.0, 2.7, 0.0}, 10000} 
  },
  {
    // {{3.9, 4.9, 0.0}, 10000},
    {{10.0, 2.7, 0.0}, 10000}
  }
};

inline std::vector<PatrolList> attack_patrol_branches = {
  {
    {{16.0, 12.0, 0.0}, 5000},
    {{17.3, 7.9, 0.0}, 5000},
    {{15.3, 11.0, 0.0}, 6000}
  }
};
#endif
// =============== 战略模式：巡逻点与云台巡检区域映射表 ===============

using PatrolList = std::vector<PatrolPoint>;
using GimbalPatrolAreaList = std::vector<GimbalPatrolPoint>;

// 云台单区域扫描配置
struct GimbalPatrolConfig {
  float scan_yaw_min;
  float scan_yaw_max;
};

// 巡检区域枚举
enum class PatrolZoneType {
  ENEMY_DEFENSE,
  OWN_DEFENSE,
  HIGHLAND,
  OWN_OUTPOST,
  STAIRZONE
};

struct PatrolZoneTypeHash {
  std::size_t operator()(PatrolZoneType t) const {
    return static_cast<std::size_t>(t);
  }
};

// 云台巡检区域映射表 (TacticalMode -> (PatrolZoneType -> GimbalPatrolConfig))
inline std::unordered_map<TacticalMode, std::unordered_map<PatrolZoneType, GimbalPatrolConfig, PatrolZoneTypeHash>> tactical_area_gimbal_map = {
  {TacticalMode::OFFENSIVE, {
    {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
    {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
    {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
  }},
  {TacticalMode::DEFENSIVE, {
    {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
    {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
    {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
  }},
  {TacticalMode::BALANCED, {
    {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
    {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
    {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
    {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
  }}
};

// inline std::unordered_map<TacticalMode, std::unordered_map<PatrolZoneType, GimbalPatrolConfig, PatrolZoneTypeHash>> tactical_area_gimbal_map = {
//   {TacticalMode::OFFENSIVE, {
//     {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
//     {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
//     {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
//   }},
//   {TacticalMode::DEFENSIVE, {
//     {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
//     {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
//     {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
//   }},
//   {TacticalMode::BALANCED, {
//     {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}},
//     {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}},
//     {PatrolZoneType::OWN_OUTPOST,   {-180.0f, 180.0f}},
//     {PatrolZoneType::STAIRZONE,     {-180.0f, 180.0f}}
//   }}
// };

// 1. 底盘巡逻点映射表 (TacticalMode -> PatrolList)
inline std::unordered_map<TacticalMode, std::vector<PatrolList>> tactical_patrol_branches = {
    {TacticalMode::DEFENSIVE, normal_patrol_branches},
    {TacticalMode::BALANCED, normal_patrol_branches},
    {TacticalMode::OFFENSIVE, attack_patrol_branches}
};
// clang-format on
inline const std::unordered_map<TacticalMode, std::vector<AreaPolygon<8, Point2D>>> tracking_areas = {
  {TacticalMode::DEFENSIVE, {own_defense_zone, highland_zone, enemy_defense_zone}},
  {TacticalMode::BALANCED, {own_defense_zone, highland_zone, enemy_defense_zone}},
  {TacticalMode::OFFENSIVE, {own_defense_zone, highland_zone, enemy_defense_zone}},
};

inline std::vector<std::string> current_nav_status = {"IDLE", "RUNNING", "SUCCESS", "FAILURE"};
inline std::vector<std::string> mode_names = {"PATROL", "TRACING", "RETREAT", "RESPONSE", "MANUAL"};
inline std::vector<std::string> stance_names = {
  "ATTACK", "DEFEND", "MOVE", "ENHANCED_ATTACK", "ENHANCED_DEFEND", "ENHANCED_MOVE"};

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
    {"tunnel_zone", tunnel_zone[0], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[1], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[2], {1.0F, 0.6F, 0.0F}},
    {"tunnel_zone", tunnel_zone[3], {1.0F, 0.6F, 0.0F}},
    {"stairs_zone", stairs_zone[0], {1.0F, 1.0F, 0.2F}},
    {"stairs_zone", stairs_zone[1], {1.0F, 1.0F, 0.2F}},
    {"stairs_lower_safe_zone", stairs_lower_safe_zone[0], {0.4F, 1.0F, 0.4F}},
    {"stairs_lower_safe_zone", stairs_lower_safe_zone[1], {0.4F, 1.0F, 0.4F}},
    {"enemy_outpost_watch_zone", enemy_outpost_watch_zone, {0.7F, 0.2F, 1.0F}},
    {"own_supply_zone", own_supply_zone, {0.2F, 1.0F, 0.8F}},
    {"enemy_supply_zone", enemy_supply_zone, {1.0F, 0.4F, 0.8F}},
  };
}

inline std::vector<CircleVizConfig> getCircleVizConfigs()
{
  return {
    {"enemy_fort_zone", enemy_fort_zone, {0.7F, 0.2F, 1.0F}},
  };
}

inline std::vector<PolygonVizConfig> getBasePolygonVizConfigs()
{
  return {
    makePolygonVizConfig("transform_zone[0]", transform_zone[0], {1.0F, 0.2F, 0.2F}),
    makePolygonVizConfig("transform_zone[1]", transform_zone[1], {1.0F, 0.2F, 0.2F}),
    makePolygonVizConfig("transform_zone[2]", transform_zone[2], {1.0F, 0.2F, 0.2F}),
    makePolygonVizConfig("transform_zone[3]", transform_zone[3], {1.0F, 0.2F, 0.2F}),
    makePolygonVizConfig("highland_zone", highland_zone, {0.2F, 0.6F, 1.0F}),
    makePolygonVizConfig("own_defense_zone", own_defense_zone, {0.6F, 0.4F, 1.0F}),
    makePolygonVizConfig("enemy_defense_zone", enemy_defense_zone, {1.0F, 0.2F, 1.0F}),
    makePolygonVizConfig("engineering_zone", engineering_zone, {1.0F, 1.0F, 0.0F}),
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
