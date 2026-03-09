#pragma once
// 数据包类型
#include <cstdint>
enum PacketTypeEnum
{
  ENUM_PACKET_HEART_BEAT = 0,                   // 心跳包
  ENUM_PACKET_SENTRY_ACCEPT_STATUS_DATA = 1,    // 哨兵接受状态数据
  ENUM_PACKET_ARMOR_DATA = 2,                   // 装甲板数据
  ENUM_PACKET_SENTRY_SEND_CONTROL_DATA = 3,     // 哨兵发送控制数据
  ENUM_PACKET_GIMBAL_RESET = 4,                 // 云台复位命令
  ENUM_PACKET_RADAR_SMALL_MAP = 5,              // 雷达小地图数据
  ENUM_PACKET_OBSTACLE_DATE = 6,                // 障碍物信息包
  ENUM_PACKET_UNDEFINED = 7,                    // 未定义包
  ENUM_PACKET_INFANTRY_ACCEPT_STATUS_DATA = 8,  // 步兵状态数据
  ENUM_PACKET_BUFF_DATA = 9,                    // buff数据包
  ENUM_PACKET_LOCATION = 10,                    // 位置相关包
  ENUM_PACKET_CMDCONTROL = 11,                  // 控制命令包
  ENUM_PACKET_DETECTOR_DATA = 12,               // 探测器数据
  ENUM_PACKET_GAMESTATUS_DATA = 13,             // 游戏状态数据
  ENUM_PACKET_NAV_DATA = 14                     // 导航数据
};
// from to 类型
enum _ArmEnum
{
  ENUM_ARM_SLAVE_COMPUTER = 0,  // 下位机，指电控系统
  ENUM_ARM_HERO = 1,            // 英雄角色
  ENUM_ARM_MINER = 2,           // 矿工角色
  ENUM_ARM_INFANTRY3 = 3,       // 3号步兵
  ENUM_ARM_INFANTRY4 = 4,       // 4号步兵
  ENUM_ARM_INFANTRY5 = 5,       // 5号步兵
  ENUM_ARM_UAV = 6,             // 无人机（UAV）
  ENUM_ARM_SENTRY = 7,          // 哨兵
  ENUM_ARM_DARTS = 8,           // 飞镖
  ENUM_ARM_RADAR = 9,           // 雷达
  ENUM_ARM_OPERATOR = 10,       // 操作手
  ENUM_ARM_BASE = 11,           // 基地
  ENUM_ARM_OUTPOST = 12,        // 前哨
  ENUM_ARM_UNDEFINED = 13,      // 未定义兵种
  ENUM_ARM_ALL = 0xff           // 所有兵种，通常用于广播或全体标识
};
using ArmEnum = enum _ArmEnum;

enum _LifterPos
{
  TOP = 0,                      // 云台顶部
  BOTTOM = 1,                   // 云台底部
  MIDDLE = 2                    // 云台升降中
};
using LifterPos = _LifterPos;

#pragma pack(push, 1)  // 设置内存对齐格式为1个字节
// 1.
// STM32to导航数据
struct __attribute__((packed, aligned(1))) _NavRes
{
  float x;
  float y;
  float yaw;
  bool is_reach;  // 是否到达目标点，布尔标志，一般0为未到达，1为已到达

  // 带参构造函数，便于初始化结构体成员值
  _NavRes(float _x, float _y, float _yaw, bool _is_reach)
    : x(_x)
    , y(_y)
    , yaw(_yaw)
    , is_reach(_is_reach)
  {
  }
};
using NavRes = struct _NavRes;
// 2.
struct _ChassisTarget
{
  float vx_mps;       // 前进方向速度(m/s)
  float vy_mps;       // 左侧方向速度(m/s)
  float vw_rpm;       // 小陀螺速度(rpm)
  float current_x;    // 当前x位置(m)
  float current_y;    // 当前y位置(m)
  float current_yaw;  // 当前朝向角(rad)
  bool is_aim_outpost;// 是否抬头击打前哨站
  uint8_t desire_stance;     // 哨兵姿态
  uint8_t desire_lifter_pos; // 云台升降状态

  // float vx_mps{}, vy_mps{}, vw_rpm{};
  // float current_x{}, current_y{}, current_yaw{}, radar_yaw{};
  // bool is_aim_outpost{};      // 发1则强制哨兵抬头巡检，寻找前哨站
  // int32_t sentry_state{};    // 哨兵姿态，遵循Sentry_StateEnum
  // minipc_to_stm32() = default;

  _ChassisTarget(float _vx_mps,
                 float _vy_mps,
                 float _vw_rpm,
                 float _current_x,
                 float _current_y,
                 float _current_yaw,
                 bool _is_aim_outpost,
                 uint8_t _desire_stance,
                 uint8_t _desire_lifter_pos
                )
    : vx_mps(_vx_mps)
    , vy_mps(_vy_mps)
    , vw_rpm(_vw_rpm)
    , current_x(_current_x)
    , current_y(_current_y)
    , current_yaw(_current_yaw)
    , is_aim_outpost(_is_aim_outpost)
    , desire_stance(_desire_stance)
    , desire_lifter_pos(_desire_lifter_pos)
  {
  }
};
using ChassisTarget = struct _ChassisTarget;

struct __attribute__((packed, aligned(1))) _Event_Status
{
  uint16_t self_health;        // 自身健康值，范围0-400
  uint16_t num_shoot;          // 当前子弹量
  uint16_t own_outpost_health; // 我方前哨站血量
  bool enemy_outpost_destroyed;// 敌方前哨站是否被摧毁标志
  bool buff_active;            // buff是否激活标志
  bool is_get;                 // 是否检测到敌人
  float x;                     // 敌人位置x坐标（相机系）
  float y;                     // 敌人位置y坐标
  float z;                     // 敌人位置z坐标
  uint8_t armor_id;            // 敌人装甲板ID
  uint8_t current_stance;      // 当前哨兵姿态
  uint8_t game_status;         // 比赛状态 
  // 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
  float gimbal_yaw;            // 云台当前yaw 逆时针为正
  uint8_t lifter_pos_now;      // 云台当前升降状态



// uint16_t self_health{};
//     uint16_t bullets_remaining{};
//     bool own_outpost_destroyed;
//     uint16_t enemy_outpost_health{};
//     bool buff_active;
//     bool is_get;
//     float armor_pos[3];
//     uint8_t armor_num;      // 进行处理，红方蓝方发送的装甲板数字相同
//     int32_t sentry_current_state;
//     float team_pos[5][2];
//     uint8_t game_status;    // 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
//     float yaw_imu;      // imu的yaw轴角度 时针逆为正
//     inf_stm32_to_minipc() = default;


  _Event_Status(uint16_t _self_health,
                uint16_t _num_shoot,
                uint16_t _own_outpost_health,
                bool _enemy_outpost_destroyed,
                bool _buff_active,
                bool _is_get,
                float _x,
                float _y,
                float _z,
                uint8_t _armor_id,
                uint8_t _current_stance,
                uint8_t _game_status,
                float _gimbal_yaw,
                uint8_t _lifter_pos_now )
    : self_health(_self_health)
    , num_shoot(_num_shoot)
    , enemy_outpost_destroyed(_enemy_outpost_destroyed)
    , own_outpost_health(_own_outpost_health)
    , buff_active(_buff_active)
    , is_get(_is_get)
    , x(_x)
    , y(_y)
    , z(_z)
    , armor_id(_armor_id)
    , current_stance(_current_stance)
    , game_status(_game_status)
    , gimbal_yaw(_gimbal_yaw)
    , lifter_pos_now(_lifter_pos_now)
  {
  }
};

using EventStatus = struct _Event_Status;
#pragma pack(pop)
