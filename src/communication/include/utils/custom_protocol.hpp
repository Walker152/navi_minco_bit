#pragma once
// 数据包类型
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
  float gimbal_yaw;   // 期望云台角
  uint8_t is_use_mid360;

  _ChassisTarget(float _vx_mps,
                 float _vy_mps,
                 float _vw_rpm,
                 float _current_x,
                 float _current_y,
                 float _current_yaw,
                 float _gimbal_yaw,
                uint8_t _is_use_mid360
                
                )
    : vx_mps(_vx_mps)
    , vy_mps(_vy_mps)
    , vw_rpm(_vw_rpm)
    , current_x(_current_x)
    , current_y(_current_y)
    , current_yaw(_current_yaw)
    , gimbal_yaw(_gimbal_yaw)
    , is_use_mid360(_is_use_mid360)
  {
  }
};
using ChassisTarget = struct _ChassisTarget;

struct __attribute__((packed, aligned(1))) _Event_Status
{
  uint16_t self_health;        // 自身健康值，范围0-400
  uint16_t num_shoot;
  bool own_outpost_destroyed;  // 我方前哨被摧毁标志
  bool buff_active;            // buff是否激活标志
  bool is_get;                 // 是否检测到敌人
  float x;                     // 敌人位置x坐标
  float y;                     // 敌人位置y坐标
  float z;                     // 敌人位置z坐标
  uint8_t armor_id;            // 敌人装甲板ID

  float team_position[5][2];


  _Event_Status(float _self_health,
                uint16_t _num_shoot,
                bool _own_outpost_destroyed,
                bool _buff_active,
                bool _is_get,
                float _x,
                float _y,
                float _z,
                int _armor_id)
    : self_health(_self_health)
    , num_shoot(_num_shoot)
    , own_outpost_destroyed(_own_outpost_destroyed)
    , buff_active(_buff_active)
  {
    is_get = _is_get;
    x = _x;
    y = _y;
    z = _z;
    armor_id = _armor_id;
    memset(team_position, 0, sizeof(team_position));
    for(int i = 0; i < 5; ++i)
    {
      team_position[i][0] = 0;
      team_position[i][1] = 0;
    }
  }
};

using EventStatus = struct _Event_Status;
#pragma pack(pop)
