#pragma once
// 数据包类型
#include <cstdint>
// enum PacketTypeEnum
// {
//   ENUM_PACKET_HEART_BEAT = 0,                   // 心跳包
//   ENUM_PACKET_SENTRY_ACCEPT_STATUS_DATA = 1,    // 哨兵接受状态数据
//   ENUM_PACKET_ARMOR_DATA = 2,                   // 装甲板数据
//   ENUM_PACKET_SENTRY_SEND_CONTROL_DATA = 3,     // 哨兵发送控制数据
//   ENUM_PACKET_GIMBAL_RESET = 4,                 // 云台复位命令
//   ENUM_PACKET_RADAR_SMALL_MAP = 5,              // 雷达小地图数据
//   ENUM_PACKET_OBSTACLE_DATE = 6,                // 障碍物信息包
//   ENUM_PACKET_UNDEFINED = 7,                    // 未定义包
//   ENUM_PACKET_INFANTRY_ACCEPT_STATUS_DATA = 8,  // 步兵状态数据
//   ENUM_PACKET_BUFF_DATA = 9,                    // buff数据包
//   ENUM_PACKET_LOCATION = 10,                    // 位置相关包
//   ENUM_PACKET_CMDCONTROL = 11,                  // 控制命令包
//   ENUM_PACKET_DETECTOR_DATA = 12,               // 探测器数据
//   ENUM_PACKET_GAMESTATUS_DATA = 13,             // 游戏状态数据
//   ENUM_PACKET_NAV_DATA = 14                     // 导航数据
// };
enum PacketTypeEnum
{
  ENUM_PACKET_NAV_DATA,
  ENUM_PACKET_ALLY_STATUS,  // 己方机器人状态
  ENUM_PACKET_GAMESTATUS_DATA,
  ENUM_PACKET_SENTRY_SERVER_DATA,  // 哨兵姿态等信息
  ENUM_PACKET_SENTRY_SELF_DATA,    // 机器人自身状态等信息
  ENUM_PACKET_RADAR,               // 雷达发送的消息
  ENUM_PACKET_GLOBAL_PATH_X,       // 全局路径X分包
  ENUM_PACKET_GLOBAL_PATH_Y,       // 全局路径Y分包
  ENUM_PACKET_BEHAVIOR_DATA,       // 行为树决策结果
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
  TOP = 0,     // 云台顶部
  BOTTOM = 1,  // 云台底部
  MIDDLE = 2   // 云台升降中
};
using LifterPos = _LifterPos;

enum _EnergyRatio
{

};
using EnergyRatio = _EnergyRatio;

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
  _NavRes(float _x, float _y, float _yaw, bool _is_reach) : x(_x), y(_y), yaw(_yaw), is_reach(_is_reach) {}
};
using NavRes = struct _NavRes;

struct __attribute__((packed, aligned(1))) _GlobalPath
{
  uint16_t start_x{};    // 起点x（minimap坐标系）
  uint16_t start_y{};    // 起点y（minimap坐标系）
  int8_t delta_x[49]{};  // x方向相对上一点增量
  int8_t delta_y[49]{};  //  y方向相对上一点增量
};
using GlobalPath = struct _GlobalPath;

struct __attribute__((packed, aligned(1))) _GlobalPathX
{
  uint16_t start_x{};    // 起点x（minimap坐标系）
  int8_t delta_x[49]{};  // x方向相对上一点增量
};
using GlobalPathX = struct _GlobalPathX;

struct __attribute__((packed, aligned(1))) _GlobalPathY
{
  uint16_t start_y{};    // 起点y（minimap坐标系）
  int8_t delta_y[49]{};  // y方向相对上一点增量
};
using GlobalPathY = struct _GlobalPathY;

// 2.
struct _ChassisTarget
{
  float vx_mps{};       // 前进方向速度(m/s)
  float vy_mps{};       // 左侧方向速度(m/s)
  float vw_rpm{};       // 小陀螺速度(rpm)
  float current_yaw{};  // 当前朝向角(rad)
  float current_vx{};
  float current_vy{};
  float current_vw{};
  float fx_global{};
  float fy_global{};
  float fw_global{};
  bool use_speed_control{};  // 是否使用速度控制模式，true为使用，false为使用力控制模式
  float delta_yaw{};

  _ChassisTarget(float _vx_mps,
    float _vy_mps,
    float _vw_rpm,
    float _current_yaw,
    float _current_vx,
    float _current_vy,
    float _current_vw,
    float _fx_global,
    float _fy_global,
    float _fw_global,
    bool _use_speed_control,
    float _delta_yaw)
  : vx_mps(_vx_mps), vy_mps(_vy_mps), vw_rpm(_vw_rpm), current_yaw(_current_yaw), current_vx(_current_vx),
    current_vy(_current_vy), current_vw(_current_vw), fx_global(_fx_global), fy_global(_fy_global),
    fw_global(_fw_global), use_speed_control(_use_speed_control), delta_yaw(_delta_yaw)
  {
  }
};
using ChassisTarget = struct _ChassisTarget;

struct __attribute__((packed, aligned(1))) _BehaviorData
{
  uint8_t pitch_mode{};              // 云台抬头模式
  uint8_t desire_stance{};           // 哨兵姿态
  uint8_t desire_lifter_pos{};       // 云台升降状态
  float scan_yaw_min_deg{};          // 扫描最小偏航角（度）
  float scan_yaw_max_deg{};          // 扫描最大偏航角（度）
  uint16_t ammo_purchase_request{};  // 弹药兑换请求（递增累计值）
  uint8_t revive_request{};          // 是否确认复活
  uint8_t remote_revive_request{};   // 远程复活请求
  uint8_t remote_ammo_request{};     // 远程买弹请求
  uint8_t remote_health_request{};   // 远程买血请求
  bool use_limited_scan{};           // 是否使用限制性扫描模式
  bool not_aim_enemy{};               // 是否瞄准敌方目标

  _BehaviorData(uint8_t _pitch_mode,
    uint8_t _desire_stance,
    uint8_t _desire_lifter_pos,
    float _scan_yaw_min_deg,
    float _scan_yaw_max_deg,
    uint16_t _ammo_purchase_request,
    uint8_t _revive_request,
    uint8_t _remote_revive_request,
    uint8_t _remote_ammo_request,
    uint8_t _remote_health_request,
    bool _use_limited_scan,
    bool _not_aim_enemy)
  : pitch_mode(_pitch_mode), desire_stance(_desire_stance), desire_lifter_pos(_desire_lifter_pos),
    scan_yaw_min_deg(_scan_yaw_min_deg), scan_yaw_max_deg(_scan_yaw_max_deg),
    ammo_purchase_request(_ammo_purchase_request), revive_request(_revive_request),
    remote_revive_request(_remote_revive_request), remote_ammo_request(_remote_ammo_request),
    remote_health_request(_remote_health_request), use_limited_scan(_use_limited_scan),
    not_aim_enemy(_not_aim_enemy)
  {
  }
};
using BehaviorData = struct _BehaviorData;
struct __attribute__((packed)) AllyRobotStatus
{
  uint8_t robot_id{};   // 机器人ID，蓝方=红方+100
  uint16_t robot_hp{};  // 机器人血量
  float robot_pos_x{};  // 机器人位置x坐标，单位m
  float robot_pos_y{};  // 机器人位置y坐标，单位m
};

struct __attribute__((packed)) EnemyRobotStatus
{
  uint8_t robot_id{};             // 机器人ID，蓝方=红方+100
  uint16_t robot_hp{};            // 机器人血量
  uint16_t allowed_projectile{};  // 允许发射的弹丸数量，单位发
  uint16_t robot_pos_x{};         // 机器人位置x坐标，单位cm
  uint16_t robot_pos_y{};         // 机器人位置y坐标，单位cm
};

struct __attribute__((packed)) _TeamInfo
{
  AllyRobotStatus ally_status[4]{};  // 4个友方机器人状态
  uint16_t outpost_hp{};             // 前哨站血量
  uint16_t base_hp{};                // 基地血量
  _TeamInfo(const AllyRobotStatus _ally_status[4], uint16_t _outpost_hp, uint16_t _base_hp)
  {
    for (int i = 0; i < 4; ++i) {
      ally_status[i] = _ally_status[i];
    }
    outpost_hp = _outpost_hp;
    base_hp = _base_hp;
  }
};
using TeamInfo = struct _TeamInfo;

struct __attribute__((packed)) _GameInfo
{
  uint16_t game_time_remaining{};  // 比赛剩余时间，单位s
  uint16_t coin_remaining{};       // 己方剩余金币数量
  uint32_t event_code{};           // 场地事件代码，未解码，需接收后根据协议解码
  uint8_t game_status{};  // 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
  float manual_point_x{};  // 手动指定的目标点x坐标，单位m，坐标系minimap
  float manual_point_y{};  // 手动指定的目标点y坐标，单位m，坐标系minimap
  uint8_t manual_key{};    // 手动指定的目标点快捷键

  _GameInfo(
    uint16_t _game_time_remaining, uint16_t _coin_remaining, uint32_t _event_code, uint8_t _game_status)
  {
    game_time_remaining = _game_time_remaining;
    coin_remaining = _coin_remaining;
    event_code = _event_code;
    game_status = _game_status;
  }
};
using GameInfo = struct _GameInfo;

struct __attribute__((packed)) _SentryInfoOnline
{
  // 数据：己方&敌方前哨站，哨兵血量，场地事件
  uint16_t self_health{};        // 机器人血量
  uint16_t bullets_remaining{};  // 剩余发弹量
  uint16_t cooling_value{};      // 机器人每秒冷却值
  uint16_t heat_limit{};         // 机器人热量上限
  uint16_t current_heat{};       // 机器人当前热量
  float sentry_pos_x{};          // 哨兵位置x坐标，单位m
  float sentry_pos_y{};          // 哨兵位置y坐标，单位m
  float speed_monitor_angle{};   // 测速模块朝向，单位为度，正北为0度
  uint32_t sentry_info_1{};      // 哨兵信息1，未解码，需接收后根据协议解码
  uint16_t sentry_info_2{};      // 哨兵信息2，未解码，需接收后根据协议解码
  uint8_t energy_ratio{};        // 底盘能量比例

  _SentryInfoOnline(uint16_t _self_health,
    uint16_t _bullets_remaining,
    uint16_t _cooling_value,
    uint16_t _heat_limit,
    uint16_t _current_heat,
    float _sentry_pos_x,
    float _sentry_pos_y,
    float _speed_monitor_angle,
    uint32_t _sentry_info_1,
    uint16_t _sentry_info_2)
  {
    self_health = _self_health;
    bullets_remaining = _bullets_remaining;
    cooling_value = _cooling_value;
    heat_limit = _heat_limit;
    current_heat = _current_heat;
    sentry_pos_x = _sentry_pos_x;
    sentry_pos_y = _sentry_pos_y;
    speed_monitor_angle = _speed_monitor_angle;
    sentry_info_1 = _sentry_info_1;
    sentry_info_2 = _sentry_info_2;
  }
};
using SentryInfoOnline = struct _SentryInfoOnline;

struct __attribute__((packed)) _SentryInfoOffline
{
  // 数据：己方&敌方前哨站，哨兵血量，场地事件
  bool is_get{};                     // 视觉是否瞄准到敌人
  float armor_pos[3]{};              // 瞄准到的装甲板位置，具体坐标系询问视觉
  uint8_t armor_num{};               // 不做红蓝方区分
  float yaw_camerainit_to_gimbal{};  // 编码器的yaw轴角度 逆时针为正
  uint8_t lifter_current_pos{};      // 0 -- kTop 1 -- kBottom 2 -- kMiddle
  bool is_transformable{};  // 是否能够进行变形（不止变形中不能变形，升降卡住后也无法进行变形）
  float transform_state{};  // 变形状态，0-1，0%为未变形，100%为完全变形，过渡状态根据实际情况变化
  uint8_t capacitor_capacity{};  // 电容容量百分比，0-100
  float chassis_imu_yaw{};       // 底盘IMU的yaw轴角度 逆时针为正

  _SentryInfoOffline(bool _is_get,
    float _armor_pos[3],
    uint8_t _armor_num,
    float _yaw_camerainit_to_gimbal,
    uint8_t _lifter_current_pos,
    bool _is_transformable,
    float _transform_state,
    uint8_t _capacitor_capacity,
    float _chassis_imu_yaw)
  {
    is_get = _is_get;
    for (int i = 0; i < 3; ++i) {
      armor_pos[i] = _armor_pos[i];
    }
    armor_num = _armor_num;
    yaw_camerainit_to_gimbal = _yaw_camerainit_to_gimbal;
    lifter_current_pos = _lifter_current_pos;
    is_transformable = _is_transformable;
    transform_state = _transform_state;
    capacitor_capacity = _capacitor_capacity;
    chassis_imu_yaw = _chassis_imu_yaw;
  }
};
using SentryInfoOffline = struct _SentryInfoOffline;

struct __attribute__((packed)) _RadarInfo
{
  EnemyRobotStatus enemy_status[6]{};  // 英雄 工程 步兵3 步兵4 无人机 哨兵
  uint16_t enemy_coin_left{};          // 敌方剩余金币数量
  uint16_t enemy_coin_accumulated{};   // 敌方累计获得金币数量
  bool is_enemy_outpost_sensed{};      // 敌方前哨站是否被雷达识别到
  _RadarInfo(const EnemyRobotStatus _enemy_status[6],
    uint16_t _enemy_coin_left,
    uint16_t _enemy_coin_accumulated,
    bool _is_enemy_outpost_sensed)
  {
    for (int i = 0; i < 6; ++i) {
      enemy_status[i] = _enemy_status[i];
    }
    enemy_coin_left = _enemy_coin_left;
    enemy_coin_accumulated = _enemy_coin_accumulated;
    is_enemy_outpost_sensed = _is_enemy_outpost_sensed;
  }
};
using RadarInfo = struct _RadarInfo;

#pragma pack(pop)
