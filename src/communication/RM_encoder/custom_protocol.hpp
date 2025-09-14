#pragma once
#ifndef __CUSTOM_PROTOCOL_HPP_
#define __CUSTOM_PROTOCOL_HPP_

// ---------------------------
// 兵种（角色）枚举类型定义，用于标识不同机器人或者操作角色
// ---------------------------
typedef enum _ArmEnum
{
    ENUM_ARM_SLAVE_COMPUTER = 0, // 下位机，指电控系统
    ENUM_ARM_HERO = 1,           // 英雄角色
    ENUM_ARM_MINER = 2,          // 矿工角色
    ENUM_ARM_INFANTRY3 = 3,      // 3号步兵
    ENUM_ARM_INFANTRY4 = 4,      // 4号步兵
    ENUM_ARM_INFANTRY5 = 5,      // 5号步兵
    ENUM_ARM_UAV = 6,            // 无人机（UAV）
    ENUM_ARM_SENTRY = 7,         // 哨兵
    ENUM_ARM_DARTS,              // 飞镖
    ENUM_ARM_RADAR,              // 雷达
    ENUM_ARM_OPERATOR,           // 操作手
    ENUM_ARM_BASE,               // 基地
    ENUM_ARM_OUTPOST,            // 前哨
    ENUM_ARM_UNDEFINED,          // 未定义兵种
    ENUM_ARM_ALL = 0xff          // 所有兵种，通常用于广播或全体标识
} ArmEnum;

// -----------------------------------------------------
// 一般用于导航系统的响应数据包结构体定义
// -----------------------------------------------------
typedef struct _NavRes
{
    float x;
    float y;
    float yaw;
    uint8_t is_reach; // 是否到达目标点，布尔标志，一般0为未到达，1为已到达

    // 带参构造函数，便于初始化结构体成员值
    _NavRes(float _x, float _y, float _yaw, uint8_t _is_reach)
        : x(_x), y(_y), yaw(_yaw), is_reach(_is_reach)
    {
    }
} NavRes;

// 发送给底盘电控的数据
typedef struct _ChassisTarget
{
    float vx_mps;      // 前进方向速度(m/s)
    float vy_mps;      // 左侧方向速度(m/s)
    float vw_rpm;      // 小陀螺速度(rpm)
    float current_x;   // 当前x位置(m)
    float current_y;   // 当前y位置(m)
    float current_yaw; // 当前朝向角(rad)

    _ChassisTarget(float _vx_mps, float _vy_mps, float _vw_rpm,
                   float _current_x, float _current_y, float _current_yaw)
        : vx_mps(_vx_mps), vy_mps(_vy_mps), vw_rpm(_vw_rpm),
          current_x(_current_x), current_y(_current_y), current_yaw(_current_yaw)
    {
    }
} ChassisTarget;

// 控制4个电机转速 // 可能用不到了
typedef struct _MotorSpeed
{
    float forward_left_motor_speed;
    float forward_right_motor_speed;
    float backward_right_motor_speed;
    float backward_left_motor_speed;

    _MotorSpeed(float _forward_left_motor_speed = 0,
                float _forward_right_motor_speed = 0,
                float _backward_right_motor_speed = 0,
                float _backward_left_motor_speed = 0)
        : forward_left_motor_speed(_forward_left_motor_speed),
          forward_right_motor_speed(_forward_right_motor_speed),
          backward_right_motor_speed(_backward_right_motor_speed),
          backward_left_motor_speed(_backward_left_motor_speed)
    {
    }
} MotorSpeed;
// ？？？？？？
// -----------------------------------------------------
// 自定义数据结构定义，用于处理字节数组及大小信息
// -----------------------------------------------------
#define PB_BYTES_ARRAY_T(n) \
    struct                  \
    {                       \
        uint16_t size;      \
        uint16_t bytes[n];  \
    }
// 定义ForwardingMsg_data_t为长度为52的字节数组加上size信息
typedef PB_BYTES_ARRAY_T(52) ForwardingMsg_data_t;

// -----------------------------------------------------
// 字节对齐指令，作用于结构体对齐，确保通信协议数据包结构精准对齐
// 这里暂时无结构体放置在pack范围内，仅为以后扩展预留
// -----------------------------------------------------
#pragma pack(push, 1)
// 这里可放置需要1字节对齐的结构体定义
#pragma pack(pop)

#endif // __CUSTOM_PROTOCOL_HPP_