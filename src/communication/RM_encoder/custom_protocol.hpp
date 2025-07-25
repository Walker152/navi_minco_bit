#pragma once
#ifndef __CUSTOM_PROTOCOL_HPP_
#define __CUSTOM_PROTOCOL_HPP_

typedef enum _ArmEnum{
    ENUM_ARM_SLAVE_COMPUTER = 0, //下位机，指电控
    ENUM_ARM_HERO = 1,
    ENUM_ARM_MINER = 2,
    ENUM_ARM_INFANTRY3 = 3, //3号步兵
    ENUM_ARM_INFANTRY4 = 4,//4号步兵
    ENUM_ARM_INFANTRY5 = 5,//5号步兵
    ENUM_ARM_UAV,
    ENUM_ARM_SENTRY,
    ENUM_ARM_DARTS,//飞镖
    ENUM_ARM_RADAR,//雷达
    ENUM_ARM_OPERATOR,//操作手
    ENUM_ARM_BASE,
    ENUM_ARM_OUTPOST,
    ENUM_ARM_UNDEFINED,//未定义兵种
    ENUM_ARM_ALL = 0xff
} ArmEnum;


/* 一般情况下导航通信包 */
typedef struct _NavRes{
    float x;//-y
    float y;//-z
    float yaw;//x
    uint8_t is_reach;

    _NavRes(float _x, float _y, float _yaw, uint8_t _is_reach)
            :x(_x),y(_y),yaw(_yaw),is_reach(_is_reach)
    {

    }
} NavRes;

// ------ 底盘 JNChassis ------
// ---- 底盘运动模式

/* ---- 底盘运动期望，世界系 @ref=SET_CHASSIS_TARGET */
typedef struct _ChassisTarget {
    float vx_mps; /* x轴，即前进速度，单位 m/s */
    float vy_mps; /* y轴，即向左速度，单位 m/s */
    float vw_rpm; /* z轴，即旋转速度，单位 */
    float current_x;
    float current_y; // 单位m
    float current_yaw; // 与x轴夹角，单位rad
    

    _ChassisTarget(float _vx_mps, float _vy_mps, float _vw_rpm, float _current_x, float _current_y, float _current_yaw)
                    :vx_mps(_vx_mps), vy_mps(_vy_mps), vw_rpm(_vw_rpm), current_x(_current_x), current_y(_current_y), current_yaw(_current_yaw)
    {

    }
} ChassisTarget;

/* -----电机转速----- */
typedef struct _MotorSpeed{
    float forward_left_motor_speed;
    float forward_right_motor_speed;
    float backward_right_motor_speed;
    float backward_left_motor_speed;

    _MotorSpeed(float _forward_left_motor_speed = 0, float _forward_right_motor_speed = 0, float _backward_right_motor_speed = 0, float _backward_left_motor_speed = 0)
                :forward_left_motor_speed(_forward_left_motor_speed), forward_right_motor_speed(_forward_right_motor_speed), backward_right_motor_speed(_backward_right_motor_speed), backward_left_motor_speed(backward_left_motor_speed)
    {

    }
}MotorSpeed;

#define PB_BYTES_ARRAY_T(n) struct { uint16_t size; uint16_t bytes[n]; }
typedef PB_BYTES_ARRAY_T(52) ForwardingMsg_data_t;

#pragma pack( push, 1 )
#pragma pack( pop )
#endif