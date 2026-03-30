#ifndef SOL_CHASSIS_MINIPC_H
#define SOL_CHASSIS_MINIPC_H

#include "BoardCom.h"
#include <cstdint>
#include <app.h>
#include <sheriffos.h>

namespace minipc {

struct Data {
    uint32_t len{};
    uint8_t buffer[128]{};
    explicit Data(uint8_t const* ptr, uint32_t const len) : len(len) { memcpy(buffer, ptr, len); }

    explicit Data() = default;
};

enum PacketTypeEnum {
    ENUM_PACKET_HEART_BEAT = 0,
    ENUM_PACKET_SENTRY_ACCEPT_STATUS_DATA = 1,
    ENUM_PACKET_ARMOR_DATA = 2,
    ENUM_PACKET_SENTRY_SEND_CONTROL_DATA,  // 哨兵--装甲板信息
    ENUM_PACKET_GIMBAL_RESET,              // 云台复位
    ENUM_PACKET_RADAR_SMALL_MAP,           // 雷达发送的消息
    ENUM_PACKET_OBSTACLE_DATE,             // 障碍快信息
    ENUM_PACKET_UNDEFINED,
    ENUM_PACKET_INFANTRY_ACCEPT_STATUS_DATA,
    ENUM_PACKET_BUFF_DATA,
    ENUM_PACKET_LOCATION,
    ENUM_PACKET_CMDCONTROL,
    ENUM_PACKET_DETECTOR_DATA,
    ENUM_PACKET_GAMESTATUS_DATA,
    ENUM_PACKET_NAV_DATA
};

enum class MiniPC_StateEnum : uint8_t { MINIPC_NULL = 0, MINIPC_CONNECTED = 1, MINIPC_LOST = 2, MINIPC_ERROR = 3 };

// 哨兵姿态：
// 进攻姿态：3倍冷却增益，底盘功率减半，25%易伤
// 防御姿态：50%防御增益，底盘功率减半，冷却速率降为1/3
// 移动姿态：底盘功率上限变为原来的1.5倍，25%易伤，冷却速率变为原来的1/3
// 削弱：
// 进攻：2倍冷却增益，底盘功率减半，25%易伤
// 防御：25%防御增益，底盘功率减半，冷却速率降为1/3
// 移动：底盘功率上限变为原来的1.2倍，25%易伤，冷却速率变为原来的1/3

// 此处代码用于和导航调试 by wyc 2025.11.14

enum Sentry_StateEnum : uint8_t {
    SENTRY_MOBILE = 0,
    SENTRY_AGGRESSIVE,
    SENTRY_DEFENSIVE,
    // SENTRY_WEAK_MOBILE,
    // SENTRY_WEAK_AGGRESSIVE,
    // SENTRY_WEAK_DEFENSIVE
};

inline float cool_rate_ratio = 1.0f;
inline float power_limit_ratio = 1.0f;

struct __attribute__((packed)) NAVIGATION_HEADER {
    uint8_t start_flag0{};  // 0xA5
    uint8_t start_flag1{};  // 0x5A
    uint8_t from{};
    uint8_t to{};
    uint8_t packet_type{};
    uint8_t data_len{};  // 数据字段长度
    uint16_t checksum{};
};

struct __attribute__((packed)) AllyRobotStatus {
    uint8_t robot_id{};      // 机器人ID，蓝方=红方+100
    uint16_t robot_hp{};     // 机器人血量
    uint16_t robot_pos_x{};  // 机器人位置x坐标，单位m
    float robot_pos_y{};     // 机器人位置y坐标，单位m
};

// 接收导航数据
struct __attribute__((packed)) minipc_to_stm32 {
    NAVIGATION_HEADER header{};
    float vx_mps{}, vy_mps{}, vw_rpm{};
    float current_x{}, current_y{}, current_yaw{};
    bool is_aim_outpost{};   // 发1则强制哨兵抬头巡检，寻找前哨站
    uint8_t sentry_state{};  // 哨兵姿态，遵循Sentry_StateEnum
    uint8_t lifter_pos{};    // 0 -- kTop 1 -- kBottom
    minipc_to_stm32() = default;
};

// 发送导航数据
struct __attribute__((packed)) stm32_to_minipc_nav {
    NAVIGATION_HEADER header{};
    float tar_x{}, tar_y{}, tar_yaw{};
    uint8_t is_reach{};
    stm32_to_minipc_nav() = default;
};

struct __attribute__((packed)) navigation_data_t {
    float last_update_time{0};
    MiniPC_StateEnum state{MiniPC_StateEnum::MINIPC_LOST};
    minipc_to_stm32 receive{};
    stm32_to_minipc_nav transmit{};
};

navigation_data_t& navigation_data_instance();

// 己方机器人信息（不包括己方哨兵），包括前哨站与基地。
// 数据均从服务器获取，想要测试请连接服务器查看，注意机器人位置只有赛场上才可获取
struct __attribute__((packed)) stm32_to_minipc_ally {
    // 数据：己方&敌方前哨站，哨兵血量，场地事件
    NAVIGATION_HEADER header{};
    AllyRobotStatus ally_status[4]{};
    uint16_t outpost_hp{};  // 前哨站血量
    uint16_t base_hp{};     // 基地血量
    stm32_to_minipc_ally() = default;
};

// 比赛相关数据，请连接服务器查看，场地数据需自行解码
struct __attribute__((packed)) stm32_to_minipc_game {
    // 数据：己方&敌方前哨站，哨兵血量，场地事件
    NAVIGATION_HEADER header{};
    uint16_t game_time_remaining{};  // 比赛剩余时间，单位s
    uint16_t coin_remaining{};       // 己方剩余金币数量
    uint32_t event_code{};           // 场地事件代码，未解码，需接收后根据协议解码
    uint8_t game_status{};           // 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
    stm32_to_minipc_game() = default;
};

// 自身从服务器/裁判系统获取的信息，哨兵姿态包含在sentry_info中
struct __attribute__((packed)) stm32_to_minipc_self_server {
    // 数据：己方&敌方前哨站，哨兵血量，场地事件
    NAVIGATION_HEADER header{};
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
    stm32_to_minipc_self_server() = default;
};

// 自身不从服务器/裁判系统获取的信息
struct __attribute__((packed)) stm32_to_minipc_self_offline {
    // 数据：己方&敌方前哨站，哨兵血量，场地事件
    NAVIGATION_HEADER header{};
    bool is_get{};                 // 视觉是否瞄准到敌人
    float armor_pos[3]{};          // 瞄准到的装甲板位置，具体坐标系询问视觉
    uint8_t armor_num{};           // 不做红蓝方区分
    float yaw_imu{};               // imu的yaw轴角度 逆时针为正
    uint8_t lifter_current_pos{};  // 0 -- kTop 1 -- kBottom 2 -- kMiddle
    bool is_transformable{};       // 是否能够进行变形（不止变形中不能变形，升降卡住后也无法进行变形）
    float transform_state{};       // 变形状态，0-1，0为未变形，1为完全变形，过渡状态根据实际情况变化
    stm32_to_minipc_self_offline() = default;
};

struct __attribute__((packed)) ally_information_data_t {
    float last_update_time{0};
    MiniPC_StateEnum state{MiniPC_StateEnum::MINIPC_LOST};
    stm32_to_minipc_ally transmit{};
};

struct __attribute__((packed)) game_information_data_t {
    float last_update_time{0};
    MiniPC_StateEnum state{MiniPC_StateEnum::MINIPC_LOST};
    stm32_to_minipc_game transmit{};
};

struct __attribute__((packed)) self_server_information_data_t {
    float last_update_time{0};
    MiniPC_StateEnum state{MiniPC_StateEnum::MINIPC_LOST};
    stm32_to_minipc_self_server transmit{};
};

struct __attribute__((packed)) self_offline_information_data_t {
    float last_update_time{0};
    MiniPC_StateEnum state{MiniPC_StateEnum::MINIPC_LOST};
    stm32_to_minipc_self_offline transmit{};
};

ally_information_data_t& ally_information_data_instance();
game_information_data_t& game_information_data_instance();
self_server_information_data_t& self_server_information_data_instance();
self_offline_information_data_t& self_offline_information_data_instance();

void MiniPC_Send();

uint8_t MiniPC_Verify(uint16_t& _checksum, uint8_t* buff, uint16_t size);

inline os::MsgQueue<Data, 3> usb_rec_queue;
}  // namespace minipc

#endif  // SOL_CHASSIS_MINIPC_H