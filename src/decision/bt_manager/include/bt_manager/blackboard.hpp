#pragma once

#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace Sentry_BT {
class Blackboard
{
private:
  std::shared_ptr<BT::Blackboard> blackboard_;

public:
  Blackboard()
  {
    blackboard_ = BT::Blackboard::create();

    // 初始化黑板变量
    // === Tree Internal Data ===

    // --- Stance Tree ---
    blackboard_->set<SentryStance>("current_stance", SentryStance::MOVE);  // 当前姿态
    blackboard_->set<SentryStance>("desired_stance", SentryStance::MOVE);  // 期望姿态
    blackboard_->set<LifterPos>("desired_lifter_pos", LifterPos::BOTTOM);  // 目标升降位置

    // --- Navigation Tree ---
    blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL));  // 当前模式
    blackboard_->set("nav_goal", Sentry_BT::Point2D{0.0, 0.0, 0.0});      // 当前导航目标
    blackboard_->set("patrol_index", 0);                                  // 巡逻点索引
    blackboard_->set("outpost_msg", false);                               // 抬头
    blackboard_->set("through_tunnel", false);                            // 是否通过隧道
    blackboard_->set("cmd_vel", geometry_msgs::msg::Twist());             // 速度指令
    blackboard_->set("current_pose", geometry_msgs::msg::Pose());         // 当前位姿缓存
    blackboard_->set("ammo_purchase_total", 0);                           // 累计买弹请求量
    blackboard_->set("ammo_purchase_request", 0);                         // 单次买弹请求量

    // --- Gimbal Tree ---
    blackboard_->set("target_gimbal_pan", 0.0f);    // 云台目标偏航
    blackboard_->set("target_gimbal_tilt", 0.0f);   // 云台目标俯仰
    blackboard_->set("scan_yaw_min_deg", -180.0f);  // 巡检范围最小偏航
    blackboard_->set("scan_yaw_max_deg", 180.0f);   // 巡检范围最大偏航
    blackboard_->set("scan_pitch_deg", 0.0f);       // 巡检俯仰

    // --- Tactical Tree ---
    blackboard_->set<TacticalMode>("tactical_mode", TacticalMode::BALANCED);  // 战术模式
    blackboard_->set("stance_refresh_required", false);                        // 姿态惩罚刷新标志

    // === External Info (From Referee/Lower Computer) ===

    // --- Team Info ---
    // team_info
    blackboard_->set("allies_info", std::vector<AllyRobotInfo>());  // 队友信息列表
    blackboard_->set("home_health", 3000);                          // 基地血量
    blackboard_->set("own_outpost_health", 1500);                   // 前哨站血量

    // --- Game Info ---
    blackboard_->set("game_time_remaining", 420);   // 比赛剩余时间（秒）
    blackboard_->set("coin_remaining", 0);          // 我方金币剩余数量
    blackboard_->set("small_energy_status", 0);     // 小能量机关状态
    blackboard_->set("big_energy_status", 0);       // 大能量机关状态
    blackboard_->set("fort_occupation_status", 0);  // 堡垒占领状态
    blackboard_->set("game_status", 0);             // 比赛状态

    // --- Radar Info ---
    blackboard_->set("enemy_coin_left", 0);                           // 敌方金币剩余数量
    blackboard_->set("enemy_coin_accumulated", 0);                    // 敌方金币累计数量
    blackboard_->set("enemies_info", std::vector<EnemyRobotInfo>());  // 敌方信息列表
    blackboard_->set("enemy_outpost_destroyed", true);                // 敌方前哨站是否被摧毁

    // --- Sentry Offline Info ---
    blackboard_->set("target_valid", false);                               // 目标锁定状态
    blackboard_->set("gimbal_yaw", 0.0f);                                  // 云台偏航角
    blackboard_->set<LifterPos>("lifter_current_pos", LifterPos::BOTTOM);  // 升降机构当前位置
    blackboard_->set("is_transformable", true);                            // 是否可变形
    blackboard_->set("transform_state", 0.0f);                             // 变形状态
    blackboard_->set("target_pose", geometry_msgs::msg::Pose());           // 目标位置
    blackboard_->set("target_armor_id", -1);                               // 目标装甲板ID

    // --- Sentry Online Info ---
    blackboard_->set("health", 100.0f);                // 初始生命值(百分比)
    blackboard_->set("bullets_remaining", 300);          // 剩余子弹数量
    blackboard_->set("cooling_value", 0);              // 冷却值
    blackboard_->set("heat_limit", 0);                 // 热量上限
    blackboard_->set("current_heat", 0);               // 当前热量
    blackboard_->set("speed_monitor_angle", 0.0f);     // 速度监测角度
    blackboard_->set("is_disengaged", true);           // 是否脱战状态
    blackboard_->set("can_activate_energy", false);    // 是否能激活能量机关
    blackboard_->set("can_free_resurrect", false);     // 是否能免费复活
    blackboard_->set("can_instant_resurrect", false);  // 是否能立即复活
    blackboard_->set("instant_resurrect_cost", 0);     // 立即复活的金币成本
  }

  template <typename T> void set(const std::string & key, const T & value) { blackboard_->set(key, value); }

  template <typename T> auto get(const std::string & key) { return blackboard_->get<T>(key); }

  std::shared_ptr<BT::Blackboard> getBTBlackboard() { return blackboard_; }
};

}  // namespace Sentry_BT