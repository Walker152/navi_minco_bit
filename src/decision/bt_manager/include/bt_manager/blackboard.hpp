#pragma once

#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace Sentry_BT
{
  class Blackboard
  {
  private:
    std::shared_ptr<BT::Blackboard> blackboard_;

  public:
    Blackboard()
    {
      blackboard_ = BT::Blackboard::create();

      // 初始化黑板变量
      blackboard_->set("health", 100.0f);                                   // 初始生命值(百分比)
      blackboard_->set<int>("own_outpost_health", 1500);                    // 我方前哨站血量
      blackboard_->set<bool>("enemy_outpost_destroyed", true);             // 敌方前哨站是否被摧毁
      blackboard_->set("target_valid", false);                              // 目标锁定状态
      blackboard_->set("target_pose", geometry_msgs::msg::Pose());          // 目标位置
      blackboard_->set("target_armor_id", -1);                              // 目标装甲板ID
      blackboard_->set("bonus_active", false);                              // 增益区是否激活
      blackboard_->set("game_status", 0);                                   // 比赛状态
      blackboard_->set("lifter_pos_now", 0);                                // 升降机构当前位置
      blackboard_->set("desired_lifter_pos", 0);                         // 目标升降位置
      blackboard_->set("gimbal_yaw", 0.0f);                                 // 云台偏航角
      blackboard_->set("num_shoot", 0);                           // 已发射数量
      blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL));  // 当前模式
      blackboard_->set("nav_goal", Point2D{0.0, 0.0});                      // 当前导航目标
      blackboard_->set("patrol_index", 0);                                  // 巡逻点索引
      blackboard_->set("patrol_wait_time", 1000);                           // 巡逻等待时间（毫秒）
      blackboard_->set<SentryStance>("current_stance", SentryStance::MOVE);
      blackboard_->set<SentryStance>("desired_stance", SentryStance::MOVE);                           
      blackboard_->set("outpost_msg", false);                               // 抬头
      blackboard_->set("through_tunnel", false);                            // 是否通过隧道
      blackboard_->set("cmd_vel", geometry_msgs::msg::Twist());             // 速度指令
      // new data
      //team_info
      blackboard_->set("allies_info", std::vector<AllyRobotInfo>());        // 队友信息列表
      blackboard_->set("home_health", 3000);                    // 基地血量
      //game_info
      blackboard_->set("game_time_remaining", 420);                    // 比赛剩余时间（秒）
      blackboard_->set("coin_remaining", 0);                    // 我方金币剩余数量
      blackboard_->set("small_energy_status", 0);                    // 小能量机关状态
      blackboard_->set("big_energy_status", 0);                    // 大能量机关状态
      blackboard_->set("fort_occupation_status", 0);                    // 堡垒占领状态
      //radar_info
      blackboard_->set("enemy_coin_left", 0);                   // 敌方金币剩余数量
      blackboard_->set("enemy_coin_accumulated", 0);                   // 敌方金币累计数量
      blackboard_->set("enemies_info", std::vector<EnemyRobotInfo>());  // 敌方信息列表
      //sentry_offline_info
      blackboard_->set("is_transformable", true);  // 是否可变形
      blackboard_->set("transform_state", 0.0f);  // 变形状态
      //sentry_online_info
       blackboard_->set("bullets_remaining", 0);  // 剩余子弹数量
      blackboard_->set("cooling_value", 0);  // 冷却值
      blackboard_->set("heat_limit", 0);  // 热量上限
      blackboard_->set("current_heat", 0);  // 当前热量
      blackboard_->set("speed_monitor_angle", 0.0f);  // 速度监测角度
      blackboard_->set("is_disengaged", true);                   // 是否脱战状态
      blackboard_->set("can_activate_energy", false);  // 是否能激活能量机关
      blackboard_->set("can_free_resurrect", false);  // 是否能免费复活
      blackboard_->set("can_instant_resurrect", false);  // 是否能立即复活
      blackboard_->set("instant_resurrect_cost", 0);  // 立即复活的金币成本
    }

    template <typename T> void set(const std::string& key, const T& value)
    {
      blackboard_->set(key, value);
    }

    template <typename T> auto get(const std::string& key)
    {
      return blackboard_->get<T>(key);
    }

    std::shared_ptr<BT::Blackboard> getBTBlackboard()
    {
      return blackboard_;
    }

    void updateHealth(float health)
    {
      set("health", health);
    }

    void updateBonusStatus(bool active)
    {
      set("bonus_active", active);
    }

    void updateTargetInfo(bool valid, const geometry_msgs::msg::Point& position, int armor_id)
    {
      set("target_valid", valid);
      if(valid)
      {
        geometry_msgs::msg::Pose pose;
        pose.position = position;
        set("target_pose", pose);
        set("target_armor_id", armor_id);
      }
    }

  };

}  // namespace Sentry_BT