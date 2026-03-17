#pragma once

#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
      blackboard_->set("gimbal_yaw", 0.0f);                                 // 云台偏航角
      blackboard_->set<uint16_t>("num_shoot", 0);                           // 已发射数量
      blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL));  // 当前模式
      blackboard_->set("nav_goal", Point2D{0.0, 0.0});                      // 当前导航目标
      blackboard_->set("patrol_index", 0);                                  // 巡逻点索引
      blackboard_->set("patrol_wait_time", 1000);                           // 巡逻等待时间（毫秒）
      blackboard_->set<SentryStance>("current_stance", SentryStance::MOVE);
      blackboard_->set<SentryStance>("desired_stance", SentryStance::MOVE);
      
      blackboard_->set("outpost_msg", false);                               // 抬头
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