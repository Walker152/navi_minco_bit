#pragma once

#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/bt_factory.h>
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
      blackboard_->set("health", 400.0f);                                   // 初始生命值(百分比)
      blackboard_->set<int>("own_outpost_health", 1500);                    // 我方前哨站血量
      blackboard_->set<bool>("enemy_outpost_destroyed", false);             // 敌方前哨站是否被摧毁
      blackboard_->set("target_valid", false);                              // 目标锁定状态
      blackboard_->set("target_pose", geometry_msgs::msg::Pose());          // 目标位置
      blackboard_->set("target_armor_id", -1);                              // 目标装甲板ID
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