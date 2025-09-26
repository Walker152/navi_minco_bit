#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "nav_zone.hpp"

namespace Sentry_BT
{

class Blackboard
{
private:
    std::shared_ptr<BT::Blackboard> blackboard_;

public:
    Blackboard(){
        blackboard_ = BT::Blackboard::create();

        // 初始化黑板变量
        blackboard_->set("health", 100.0f); // 初始生命值
        blackboard_->set("outpost_destroyed", false); // 前哨站状态
        blackboard_->set("bonus_active", false); // 前哨站增益状态
        blackboard_->set("target_valid", false); // 目标锁定状态
        blackboard_->set("target_in_range", false); // 目标是否在攻击范围内
        blackboard_->set("target_pose", geometry_msgs::msg::Pose()); // 目标位置
        blackboard_->set("target_armor_id", -1); // 目标装甲板ID
        blackboard_->set("nav_status", static_cast<int>(NavStatus::IDLE)); // 导航状态
        blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL)); // 当前模式
        blackboard_->set("nav_goal", Point2D{0.0, 0.0}); // 当前导航目标
        blackboard_->set("patrol_index", 0); // 巡逻点索引
    }
    
    template <typename T>
    void set(const std::string& key, const T& value) {
        blackboard_->set(key, value);
    }
    
    template <typename T>
    auto get(const std::string& key) {
        return blackboard_->get<T>(key);
    }
    
    std::shared_ptr<BT::Blackboard> getBTBlackboard() {
        return blackboard_;
    }
    
    void updateHealth(float health) {
        set("health", health);
    }
    
    void updateOutpostStatus(bool destroyed) {
        set("outpost_destroyed", destroyed);
    }
    
    void updateBonusStatus(bool active) {
        set("bonus_active", active);
    }
    
    void updateTargetInfo(bool valid, const geometry_msgs::msg::Point& position, int armor_id) {
        set("target_valid", valid);
        if (valid) {
            geometry_msgs::msg::Pose pose;
            pose.position = position;
            set("target_pose", pose);
            set("target_armor_id", armor_id);
        }
    }
    
    void updateNavStatus(NavStatus status) {
        set("nav_status", static_cast<int>(status));
    }
};

} // namespace Sentry_BT