#include "bt_manager/blackboard.hpp"

namespace Sentry_BT
{
    Blackboard::Blackboard()
    : current_health_(100.0),
      outpost_destroyed_(false),
      fort_bonus_active_(false),
      target_locked_(false),
      target_id_(""),
      current_mode_("Patrol"),
      patrol_index_(0),
      last_inspection_time_(0.0),
      bt_blackboard_(BT::Blackboard::create())
    {
        // 初始化黑板数据
        initialize();
    }

    void Blackboard::updateHealth(float health)
    {
        current_health_ = health;
        bt_blackboard_->set("health", current_health_);
    }

    void Blackboard::updateOutpostStatus(bool destroyed)
    {
        outpost_destroyed_ = destroyed;
        bt_blackboard_->set("outpost_destroyed", outpost_destroyed_);
    }

    void Blackboard::updateBonusStatus(bool active)
    {
        fort_bonus_active_ = active;
        bt_blackboard_->set("fort_bonus_active", fort_bonus_active_);
    }

    void Blackboard::updateTargetInfo(bool locked, const geometry_msgs::msg::Point & pose, const int& id)
    {
        target_locked_ = locked;
        target_position_ = pose;
        target_id_ = id;

        bt_blackboard_->set("target_locked", target_locked_);
        bt_blackboard_->set("target_position", target_position_);
        bt_blackboard_->set("target_id", target_id_);
    }

    float Blackboard::getHealth() const
    {
        return current_health_;
    }

    geometry_msgs::msg::Pose Blackboard::getPosition() const
    {
        return geometry_msgs::msg::Pose();
    }

    bool Blackboard::isOutpostDestroyed() const
    {
        return outpost_destroyed_;
    }

    bool Blackboard::isBonusActive() const
    {
        return fort_bonus_active_;
    }

    bool Blackboard::isTargetLocked() const
    {
        return target_locked_;
    }

    geometry_msgs::msg::Point Blackboard::getTargetPosition() const
    {
        return target_position_;
    }

    std::string Blackboard::getTargetId() const
    {
        return target_id_;
    }

    void Blackboard::setCurrentMode(const std::string & mode)
    {
        current_mode_ = mode;
        bt_blackboard_->set("current_mode", current_mode_);
    }

    std::string Blackboard::getCurrentMode() const
    {
        return current_mode_;
    }

    void Blackboard::setNavigationGoal(const geometry_msgs::msg::Pose & goal)
    {
        navigation_goal_ = goal;
        bt_blackboard_->set("navigation_goal", navigation_goal_);
    }

    geometry_msgs::msg::Pose Blackboard::getNavigationGoal() const
    {
        return navigation_goal_;
    }

    BT::Blackboard::Ptr Blackboard::getBTBlackboard()
    {
        return bt_blackboard_;
    }

    void Blackboard::initialize()
    {
        current_health_ = 100.0;
        outpost_destroyed_ = false;
        fort_bonus_active_ = false;
        target_locked_ = false;
        target_position_ = geometry_msgs::msg::Point();
        target_id_ = "";
        current_mode_ = "Patrol";
        patrol_index_ = 0;
        last_inspection_time_ = 0.0;
        navigation_goal_ = geometry_msgs::msg::Pose();

        // 初始化BT黑板
        bt_blackboard_->set("health", current_health_);
        bt_blackboard_->set("outpost_destroyed", outpost_destroyed_);
        bt_blackboard_->set("fort_bonus_active", fort_bonus_active_);
        bt_blackboard_->set("target_locked", target_locked_);
        bt_blackboard_->set("target_position", target_position_);
        bt_blackboard_->set("target_id", target_id_);
        bt_blackboard_->set("current_mode", current_mode_);
        bt_blackboard_->set("navigation_goal", navigation_goal_);
    }
}  // namespace Sentry_BT