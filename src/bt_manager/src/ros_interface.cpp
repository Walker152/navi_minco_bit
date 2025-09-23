#include "bt_manager/ros_interface.hpp"

namespace Sentry_BT
{
ros_interface::ros_interface(std::shared_ptr<Blackboard> blackboard_ptr)
: Node("ros_interface_node"),
  blackboard_(blackboard_ptr)
{
    // 订阅事件状态话题
    event_sub = this->create_subscription<robots_msgs::msg::EventStatus>(
        "/sentry/event_status", 10,
        std::bind(&ros_interface::eventCallback, this, std::placeholders::_1));
}   

ros_interface::~ros_interface()
{   
}

void ros_interface::eventCallback(const robots_msgs::msg::EventStatus::SharedPtr msg)
{
    // 更新黑板中的数据
    blackboard_->updateHealth(msg->self_health);
    blackboard_->updateOutpostStatus(msg->outpost_desroyed);
    blackboard_->updateBonusStatus(msg->buff_active);
    blackboard_->updateTargetInfo(msg->enemy_detected.is_get, msg->enemy_detected.position, msg->enemy_detected.armor_id);
    RCLCPP_INFO(this->get_logger(), "EventStatus received: health=%.2f, outpost=%s, buff=%s, target_locked=%s, target_id=%d",
                msg->self_health,
                msg->outpost_desroyed ? "true" : "false",
                msg->buff_active ? "true" : "false",
                msg->enemy_detected.is_get ? "true" : "false",
                msg->enemy_detected.armor_id);
}
} // namespace Sentry_BT
