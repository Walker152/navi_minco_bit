#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/event_status.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("event_status_pub_test");
    auto pub = node->create_publisher<robot_msgs::msg::EventStatus>("/sentry/event_status", 10);

    robot_msgs::msg::EventStatus msg;
    msg.self_health = 81.0;
    msg.own_outpost_destroyed = true;
    msg.buff_active = false;
    msg.enemy_detected.is_get = true;
    msg.enemy_detected.position.x = 0.1;
    msg.enemy_detected.position.y = 0.3;
    msg.enemy_detected.position.z = 8.0;
    msg.enemy_detected.armor_id = 3;

    rclcpp::Rate rate(1); // 1Hz
    while (rclcpp::ok()) {
        pub->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published test EventStatus");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
