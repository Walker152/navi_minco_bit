#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/event_status.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("event_status_pub_test");
  auto pub = node->create_publisher<ros_interfaces::msg::EventStatus>("/sentry/event_status", 10);

  ros_interfaces::msg::EventStatus msg;
  msg.self_health = 90.0;
  msg.enemy_outpost_health = 1500.0;
  msg.own_outpost_destroyed = false;

  msg.buff_active = false;
  msg.enemy_detected.is_detect = false;
  msg.enemy_detected.position.x = 0.1;
  msg.enemy_detected.position.y = 0.3;
  msg.enemy_detected.position.z = 8.0;
  msg.enemy_detected.armor_id = 3;

  rclcpp::Rate rate(1);  // 1Hz
  int count = 0;
  while(rclcpp::ok())
  {
    // if(count == 15)
    // {
    //   msg.self_health = 100.0;
    // }
    // else if(count == 30)
    // {
    //   msg.enemy_detected.is_detect = true;
    // }
    // else if(count == 45)
    // {
    //   msg.self_health = 350.0;
    // }
    // else if(count == 60)
    // {
    //   msg.enemy_outpost_health = 0.0;
    // }
    // else if(count == 90)
    // {
    //   msg.enemy_detected.is_detect = false;
    // }
    // else if(count == 105)
    // {
    //   msg.self_health = 110.0;
    // }
    // else if(count == 120)
    // {
    //   msg.self_health = 350.0;
    // }
    pub->publish(msg);
    std::cout << "health: " << msg.self_health << ", buff: " << msg.buff_active
              << ", enemy: " << (msg.enemy_detected.is_detect ? "true" : "false")
              << ", outpost: " << (msg.own_outpost_destroyed ? "true" : "false")
              << ", enemy_outpost_health: " << msg.enemy_outpost_health << std::endl;
    count += 5;
    RCLCPP_INFO(node->get_logger(), "Published test EventStatus");
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
