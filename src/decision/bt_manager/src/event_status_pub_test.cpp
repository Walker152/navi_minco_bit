#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/event_status.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("event_status_pub_test");
  auto pub = node->create_publisher<ros_interfaces::msg::EventStatus>("/sentry/event_status", 10);

  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "行为树系统测试程序启动 - 重点测试场景");
  RCLCPP_INFO(node->get_logger(), "========================================");

  ros_interfaces::msg::EventStatus msg;
  msg.self_health = 400.0;
  msg.enemy_outpost_destroyed = false;
  msg.own_outpost_health = 1500;
  msg.buff_active = false;
  msg.enemy_detected.is_detect = false;
  msg.enemy_detected.position.x = 0.1;
  msg.enemy_detected.position.y = 0.3;
  msg.enemy_detected.position.z = 8.0;
  msg.enemy_detected.armor_id = 3;
  msg.position = 1;

  RCLCPP_INFO(node->get_logger(), "初始状态：血量=400.0，前哨站存在，无敌人，巡逻状态");

  rclcpp::Rate rate(1);  // 1Hz
  int time_counter = 0;  // 时间计数器（秒）
  int last_phase = -1;

  while(rclcpp::ok())
  {
    int phase = time_counter / 15;

    switch(phase)
    {
      case 0:  // [0, 15)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 1: 测试常规巡逻与移动姿态 (预期：进入 RegularPatrol 序列，姿态目标为 MOVE)");
          last_phase = phase;
        }
        msg.self_health = 400.0;
        msg.enemy_outpost_destroyed = false;
        msg.enemy_detected.is_detect = false;
        msg.position = 1;
        break;

      case 1:  // [15, 30)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 2: 测试目标追击与进攻姿态打断 (预期：打断巡逻，进入 TargetPursuit 序列，姿态目标为 ATTACK)");
          last_phase = phase;
        }
        msg.enemy_detected.is_detect = true;
        msg.enemy_detected.position.x = 6.0;
        msg.enemy_detected.position.y = 0.8;
        if(time_counter == 20)
        {
          msg.position = 2;
        }
        break;

      case 2:  // [30, 45)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 3: 测试前哨站响应与姿态切回 (预期：进入 OutpostResponse 序列，姿态目标切回 MOVE)");
          last_phase = phase;
        }
        msg.enemy_detected.is_detect = false;
        msg.enemy_outpost_destroyed = true;
        if(time_counter == 35)
        {
          msg.position = 1;
        }
        break;

      case 3:  // [45, 60)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 4: 测试紧急撤退最高优先级 (预期：强制打断所有行为，进入 EmergencyRetreat 序列，目标设为 HOME)");
          last_phase = phase;
        }
        msg.self_health = 45.0;
        msg.enemy_outpost_destroyed = false;
        msg.enemy_detected.is_detect = false;
        break;

      case 4:  // [60, 75)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 5: 测试濒死防御姿态 (预期：保持撤退序列，但姿态强制切换为 DEFEND)");
          last_phase = phase;
        }
        msg.self_health = 25.0;
        if(time_counter == 65)
        {
          msg.position = 3;
        }
        break;

      case 5:  // [75, 90)
        if(last_phase != phase)
        {
          RCLCPP_INFO(node->get_logger(), "Phase 6: 测试血量恢复后的行为降级 (预期：撤退条件解除，重新回到 RegularPatrol 序列，姿态切回 MOVE)");
          last_phase = phase;
        }
        msg.self_health = 400.0;
        msg.enemy_outpost_destroyed = false;
        msg.enemy_detected.is_detect = false;
        msg.position = 1;
        break;

      default:
        RCLCPP_INFO(node->get_logger(), "\n===============================");
        RCLCPP_INFO(node->get_logger(), "所有测试完成");
        RCLCPP_INFO(node->get_logger(), "===============================");
        rclcpp::shutdown();
        continue;
    }

    // 发布消息
    pub->publish(msg);

    // 输出当前状态
    std::cout << "\n[" << time_counter << "秒] "
              << "血量: " << msg.self_health 
              << ", 敌方前哨站是否摧毁: " << (msg.enemy_outpost_destroyed ? "是" : "否")
              << ", 我方前哨站血量: " << msg.own_outpost_health
              << ", 当前姿态(position): " << static_cast<int>(msg.position)
              << ", 敌人检测: " << (msg.enemy_detected.is_detect ? "是" : "否");

    if(msg.enemy_detected.is_detect)
    {
      std::cout << " 位置(" << msg.enemy_detected.position.x 
                << ", " << msg.enemy_detected.position.y << ")";
    }
    std::cout << std::endl;
    
    time_counter++;
    rclcpp::spin_some(node);
    rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "测试程序正常结束");
  rclcpp::shutdown();
  return 0;
}
