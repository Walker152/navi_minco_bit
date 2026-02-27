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
    if(time_counter >= 105)
    {
      RCLCPP_INFO(node->get_logger(), "\n===============================");
      RCLCPP_INFO(node->get_logger(), "所有测试完成");
      RCLCPP_INFO(node->get_logger(), "===============================");
      rclcpp::shutdown();
      continue;
    }

    int phase = time_counter / 15;

    if(phase == 0)  // [0, 15)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 0: 测试常规巡逻与兜底姿态 (预期: 导航->RegularPatrol, 姿态->MOVE)");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = false;
      msg.position = 1;
    }
    else if(phase == 1)  // [15, 30)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 1: 测试索敌打断 (预期: 导航打断巡逻切为 TargetPursuit, 姿态切为 ATTACK)");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = true;
      msg.enemy_detected.position.x = 1700;
      msg.enemy_detected.position.y = 0.0;
      msg.enemy_detected.position.z = 10700;
      if(time_counter == 18)
      {
        msg.position = 2;
      }
    }
    else if(phase == 2)  // [30, 45)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 2: 视觉丢失防抖测试 (预期: 前1秒依然保持追击，1秒后打断追击回退到 RegularPatrol，姿态切回 MOVE)");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = false;
      if(time_counter == 35)
      {
        msg.position = 1;
      }
    }
    else if(phase == 3)  // [45, 60)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 3: 优先级跃升测试 (预期: 导航打断巡逻，进入 OutpostResponse 前往目标 2)");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_detected.is_detect = false;
      msg.enemy_outpost_destroyed = false;
    }
    else if(phase == 4)  // [60, 75)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 4: 撤退最高优打断 (预期: 强行打断前哨站响应，进入 EmergencyRetreat 前往 HOME)");
        last_phase = phase;
      }
      msg.self_health = 45.0;
      msg.enemy_detected.is_detect = false;
      msg.enemy_outpost_destroyed = false;
    }
    else if(phase == 5)  // [75, 90)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 5: 濒死防御测试 (预期: 导航保持撤退不变，但姿态瞬间被最高优先级 CheckDPCondition 捕获，切为 DEFEND)");
        last_phase = phase;
      }
      msg.self_health = 25.0;
      msg.enemy_detected.is_detect = false;
      msg.enemy_outpost_destroyed = false;
      if(time_counter == 80)
      {
        msg.position = 3;
      }
    }
    else  // phase == 6, [90, 105)
    {
      if(last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(),
                    "Phase 6: 状态重置测试 (预期: 撤退解除，因为 enemy_outpost_destroyed=false，导航恢复为 OutpostResponse，姿态恢复 MOVE)");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = false;
      msg.enemy_detected.is_detect = false;
      msg.position = 1;
    }

    // 发布消息
    pub->publish(msg);

    // 输出当前状态
    std::cout << "[t=" << time_counter << "s][Phase " << phase << "] "
              << "self_health=" << msg.self_health
              << ", enemy_outpost_destroyed=" << (msg.enemy_outpost_destroyed ? "true" : "false")
              << ", own_outpost_health=" << msg.own_outpost_health
              << ", is_detect=" << (msg.enemy_detected.is_detect ? "true" : "false")
              << ", position=" << static_cast<int>(msg.position)
              << ", armor_id=" << static_cast<int>(msg.enemy_detected.armor_id);

    if(msg.enemy_detected.is_detect)
    {
      std::cout << ", enemy_pos(m)=(" << msg.enemy_detected.position.x / 1000.0 << ", "
                << msg.enemy_detected.position.y / 1000.0 << ", "
                << msg.enemy_detected.position.z / 1000.0 << ")";
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
