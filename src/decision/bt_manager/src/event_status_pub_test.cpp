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
  msg.enemy_outpost_health = 1500;
  msg.own_outpost_destroyed = false;
  msg.buff_active = false;
  msg.enemy_detected.is_detect = false;
  msg.enemy_detected.position.x = 0.1;
  msg.enemy_detected.position.y = 0.3;
  msg.enemy_detected.position.z = 8.0;
  msg.enemy_detected.armor_id = 3;

  RCLCPP_INFO(node->get_logger(), "初始状态：血量=400.0，前哨站存在，无敌人，巡逻状态");

  rclcpp::Rate rate(1);  // 1Hz
  int time_counter = 0;  // 时间计数器（秒）
  
  while(rclcpp::ok())
  {
    // 测试场景1: 攻击前哨站 -> 掉血 -> 撤退 -> 回家恢复
    if(time_counter == 0)
    {
      RCLCPP_INFO(node->get_logger(), "===============================");
      RCLCPP_INFO(node->get_logger(), "测试场景1: 攻击前哨站测试");
      RCLCPP_INFO(node->get_logger(), "===============================");
      RCLCPP_INFO(node->get_logger(), "开始攻击前哨站，初始血量400");
      msg.self_health = 400.0;
      msg.enemy_detected.is_detect = true;
      msg.enemy_detected.position.x = 7.0;
      msg.enemy_detected.position.y = 0.5;
      RCLCPP_INFO(node->get_logger(), "预期行为: 开始攻击前哨站");
    }
    else if(time_counter == 20)
    {
      msg.self_health = 190.0;
      msg.enemy_outpost_health = 0.0;  // 前哨站被摧毁
      RCLCPP_INFO(node->get_logger(), "攻击20秒后，血量: 190.0 (低于200，应该撤退)");
      RCLCPP_INFO(node->get_logger(), "预期行为: 开始撤退回家");
    }
    else if(time_counter == 25)
    {
      msg.self_health = 180.0;
      msg.enemy_detected.is_detect = false;  // 撤退中，不再检测敌人
      RCLCPP_INFO(node->get_logger(), "撤退5秒后，血量: 180.0 ");
    }
    else if(time_counter == 30)
    {
      msg.self_health = 170.0;
      RCLCPP_INFO(node->get_logger(), "撤退10秒后，血量: 170.0 ");
    }

    else if(time_counter == 50)
    {
      msg.self_health = 320.0;  // 达到恢复阈值
      RCLCPP_INFO(node->get_logger(), "回家后，血量: 320.0 (达到恢复阈值)");
      RCLCPP_INFO(node->get_logger(), "预期行为: 停止恢复，重新开始巡逻");
    }
    else if(time_counter == 55)
    {
      msg.self_health = 350.0;
      msg.enemy_detected.is_detect = false;
      RCLCPP_INFO(node->get_logger(), "血量: 350.0 (健康，巡逻状态)");
    }
    
    // 测试场景2: 前哨站被摧毁后的巡逻
    else if(time_counter == 60)
    {
      RCLCPP_INFO(node->get_logger(), "\n===============================");
      RCLCPP_INFO(node->get_logger(), "测试场景2: 前哨站被摧毁后的巡逻测试");
      RCLCPP_INFO(node->get_logger(), "===============================");
      msg.self_health = 400.0;
      msg.enemy_outpost_health = 0.0;  // 前哨站被摧毁
      msg.enemy_detected.is_detect = false;
      RCLCPP_INFO(node->get_logger(), "前哨站已被摧毁，血量: 400.0");
      RCLCPP_INFO(node->get_logger(), "预期行为: 巡逻状态 (前哨站被摧毁，无攻击目标)");
    }
    
    // 前哨站摧毁后，持续巡逻（50秒）
    else if(time_counter == 70)
    {
      msg.enemy_detected.is_detect = false;
      RCLCPP_INFO(node->get_logger(), "持续巡逻10秒... 无敌人检测");
    }
    else if(time_counter == 80)
    {
      RCLCPP_INFO(node->get_logger(), "持续巡逻20秒... 保持警戒状态");
    }

    else if(time_counter == 100)
    {
      msg.enemy_detected.is_detect = false;
      RCLCPP_INFO(node->get_logger(), "巡逻40秒，继续巡逻");
    }
    else if(time_counter == 110)
    {
      RCLCPP_INFO(node->get_logger(), "巡逻50秒，测试完成，保持巡逻状态");
      RCLCPP_INFO(node->get_logger(), "巡逻测试完成");
    }
    
    // 结束测试
    else if(time_counter == 120)
    {
      RCLCPP_INFO(node->get_logger(), "\n===============================");
      RCLCPP_INFO(node->get_logger(), "所有测试完成");
      RCLCPP_INFO(node->get_logger(), "===============================");
      break;
    }
    
    // 发布消息
    pub->publish(msg);
    
    // 输出当前状态
    std::cout << "\n[" << time_counter << "秒] "
              << "血量: " << msg.self_health 
              << ", 前哨站血量: " << msg.enemy_outpost_health
              << ", 敌人检测: " << (msg.enemy_detected.is_detect ? "是" : "否");
    
    if(msg.enemy_detected.is_detect) {
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
