#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/event_status.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("event_status_pub_test");
  auto pub = node->create_publisher<ros_interfaces::msg::EventStatus>("/sentry/event_status", 10);

  RCLCPP_INFO(node->get_logger(), "=========================================================");
  RCLCPP_INFO(node->get_logger(), "哨兵响应式行为树业务逻辑压测启动 (10Hz / EventStatus only)");
  RCLCPP_INFO(node->get_logger(), "=========================================================");

  ros_interfaces::msg::EventStatus msg;
  msg.self_health = 400.0;
  msg.enemy_outpost_destroyed = true;
  msg.own_outpost_health = 1500;
  msg.buff_active = false;
  msg.enemy_detected.is_detect = false;
  msg.enemy_detected.position.x = 0.0;
  msg.enemy_detected.position.y = 0.0;
  msg.enemy_detected.position.z = 0.0;
  msg.enemy_detected.armor_id = 3;
  msg.current_stance = 1;  // 1=MOVE, 2=ATTACK, 3=DEFEND
  msg.gimbal_yaw = 0.0;
  msg.lifter_pos_now = 0.0;

  // Coordinate convention reminder for test vectors:
  // enemy_detected.position is in gimbal frame, millimeters:
  // Z forward, X right, Y down.
  RCLCPP_INFO(node->get_logger(), "坐标约定: gimbal系, Z前/X右/Y下, 单位mm");

  rclcpp::Rate rate(10);
  int time_counter = 0;  // +1 = 0.1s, every 100 ticks = 10s phase
  int last_phase = -1;

  while (rclcpp::ok())
  {
    const int sec = time_counter / 10;

    if (time_counter >= 800)
    {
      RCLCPP_INFO(node->get_logger(), "\n===============================");
      RCLCPP_INFO(node->get_logger(), "所有测试完成");
      RCLCPP_INFO(node->get_logger(), "===============================");
      rclcpp::shutdown();
      continue;
    }

    // Default values each tick; phases override targeted fields.
    int phase = -1;
    msg.self_health = 400.0;
    msg.enemy_outpost_destroyed = true;
    msg.enemy_detected.is_detect = false;
    msg.enemy_detected.armor_id = 3;
    msg.enemy_detected.position.x = 0.0;
    msg.enemy_detected.position.y = 0.0;
    msg.enemy_detected.position.z = 0.0;
    msg.current_stance = 1;

    if (time_counter < 100)  // Phase 0: [0, 10s)
    {
      phase = 0;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 0 [0-10s] 基础巡逻与兜底姿态");
        RCLCPP_INFO(node->get_logger(), "预期: RegularPatrol + MOVE");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = false;
      msg.current_stance = 1;
    }
    else if (time_counter < 200)  // Phase 1: [10, 20s)
    {
      phase = 1;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 1 [10-20s] 区域外索敌拦截 (Out of Bounds)");
        RCLCPP_INFO(node->get_logger(), "预期: CheckTargetLocked拦截FAILURE, 保持巡逻");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.enemy_detected.is_detect = true;
      // Very far target in gimbal frame (10m ahead)
      msg.enemy_detected.position.z = 6000.0;
      msg.enemy_detected.position.x = -500.0;
      msg.enemy_detected.position.y = 0.0;
      msg.enemy_outpost_destroyed = true;
      msg.current_stance = 1;
    }
    else if (time_counter < 300)  // Phase 2: [20, 30s)
    {
      phase = 2;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 2 [20-30s] 正常追击 (>0.5m) 与进攻姿态");
        RCLCPP_INFO(node->get_logger(), "预期: TargetPursuit触发, 日志应出现前进提示, 姿态ATTACK");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.enemy_detected.is_detect = true;
      // Enemy 4m ahead in gimbal frame (mm)
      msg.enemy_detected.position.z = 4000.0;
      msg.enemy_detected.position.x = 0.0;
      msg.enemy_detected.position.y = 0.0;
      msg.enemy_outpost_destroyed = true;
      msg.current_stance = 2;
    }
    else if (time_counter < 400)  // Phase 3: [30, 40s)
    {
      phase = 3;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 3 [30-40s] 空间限频测试 (<0.5m跳过, >0.5m更新)");
        RCLCPP_INFO(node->get_logger(), "预期: 30-35s跳过目标下发, 35-40s重新Set target pose");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.enemy_detected.is_detect = true;
      msg.enemy_outpost_destroyed = true;
      msg.current_stance = 2;

      if (time_counter < 350)
      {
        // 30-35s: move 0.2m only -> should be filtered by 0.5m limiter
        msg.enemy_detected.position.x = 200.0;
      }
      else
      {
        // 35-40s: move 0.8m from previous 3.2m baseline -> should update
        msg.enemy_detected.position.x = 1000.0;
      }
      msg.enemy_detected.position.y = 0.0;
      msg.enemy_detected.position.z = 4000.0;
    }
    else if (time_counter < 500)  // Phase 4: [40, 50s)
    {
      phase = 4;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 4 [40-50s] 贴脸防卫与后退 (<=0.5m)");
        RCLCPP_INFO(node->get_logger(), "预期: 触发风筝后退, 日志应出现后退提示");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.enemy_detected.is_detect = true;
      msg.enemy_detected.position.z = 0.0;
      msg.enemy_detected.position.x = 400.0;
      msg.enemy_detected.position.y = 0.0;
      msg.enemy_outpost_destroyed = true;
      msg.current_stance = 2;
    }
    else if (time_counter < 600)  // Phase 5: [50, 60s)
    {
      phase = 5;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 5 [50-60s] 1.0秒视觉防抖测试");
        RCLCPP_INFO(node->get_logger(), "预期: 50-51s保留TRACING/ATTACK, 51s后平滑回巡逻/MOVE");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      if (time_counter == 500)
      {
        RCLCPP_INFO(node->get_logger(), "[Phase 5] t=50.0s: 瞬时丢失视觉目标 is_detect=false");
      }
      msg.enemy_detected.is_detect = false;
      msg.enemy_outpost_destroyed = true;

      // Simulate expected behavior timeline for observation convenience.
      if (time_counter < 510)
      {
        msg.current_stance = 2;  // ATTACK during debounce window
      }
      else
      {
        msg.current_stance = 1;  // MOVE after debounce timeout
      }
    }
    else if (time_counter < 700)  // Phase 6: [60, 70s)
    {
      phase = 6;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 6 [60-70s] 绝对优先: 紧急撤退与防御姿态");
        RCLCPP_INFO(node->get_logger(), "预期: EmergencyRetreat + CheckDPCondition => DEFEND");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.self_health = 45.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = false;
      msg.current_stance = 3;
    }
    else  // Phase 7: [70, 80s)
    {
      phase = 7;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 7 [70-80s] 回血重置");
        RCLCPP_INFO(node->get_logger(), "预期: 撤退解除, 回归常规巡逻");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }
      msg.self_health = 400.0;
      msg.enemy_outpost_destroyed = true;
      msg.enemy_detected.is_detect = false;
      msg.current_stance = 1;
    }

    pub->publish(msg);

    if (time_counter % 10 == 0)
    {
      std::cout << "[t=" << sec << "s][Phase " << phase << "] "
                << "health=" << msg.self_health
                << ", enemy_outpost_destroyed=" << (msg.enemy_outpost_destroyed ? "true" : "false")
                << ", is_detect=" << (msg.enemy_detected.is_detect ? "true" : "false")
                << ", enemy_gimbal_mm(x,y,z)=(" << msg.enemy_detected.position.x << ", "
                << msg.enemy_detected.position.y << ", " << msg.enemy_detected.position.z << ")"
                << ", current_stance=" << static_cast<int>(msg.current_stance)
                << std::endl;
    }

    time_counter++;
    rclcpp::spin_some(node);
    rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "测试程序正常结束");
  rclcpp::shutdown();
  return 0;
}
