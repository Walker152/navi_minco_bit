#include <chrono>
#include <cstdint>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>

namespace
{
uint16_t buildSentryInfo2(bool is_disengaged, uint8_t current_stance_bits, bool can_activate_energy)
{
  uint16_t value = 0;
  if (is_disengaged)
  {
    value |= 0x0001;
  }
  value |= static_cast<uint16_t>((current_stance_bits & 0x3) << 12);
  if (can_activate_energy)
  {
    value |= static_cast<uint16_t>(1U << 14);
  }
  return value;
}

uint32_t buildSentryInfo1(bool can_free_resurrect, bool can_instant_resurrect, uint16_t instant_resurrect_cost)
{
  uint32_t value = 0;
  if (can_free_resurrect)
  {
    value |= (1U << 19);
  }
  if (can_instant_resurrect)
  {
    value |= (1U << 20);
  }
  value |= (static_cast<uint32_t>(instant_resurrect_cost & 0x03FF) << 21);
  return value;
}

uint32_t buildEventCode(uint8_t small_energy_status, uint8_t big_energy_status, uint8_t fort_occupation_status)
{
  uint32_t value = 0;
  value |= (static_cast<uint32_t>(small_energy_status & 0x3) << 3);
  value |= (static_cast<uint32_t>(big_energy_status & 0x3) << 5);
  value |= (static_cast<uint32_t>(fort_occupation_status & 0x3) << 25);
  return value;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("event_status_test");

  auto team_pub = node->create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
  auto game_pub = node->create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
  auto radar_pub = node->create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);
  auto offline_pub = node->create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
  auto online_pub = node->create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);

  RCLCPP_INFO(node->get_logger(), "=========================================================");
  RCLCPP_INFO(node->get_logger(), "哨兵响应式行为树业务逻辑压测启动 (10Hz / multi-topic)");
  RCLCPP_INFO(node->get_logger(), "topics: /sentry/{team_info,game_info,radar_info,offline_info,online_info}");
  RCLCPP_INFO(node->get_logger(), "=========================================================");

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

    ros_interfaces::msg::TeamInformation team_msg;
    ros_interfaces::msg::GameInfo game_msg;
    ros_interfaces::msg::RadarInfo radar_msg;
    ros_interfaces::msg::SentryInfoOffline offline_msg;
    ros_interfaces::msg::SentryInfoOnline online_msg;

    const auto stamp = node->now();
    team_msg.header.stamp = stamp;
    game_msg.header.stamp = stamp;
    radar_msg.header.stamp = stamp;
    offline_msg.header.stamp = stamp;
    online_msg.header.stamp = stamp;

    // Baseline defaults each tick; phases override targeted fields.
    int phase = -1;

    team_msg.outpost_hp = 1500;
    team_msg.base_hp = 3000;
    for (size_t i = 0; i < team_msg.allies.size(); ++i)
    {
      auto& ally = team_msg.allies[i];
      ally.armor_id = static_cast<uint8_t>(i + 1);
      ally.remain_hp = 200;
      ally.position.x = 0.0;
      ally.position.y = 0.0;
      ally.position.z = 0.0;
    }

    game_msg.game_time_remaining = static_cast<uint16_t>(std::max(0, 420 - sec));
    game_msg.coin_remaining = 60;
    game_msg.game_status = 4;
    game_msg.event_code = buildEventCode(0, 0, 0);

    radar_msg.enemy_coin_left = 30;
    radar_msg.enemy_coin_accumulated = 120;
    radar_msg.is_enemy_outpost_sensed = false;  // false => enemy_outpost_destroyed=true
    for (auto& enemy : radar_msg.enemies)
    {
      enemy.robot_id = 0;
      enemy.robot_hp = 0;
      enemy.allowed_projectile = 0;
      enemy.position.x = 0.0;
      enemy.position.y = 0.0;
      enemy.position.z = 0.0;
    }

    offline_msg.is_get = false;
    offline_msg.armor_num = 3;
    offline_msg.armor_pos.x = 0.0;  // mm
    offline_msg.armor_pos.y = 0.0;  // mm
    offline_msg.armor_pos.z = 0.0;  // mm
    offline_msg.yaw_imu = 0.0F;
    offline_msg.lifter_current_pos = 0;
    offline_msg.is_transformable = true;
    offline_msg.transform_state = 0.0F;

    online_msg.self_health = 1600;  // 1600 / 4 => blackboard health = 400
    online_msg.bullets_remaining = 120;
    online_msg.cooling_value = 40;
    online_msg.heat_limit = 200;
    online_msg.current_heat = 20;
    online_msg.sentry_pos.x = 0.0;
    online_msg.sentry_pos.y = 0.0;
    online_msg.sentry_pos.z = 0.0;
    online_msg.speed_monitor_angle = 0.0F;
    online_msg.sentry_info_2 = buildSentryInfo2(true, 0, false);
    online_msg.sentry_info_1 = buildSentryInfo1(false, false, 0);

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

      offline_msg.is_get = true;
      offline_msg.armor_pos.x = 10000.0;  // 10m, likely outside attack area
      offline_msg.armor_pos.y = 10000.0;
      offline_msg.armor_pos.z = 0.0;
    }
    else if (time_counter < 300)  // Phase 2: [20, 30s)
    {
      phase = 2;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 2 [20-30s] 正常追击与进攻姿态");
        RCLCPP_INFO(node->get_logger(), "预期: CheckTargetLocked通过, current_mode进入TRACING, 姿态ATTACK");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      offline_msg.is_get = true;
      offline_msg.armor_pos.x = 6000.0;  // 6m
      offline_msg.armor_pos.y = 4000.0;  // 4m (inside attack_area when transform unavailable)
      offline_msg.armor_pos.z = 0.0;
    }
    else if (time_counter < 400)  // Phase 3: [30, 40s)
    {
      phase = 3;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 3 [30-40s] 1.0秒视觉防抖测试");
        RCLCPP_INFO(node->get_logger(), "预期: 30-31s短时丢目标仍保持TRACING, 31s后回落");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      // Keep target pose in attack area, only toggle validity to test debounce path.
      offline_msg.armor_pos.x = 6000.0;
      offline_msg.armor_pos.y = 4000.0;
      offline_msg.armor_pos.z = 0.0;
      offline_msg.is_get = (time_counter >= 310);
    }
    else if (time_counter < 500)  // Phase 4: [40, 50s)
    {
      phase = 4;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 4 [40-50s] 前哨站响应抢占");
        RCLCPP_INFO(node->get_logger(), "预期: CheckOutpostRemained成功, 进入RESPONSE导航");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      radar_msg.is_enemy_outpost_sensed = true;  // true => enemy_outpost_destroyed=false
      offline_msg.is_get = false;
    }
    else if (time_counter < 600)  // Phase 5: [50, 60s)
    {
      phase = 5;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 5 [50-60s] 紧急撤退与防御姿态");
        RCLCPP_INFO(node->get_logger(), "预期: EmergencyRetreat + CheckDPCondition => DEFEND");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      online_msg.self_health = 180;  // 180 / 4 => 45
      online_msg.sentry_info_2 = buildSentryInfo2(false, 2, false);
      offline_msg.is_get = false;
      radar_msg.is_enemy_outpost_sensed = false;
    }
    else if (time_counter < 700)  // Phase 6: [60, 70s)
    {
      phase = 6;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 6 [60-70s] 回血重置撤退");
        RCLCPP_INFO(node->get_logger(), "预期: 血量>recovery_threshold后退出RETREAT");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      online_msg.self_health = 360;  // 360 / 4 => 90 (> 80)
      online_msg.sentry_info_2 = buildSentryInfo2(true, 0, true);
      game_msg.coin_remaining = 120;
      game_msg.event_code = buildEventCode(1, 0, 1);
      online_msg.sentry_info_1 = buildSentryInfo1(true, false, 0);
    }
    else  // Phase 7: [70, 80s)
    {
      phase = 7;
      if (last_phase != phase)
      {
        RCLCPP_INFO(node->get_logger(), "\n=================================================");
        RCLCPP_INFO(node->get_logger(), "Phase 7 [70-80s] 收尾综合数据");
        RCLCPP_INFO(node->get_logger(), "预期: 常规巡逻，同时验证经济/机关/复活位解码字段");
        RCLCPP_INFO(node->get_logger(), "=================================================");
        last_phase = phase;
      }

      game_msg.coin_remaining = 180;
      game_msg.event_code = buildEventCode(2, 1, 2);
      radar_msg.enemy_coin_left = 5;
      radar_msg.enemy_coin_accumulated = 300;
      online_msg.sentry_info_1 = buildSentryInfo1(true, true, 80);
      online_msg.sentry_info_2 = buildSentryInfo2(true, 1, true);
      offline_msg.is_get = true;
      offline_msg.armor_pos.x = 6500.0;
      offline_msg.armor_pos.y = 4500.0;
      offline_msg.armor_pos.z = 0.0;
    }

    // Fill one ally and one enemy entry with meaningful values for blackboard inspection.
    team_msg.allies[0].armor_id = 1;
    team_msg.allies[0].remain_hp = 180;
    team_msg.allies[0].position.x = 2.0;
    team_msg.allies[0].position.y = -1.0;
    team_msg.allies[0].position.z = 0.0;

    radar_msg.enemies[0].robot_id = 101;
    radar_msg.enemies[0].robot_hp = 220;
    radar_msg.enemies[0].allowed_projectile = 90;
    radar_msg.enemies[0].position.x = 8.0;
    radar_msg.enemies[0].position.y = 3.0;
    radar_msg.enemies[0].position.z = 0.0;

    team_pub->publish(team_msg);
    game_pub->publish(game_msg);
    radar_pub->publish(radar_msg);
    offline_pub->publish(offline_msg);
    online_pub->publish(online_msg);

    if (time_counter % 10 == 0)
    {
      std::cout << "[t=" << sec << "s][Phase " << phase << "] "
                << "self_health_raw=" << online_msg.self_health
                << ", target_valid=" << (offline_msg.is_get ? "true" : "false")
                << ", target_gimbal_mm(x,y,z)=(" << offline_msg.armor_pos.x << ", "
                << offline_msg.armor_pos.y << ", " << offline_msg.armor_pos.z << ")"
                << ", enemy_outpost_sensed=" << (radar_msg.is_enemy_outpost_sensed ? "true" : "false")
                << ", game_event_code=" << game_msg.event_code
                << std::endl;
    }

    ++time_counter;
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "测试程序正常结束");
  rclcpp::shutdown();
  return 0;
}
