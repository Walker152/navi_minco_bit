#pragma once
#include "bt_manager/utils/nav_zone.hpp"
#include "bt_manager/utils/log.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_interfaces/msg/team_information.hpp"
#include "ros_interfaces/msg/game_info.hpp"
#include "ros_interfaces/msg/radar_info.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "ros_interfaces/msg/sentry_info_online.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"
#include "ros_interfaces/msg/behavior.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

class BlackboardTestNode : public BT::CoroActionNode
{
public:
  BlackboardTestNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    // 获取黑板指针
    auto blackboard = config().blackboard;

    if (!blackboard)
    {
      std::cout << "Blackboard is null!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // 定时打印黑板变量的函数
    auto printBlackboardVariables = [this, blackboard]() -> BT::NodeStatus
    {
      rclcpp::Rate rate(0.5); // 2秒一次
      while (true)
      {

        std::cout << "====================== Blackboard Variables ======================" << std::endl;
        std::cout << "Timestamp: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;

        // 尝试获取并打印所有可能的变量
        try
        {
          // 1. 从teamInfoCallback设置的变量
          try {
            auto team_info = blackboard->get<ros_interfaces::msg::TeamInformation>("team_info");
            std::cout << "team_info: [TeamInformation message]" << std::endl;
          } catch(...) {}

          try {
            auto home_health = blackboard->get<int>("home_health");
            std::cout << "home_health: " << home_health << std::endl;
          } catch(...) {}

          try {
            auto own_outpost_health = blackboard->get<int>("own_outpost_health");
            std::cout << "own_outpost_health: " << own_outpost_health << std::endl;
          } catch(...) {}

          try {
            auto hero_hp = blackboard->get<int>("hero_hp");
            std::cout << "hero_hp: " << hero_hp << std::endl;
          } catch(...) {}

          try {
            auto hero_position = blackboard->get<geometry_msgs::msg::Point>("hero_position");
            std::cout << "hero_position: (" << hero_position.x << ", " 
                      << hero_position.y << ", " << hero_position.z << ")" << std::endl;
          } catch(...) {}

          try {
            auto engineer_hp = blackboard->get<int>("engineer_hp");
            std::cout << "engineer_hp: " << engineer_hp << std::endl;
          } catch(...) {}

          try {
            auto engineer_position = blackboard->get<geometry_msgs::msg::Point>("engineer_position");
            std::cout << "engineer_position: (" << engineer_position.x << ", " 
                      << engineer_position.y << ", " << engineer_position.z << ")" << std::endl;
          } catch(...) {}

          try {
            auto infantry3_hp = blackboard->get<int>("infantry3_hp");
            std::cout << "infantry3_hp: " << infantry3_hp << std::endl;
          } catch(...) {}

          try {
            auto infantry3_position = blackboard->get<geometry_msgs::msg::Point>("infantry3_position");
            std::cout << "infantry3_position: (" << infantry3_position.x << ", " 
                      << infantry3_position.y << ", " << infantry3_position.z << ")" << std::endl;
          } catch(...) {}

          try {
            auto infantry4_hp = blackboard->get<int>("infantry4_hp");
            std::cout << "infantry4_hp: " << infantry4_hp << std::endl;
          } catch(...) {}

          try {
            auto infantry4_position = blackboard->get<geometry_msgs::msg::Point>("infantry4_position");
            std::cout << "infantry4_position: (" << infantry4_position.x << ", " 
                      << infantry4_position.y << ", " << infantry4_position.z << ")" << std::endl;
          } catch(...) {}

          // 2. 从gameInfoCallback设置的变量
          try {
            auto game_time_remaining = blackboard->get<int>("game_time_remaining");
            std::cout << "game_time_remaining: " << game_time_remaining << std::endl;
          } catch(...) {}

          try {
            auto coin_remaining = blackboard->get<int>("coin_remaining");
            std::cout << "coin_remaining: " << coin_remaining << std::endl;
          } catch(...) {}

          try {
            auto game_status = blackboard->get<int>("game_status");
            std::cout << "game_status: " << game_status << std::endl;
          } catch(...) {}

          try {
            auto small_energy_status = blackboard->get<int>("small_energy_status");
            std::cout << "small_energy_status: " << small_energy_status << std::endl;
          } catch(...) {}

          try {
            auto big_energy_status = blackboard->get<int>("big_energy_status");
            std::cout << "big_energy_status: " << big_energy_status << std::endl;
          } catch(...) {}

          try {
            auto fort_occupation_status = blackboard->get<int>("fort_occupation_status");
            std::cout << "fort_occupation_status: " << fort_occupation_status << std::endl;
          } catch(...) {}

          try {
            auto event_code_raw = blackboard->get<uint32_t>("event_code_raw");
            std::cout << "event_code_raw: " << event_code_raw << std::endl;
          } catch(...) {}

          // 3. 从radarInfoCallback设置的变量
          try {
            auto enemy_coin_left = blackboard->get<int>("enemy_coin_left");
            std::cout << "enemy_coin_left: " << enemy_coin_left << std::endl;
          } catch(...) {}

          try {
            auto enemy_coin_accumulated = blackboard->get<int>("enemy_coin_accumulated");
            std::cout << "enemy_coin_accumulated: " << enemy_coin_accumulated << std::endl;
          } catch(...) {}

          try {
            auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
            std::cout << "enemy_outpost_destroyed: " << std::boolalpha << enemy_outpost_destroyed << std::endl;
          } catch(...) {}

          try {
            auto radar_info = blackboard->get<ros_interfaces::msg::RadarInfo>("radar_info");
            std::cout << "radar_info: [RadarInfo message]" << std::endl;
          } catch(...) {}

          // 4. 从sentryOfflineCallback设置的变量
          try {
            auto target_valid = blackboard->get<bool>("target_valid");
            std::cout << "target_valid: " << std::boolalpha << target_valid << std::endl;
          } catch(...) {}

          try {
            auto gimbal_yaw = blackboard->get<float>("gimbal_yaw");
            std::cout << "gimbal_yaw: " << gimbal_yaw << std::endl;
          } catch(...) {}

          try {
            auto lifter_current_pos = blackboard->get<int>("lifter_current_pos");
            std::cout << "lifter_current_pos: " << lifter_current_pos << std::endl;
          } catch(...) {}

          try {
            auto is_transformable = blackboard->get<bool>("is_transformable");
            std::cout << "is_transformable: " << std::boolalpha << is_transformable << std::endl;
          } catch(...) {}

          try {
            auto transform_state = blackboard->get<float>("transform_state");
            std::cout << "transform_state: " << transform_state << std::endl;
          } catch(...) {}

          try {
            auto target_armor_id = blackboard->get<int>("target_armor_id");
            std::cout << "target_armor_id: " << target_armor_id << std::endl;
          } catch(...) {}

          try {
            auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
            std::cout << "target_pose: (" << target_pose.position.x << ", " 
                      << target_pose.position.y << ", " << target_pose.position.z << ")" << std::endl;
          } catch(...) {}

          // 5. 从sentryOnlineCallback设置的变量
          try {
            auto health = blackboard->get<float>("health");
            std::cout << "health: " << health << std::endl;
          } catch(...) {}

          try {
            auto bullets_remaining = blackboard->get<int>("bullets_remaining");
            std::cout << "bullets_remaining: " << bullets_remaining << std::endl;
          } catch(...) {}

          try {
            auto cooling_value = blackboard->get<int>("cooling_value");
            std::cout << "cooling_value: " << cooling_value << std::endl;
          } catch(...) {}

          try {
            auto heat_limit = blackboard->get<int>("heat_limit");
            std::cout << "heat_limit: " << heat_limit << std::endl;
          } catch(...) {}

          try {
            auto current_heat = blackboard->get<int>("current_heat");
            std::cout << "current_heat: " << current_heat << std::endl;
          } catch(...) {}

          try {
            auto speed_monitor_angle = blackboard->get<float>("speed_monitor_angle");
            std::cout << "speed_monitor_angle: " << speed_monitor_angle << std::endl;
          } catch(...) {}

          try {
            auto sentry_position = blackboard->get<geometry_msgs::msg::Point>("sentry_position");
            std::cout << "sentry_position: (" << sentry_position.x << ", " 
                      << sentry_position.y << ", " << sentry_position.z << ")" << std::endl;
          } catch(...) {}

          try {
            auto sentry_info_1_raw = blackboard->get<uint32_t>("sentry_info_1_raw");
            std::cout << "sentry_info_1_raw: " << sentry_info_1_raw << std::endl;
          } catch(...) {}

          try {
            auto sentry_info_2_raw = blackboard->get<uint16_t>("sentry_info_2_raw");
            std::cout << "sentry_info_2_raw: " << sentry_info_2_raw << std::endl;
          } catch(...) {}

          try {
            auto is_disengaged = blackboard->get<bool>("is_disengaged");
            std::cout << "is_disengaged: " << std::boolalpha << is_disengaged << std::endl;
          } catch(...) {}

          try {
            auto current_stance = blackboard->get<int>("current_stance");
            std::cout << "current_stance: " << current_stance << std::endl;
          } catch(...) {}

          try {
            auto can_activate_energy = blackboard->get<bool>("can_activate_energy");
            std::cout << "can_activate_energy: " << std::boolalpha << can_activate_energy << std::endl;
          } catch(...) {}

          try {
            auto can_free_resurrect = blackboard->get<bool>("can_free_resurrect");
            std::cout << "can_free_resurrect: " << std::boolalpha << can_free_resurrect << std::endl;
          } catch(...) {}

          try {
            auto can_instant_resurrect = blackboard->get<bool>("can_instant_resurrect");
            std::cout << "can_instant_resurrect: " << std::boolalpha << can_instant_resurrect << std::endl;
          } catch(...) {}

          try {
            auto instant_resurrect_cost = blackboard->get<int>("instant_resurrect_cost");
            std::cout << "instant_resurrect_cost: " << instant_resurrect_cost << std::endl;
          } catch(...) {}

          // 6. 从定时器回调设置的变量
          try {
            auto outpost_msg = blackboard->get<bool>("outpost_msg");
            std::cout << "outpost_msg: " << std::boolalpha << outpost_msg << std::endl;
          } catch(...) {}

          // 7. 从mpc_cmd_sub回调设置的变量
          try {
            auto through_tunnel = blackboard->get<bool>("through_tunnel");
            std::cout << "through_tunnel: " << std::boolalpha << through_tunnel << std::endl;
          } catch(...) {}

          // 8. 从cmd_vel_sub回调设置的变量
          try {
            auto cmd_vel = blackboard->get<geometry_msgs::msg::Twist>("cmd_vel");
            std::cout << "cmd_vel: linear(" 
                      << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.linear.z 
                      << ") angular("
                      << cmd_vel.angular.x << ", " << cmd_vel.angular.y << ", " << cmd_vel.angular.z 
                      << ")" << std::endl;
          } catch(...) {}

          // 9. 可能还有其他变量
          try {
            auto current_mode = blackboard->get<int>("current_mode");
            std::cout << "current_mode: " << current_mode << std::endl;
          } catch(...) {}

          try {
            auto desired_stance = blackboard->get<int>("desired_stance");
            std::cout << "desired_stance: " << desired_stance << std::endl;
          } catch(...) {}

          try {
            auto desired_lifter_pos = blackboard->get<int>("desired_lifter_pos");
            std::cout << "desired_lifter_pos: " << desired_lifter_pos << std::endl;
          } catch(...) {}

        }
        catch (const std::exception& e)
        {
          std::cout << "Error reading from blackboard: " << e.what() << std::endl;
        }

        std::cout << "==================================================================" << std::endl;
        std::cout << std::endl;

        // 等待5秒
        rate.sleep();
      }

      return BT::NodeStatus::RUNNING;
    };

    // 运行打印循环
    return printBlackboardVariables();
  }

  void halt() override
  {
    // 停止节点
  }
};