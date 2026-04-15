#pragma once
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_interfaces/msg/behavior.hpp"
#include "ros_interfaces/msg/game_info.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"
#include "ros_interfaces/msg/radar_info.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "ros_interfaces/msg/sentry_info_online.hpp"
#include "ros_interfaces/msg/team_information.hpp"
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// 定义机器人ID常量
constexpr int HERO_ROBOT_ID = 1;        // 假设英雄的ID是1
constexpr int ENEMY_HERO_ROBOT_ID = 1;  // 假设敌方英雄的ID是1

class BlackboardTestNode : public BT::CoroActionNode
{
public:
  BlackboardTestNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::CoroActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  // 辅助函数：格式化浮点数输出
  std::string formatFloat(float value, int precision = 2)
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
  }

  // 辅助函数：格式化点坐标
  std::string formatPoint(const geometry_msgs::msg::Point & point, int precision = 2)
  {
    std::ostringstream ss;
    ss << "(" << formatFloat(point.x, precision) << ", " << formatFloat(point.y, precision) << ", "
       << formatFloat(point.z, precision) << ")";
    return ss.str();
  }

  // 辅助函数：格式化姿态
  std::string formatPose(const geometry_msgs::msg::Pose & pose, int precision = 2)
  {
    std::ostringstream ss;
    ss << "位置: " << formatPoint(pose.position, precision) << ", 朝向: ("
       << formatFloat(pose.orientation.x, precision) << ", " << formatFloat(pose.orientation.y, precision)
       << ", " << formatFloat(pose.orientation.z, precision) << ", "
       << formatFloat(pose.orientation.w, precision) << ")";
    return ss.str();
  }

  BT::NodeStatus tick() override
  {
    // 获取黑板指针
    auto blackboard = config().blackboard;

    if (!blackboard) {
      std::cout << "Blackboard is null!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // 定时打印黑板变量的函数
    auto printBlackboardVariables = [this, blackboard]() -> BT::NodeStatus {
      std::cout << "\033[2J\033[1;1H";  // 清屏
      std::cout << "====================== Blackboard Variables ======================" << std::endl;

      auto now = std::chrono::system_clock::now();
      auto time = std::chrono::system_clock::to_time_t(now);
      std::cout << "时间戳: " << std::ctime(&time);
      std::cout << "==================================================================" << std::endl;

      try {
        // 1. 游戏基本信息
        std::cout << "\033[1;36m[游戏状态]\033[0m" << std::endl;
        try {
          auto game_time_remaining = blackboard->get<int>("game_time_remaining");
          std::cout << "  ▸ 游戏剩余时间: " << game_time_remaining << "秒" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 游戏剩余时间: 不可用" << std::endl;
        }

        try {
          auto game_status = blackboard->get<int>("game_status");
          std::string status_str = "未知";
          switch (game_status) {
          case 0:
            status_str = "未开始";
            break;
          case 1:
            status_str = "进行中";
            break;
          case 2:
            status_str = "暂停";
            break;
          case 3:
            status_str = "结束";
            break;
          }
          std::cout << "  ▸ 游戏状态: " << status_str << " (" << game_status << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 游戏状态: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 2. 团队信息
        std::cout << "\033[1;36m[团队信息]\033[0m" << std::endl;
        try {
          auto allies_info = blackboard->get<std::vector<Sentry_BT::AllyRobotInfo>>("allies_info");
          std::cout << "  ▸ 队友数量: " << allies_info.size() << std::endl;

          bool hero_found = false;
          for (const auto & ally : allies_info) {
            if (ally.robot_id == 1) {
              std::cout << "  ▸ \033[1;33m【英雄】\033[0m ID: " << ally.robot_id
                        << ", 血量: " << ally.remain_hp << ", 位置: " << formatPoint(ally.position)
                        << std::endl;
              hero_found = true;
              break;
            }
          }

          if (!hero_found) {
            std::cout << "  ▸ 英雄: 未找到" << std::endl;
          }
        } catch (const std::exception & e) {
          std::cout << "  ▸ 队友信息: 读取错误 - " << e.what() << std::endl;
        }

        try {
          auto home_health = blackboard->get<int>("home_health");
          std::cout << "  ▸ 基地血量: " << home_health << std::endl;
        } catch (...) {
          std::cout << "  ▸ 基地血量: 不可用" << std::endl;
        }

        try {
          auto own_outpost_health = blackboard->get<int>("own_outpost_health");
          std::cout << "  ▸ 前哨站血量: " << own_outpost_health << std::endl;
        } catch (...) {
          std::cout << "  ▸ 前哨站血量: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 3. 敌人信息
        std::cout << "\033[1;36m[敌人信息]\033[0m" << std::endl;
        try {
          auto enemies_info = blackboard->get<std::vector<Sentry_BT::EnemyRobotInfo>>("enemies_info");
          std::cout << "  ▸ 敌人数量: " << enemies_info.size() << std::endl;

          bool enemy_hero_found = false;
          for (const auto & enemy : enemies_info) {
            if (enemy.robot_id == ENEMY_HERO_ROBOT_ID) {
              std::cout << "  ▸ \033[1;31m【敌方英雄】\033[0m ID: " << enemy.robot_id
                        << ", 血量: " << enemy.remain_hp << ", 弹药: " << enemy.allowed_projectile
                        << ", 位置: " << formatPoint(enemy.position) << std::endl;
              enemy_hero_found = true;
            }
          }

          if (!enemy_hero_found) {
            std::cout << "  ▸ 敌方英雄: 未发现" << std::endl;
          }
        } catch (const std::exception & e) {
          std::cout << "  ▸ 敌人信息: 读取错误 - " << e.what() << std::endl;
        }

        try {
          auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
          std::cout << "  ▸ 敌方前哨站状态: " << (enemy_outpost_destroyed ? "已摧毁" : "正常") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 敌方前哨站状态: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 4. 经济信息
        std::cout << "\033[1;36m[经济信息]\033[0m" << std::endl;
        try {
          auto coin_remaining = blackboard->get<int>("coin_remaining");
          std::cout << "  ▸ 剩余金币: " << coin_remaining << std::endl;
        } catch (...) {
          std::cout << "  ▸ 剩余金币: 不可用" << std::endl;
        }

        try {
          auto enemy_coin_left = blackboard->get<int>("enemy_coin_left");
          std::cout << "  ▸ 敌方剩余金币: " << enemy_coin_left << std::endl;
        } catch (...) {
          std::cout << "  ▸ 敌方剩余金币: 不可用" << std::endl;
        }

        try {
          auto enemy_coin_accumulated = blackboard->get<int>("enemy_coin_accumulated");
          std::cout << "  ▸ 敌方累积金币: " << enemy_coin_accumulated << std::endl;
        } catch (...) {
          std::cout << "  ▸ 敌方累积金币: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 5. 哨兵状态信息
        std::cout << "\033[1;36m[哨兵状态]\033[0m" << std::endl;
        try {
          auto health = blackboard->get<float>("health");
          std::cout << "  ▸ 当前血量: " << health << std::endl;
        } catch (...) {
          std::cout << "  ▸ 当前血量: 不可用" << std::endl;
        }

        try {
          auto bullets_remaining = blackboard->get<int>("bullets_remaining");
          std::cout << "  ▸ 剩余弹量: " << bullets_remaining << std::endl;
        } catch (...) {
          std::cout << "  ▸ 剩余弹量: 不可用" << std::endl;
        }

        try {
          auto current_heat = blackboard->get<int>("current_heat");
          auto heat_limit = blackboard->get<int>("heat_limit");
          auto cooling_value = blackboard->get<int>("cooling_value");
          std::cout << "  ▸ 热量: " << current_heat << "/" << heat_limit << ", 冷却值: " << cooling_value
                    << " (" << std::fixed << std::setprecision(1)
                    << (heat_limit > 0 ? (static_cast<float>(current_heat) / heat_limit * 100) : 0) << "%)"
                    << std::endl;
        } catch (...) {
          std::cout << "  ▸ 热量信息: 不可用" << std::endl;
        }

        try {
          auto speed_monitor_angle = blackboard->get<float>("speed_monitor_angle");
          std::cout << "  ▸ 速度监测角度: " << formatFloat(speed_monitor_angle) << std::endl;
        } catch (...) {
          std::cout << "  ▸ 速度监测角度: 不可用" << std::endl;
        }

        try {
          auto lifter_pos_now = blackboard->get<int>("lifter_pos_now");
          std::cout << "  ▸ 升降机构位置: " << lifter_pos_now << std::endl;
        } catch (...) {
          std::cout << "  ▸ 升降机构位置: 不可用" << std::endl;
        }

        try {
          auto is_transformable = blackboard->get<bool>("is_transformable");
          auto transform_state = blackboard->get<float>("transform_state");
          std::cout << "  ▸ 形态变换: " << (is_transformable ? "可变换" : "不可变换")
                    << ", 状态: " << formatFloat(transform_state) << std::endl;
        } catch (...) {
          std::cout << "  ▸ 形态变换: 不可用" << std::endl;
        }

        try {
          auto is_disengaged = blackboard->get<bool>("is_disengaged");
          std::cout << "  ▸ 脱战状态: " << (is_disengaged ? "已脱战" : "战斗中") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 脱战状态: 不可用" << std::endl;
        }

        try {
          auto current_stance = blackboard->get<int>("current_stance");
          std::string stance_str = "未知";
          switch (current_stance) {
          case 0:
            stance_str = "地面";
            break;
          case 1:
            stance_str = "悬空";
            break;
          case 2:
            stance_str = "爬坡";
            break;
          case 3:
            stance_str = "高打";
            break;
          }
          std::cout << "  ▸ 当前姿态: " << stance_str << " (" << current_stance << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 当前姿态: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 6. 能量机关信息
        std::cout << "\033[1;36m[能量机关]\033[0m" << std::endl;
        try {
          auto small_energy_status = blackboard->get<int>("small_energy_status");
          std::string small_str = "未知";
          switch (small_energy_status) {
          case 0:
            small_str = "未激活";
            break;
          case 1:
            small_str = "正在激活";
            break;
          case 2:
            small_str = "已激活";
            break;
          }
          std::cout << "  ▸ 小能量机关: " << small_str << " (" << small_energy_status << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 小能量机关: 不可用" << std::endl;
        }

        try {
          auto big_energy_status = blackboard->get<int>("big_energy_status");
          std::string big_str = "未知";
          switch (big_energy_status) {
          case 0:
            big_str = "未激活";
            break;
          case 1:
            big_str = "正在激活";
            break;
          case 2:
            big_str = "已激活";
            break;
          }
          std::cout << "  ▸ 大能量机关: " << big_str << " (" << big_energy_status << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 大能量机关: 不可用" << std::endl;
        }

        try {
          auto fort_occupation_status = blackboard->get<int>("fort_occupation_status");
          std::string fort_str = "未知";
          switch (fort_occupation_status) {
          case 0:
            fort_str = "未被占领";
            break;
          case 1:
            fort_str = "正在占领";
            break;
          case 2:
            fort_str = "已占领";
            break;
          }
          std::cout << "  ▸ 堡垒增益点: " << fort_str << " (" << fort_occupation_status << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 堡垒增益点: 不可用" << std::endl;
        }

        try {
          auto can_activate_energy = blackboard->get<bool>("can_activate_energy");
          std::cout << "  ▸ 能否激活能量机关: " << (can_activate_energy ? "是" : "否") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 能否激活能量机关: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 7. 目标信息
        std::cout << "\033[1;36m[目标信息]\033[0m" << std::endl;
        try {
          auto target_valid = blackboard->get<bool>("target_valid");
          std::cout << "  ▸ 目标有效性: " << (target_valid ? "有效" : "无效") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 目标有效性: 不可用" << std::endl;
        }

        if (blackboard->get<bool>("target_valid")) {
          try {
            auto target_armor_id = blackboard->get<int>("target_armor_id");
            std::cout << "  ▸ 目标装甲板ID: " << target_armor_id << std::endl;
          } catch (...) {
            std::cout << "  ▸ 目标装甲板ID: 不可用" << std::endl;
          }

          try {
            auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
            std::cout << "  ▸ 目标位置: " << formatPoint(target_pose.position) << std::endl;
          } catch (...) {
            std::cout << "  ▸ 目标位置: 不可用" << std::endl;
          }
        } else {
          std::cout << "  ▸ 无有效目标" << std::endl;
        }
        std::cout << std::endl;

        // 8. 复活与能力信息
        std::cout << "\033[1;36m[复活与能力]\033[0m" << std::endl;
        try {
          auto can_free_resurrect = blackboard->get<bool>("can_free_resurrect");
          std::cout << "  ▸ 免费复活: " << (can_free_resurrect ? "可用" : "不可用") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 免费复活: 不可用" << std::endl;
        }

        try {
          auto can_instant_resurrect = blackboard->get<bool>("can_instant_resurrect");
          auto instant_resurrect_cost = blackboard->get<int>("instant_resurrect_cost");
          std::cout << "  ▸ 立即复活: " << (can_instant_resurrect ? "可用" : "不可用");
          if (can_instant_resurrect) {
            std::cout << " (花费: " << instant_resurrect_cost << "金币)";
          }
          std::cout << std::endl;
        } catch (...) {
          std::cout << "  ▸ 立即复活: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 9. 位置与导航信息
        std::cout << "\033[1;36m[位置与导航]\033[0m" << std::endl;
        try {
          auto gimbal_yaw = blackboard->get<float>("gimbal_yaw");
          std::cout << "  ▸ 云台偏航角: " << formatFloat(gimbal_yaw) << "°" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 云台偏航角: 不可用" << std::endl;
        }

        try {
          auto sentry_position = blackboard->get<geometry_msgs::msg::Point>("sentry_position");
          std::cout << "  ▸ 哨兵位置: " << formatPoint(sentry_position) << std::endl;
        } catch (...) {
          std::cout << "  ▸ 哨兵位置: 不可用" << std::endl;
        }

        try {
          auto cmd_vel = blackboard->get<geometry_msgs::msg::Twist>("cmd_vel");
          std::cout << "  ▸ 速度命令: 线速度(" << formatFloat(cmd_vel.linear.x) << ", "
                    << formatFloat(cmd_vel.linear.y) << ", " << formatFloat(cmd_vel.linear.z) << ") 角速度("
                    << formatFloat(cmd_vel.angular.x) << ", " << formatFloat(cmd_vel.angular.y) << ", "
                    << formatFloat(cmd_vel.angular.z) << ")" << std::endl;
        } catch (...) {
          std::cout << "  ▸ 速度命令: 不可用" << std::endl;
        }

        try {
          auto through_tunnel = blackboard->get<bool>("through_tunnel");
          std::cout << "  ▸ 隧道通行: " << (through_tunnel ? "是" : "否") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 隧道通行: 不可用" << std::endl;
        }

        try {
          auto outpost_msg = blackboard->get<bool>("outpost_msg");
          std::cout << "  ▸ 前哨站到达: " << (outpost_msg ? "是" : "否") << std::endl;
        } catch (...) {
          std::cout << "  ▸ 前哨站到达: 不可用" << std::endl;
        }
        std::cout << std::endl;

        // 10. 原始数据（调试用）
        std::cout << "\033[1;36m[原始数据]\033[0m" << std::endl;
        try {
          auto event_code_raw = blackboard->get<uint32_t>("event_code_raw");
          std::cout << "  ▸ 事件码: 0x" << std::hex << std::uppercase << event_code_raw << std::dec
                    << std::endl;
        } catch (...) {
          std::cout << "  ▸ 事件码: 不可用" << std::endl;
        }

        try {
          auto sentry_info_1_raw = blackboard->get<uint32_t>("sentry_info_1_raw");
          std::cout << "  ▸ 哨兵信息1: 0x" << std::hex << std::uppercase << sentry_info_1_raw << std::dec
                    << std::endl;
        } catch (...) {
          std::cout << "  ▸ 哨兵信息1: 不可用" << std::endl;
        }

        try {
          auto sentry_info_2_raw = blackboard->get<uint16_t>("sentry_info_2_raw");
          std::cout << "  ▸ 哨兵信息2: 0x" << std::hex << std::uppercase << sentry_info_2_raw << std::dec
                    << std::endl;
        } catch (...) {
          std::cout << "  ▸ 哨兵信息2: 不可用" << std::endl;
        }

      } catch (const std::exception & e) {
        std::cout << "\033[1;31m读取黑板错误: " << e.what() << "\033[0m" << std::endl;
      }

      std::cout << "==================================================================" << std::endl;
      std::cout << "按Ctrl+C退出" << std::endl;

      return BT::NodeStatus::RUNNING;
    };

    // 持续打印信息
    while (true) {
      auto status = printBlackboardVariables();
      if (status != BT::NodeStatus::RUNNING) {
        return status;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 每0.5秒刷新一次
    }
  }

  void halt() override
  {
    // 停止节点
  }
};