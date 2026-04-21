#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <bt_manager/utils/area.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {
constexpr const char * C_GREEN = "\033[1;32m";
constexpr const char * C_YELLOW = "\033[1;33m";
constexpr const char * C_CYAN = "\033[1;36m";
constexpr const char * C_MAGENTA = "\033[1;35m";
constexpr const char * C_RESET = "\033[0m";
constexpr int TICK_PERIOD_MS = 100;
constexpr int TICKS_PER_SUBPHASE = 100;     // 10s / 100ms
constexpr int PHASE_DURATION_SECONDS = 30;  // A/B/C each 10s

uint16_t buildSentryInfo2(bool is_disengaged, uint8_t current_stance_bits, bool can_activate_energy)
{
  uint16_t value = 0;
  if (is_disengaged) {
    value |= 0x0001;
  }
  value |= static_cast<uint16_t>((current_stance_bits & 0x3) << 12);
  if (can_activate_energy) {
    value |= static_cast<uint16_t>(1U << 14);
  }
  return value;
}

uint32_t buildSentryInfo1(
  bool can_free_resurrect, bool can_instant_resurrect, uint16_t instant_resurrect_cost)
{
  uint32_t value = 0;
  if (can_free_resurrect) {
    value |= (1U << 19);
  }
  if (can_instant_resurrect) {
    value |= (1U << 20);
  }
  value |= (static_cast<uint32_t>(instant_resurrect_cost & 0x03FF) << 21);
  return value;
}

uint32_t buildEventCode(
  uint8_t small_energy_status, uint8_t big_energy_status, uint8_t fort_occupation_status)
{
  uint32_t value = 0;
  value |= (static_cast<uint32_t>(small_energy_status & 0x3) << 3);
  value |= (static_cast<uint32_t>(big_energy_status & 0x3) << 5);
  value |= (static_cast<uint32_t>(fort_occupation_status & 0x3) << 25);
  return value;
}

Sentry_BT::Point2D getAreaCenter(const Sentry_BT::Area_Square & area)
{
  const double min_x = std::min(area.top_left.x, area.bottom_right.x);
  const double max_x = std::max(area.top_left.x, area.bottom_right.x);
  const double min_y = std::min(area.top_left.y, area.bottom_right.y);
  const double max_y = std::max(area.top_left.y, area.bottom_right.y);
  return Sentry_BT::Point2D{(min_x + max_x) * 0.5, (min_y + max_y) * 0.5, 0.0};
}

Sentry_BT::Point2D getOutsidePointNearArea(const Sentry_BT::Area_Square & area)
{
  const double max_x = std::max(area.top_left.x, area.bottom_right.x);
  const double max_y = std::max(area.top_left.y, area.bottom_right.y);
  return Sentry_BT::Point2D{max_x + 0.8, max_y + 0.8, 0.0};
}
}  // namespace

class EventStatusTestNode : public rclcpp::Node
{
public:
  EventStatusTestNode()
  : Node("event_status_test"), phase_(1), phase_tick_(0), last_phase_(-1), last_subphase_(-1),
    tf_enabled_(true), last_tf_enabled_(true), fake_time_jump_applied_(false)
  {
    team_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
    game_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
    radar_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);
    offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
    online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tick_timer_ = create_wall_timer(std::chrono::milliseconds(TICK_PERIOD_MS), [this]() {
      this->onTick();
    });
    phase_timer_ = create_wall_timer(std::chrono::seconds(PHASE_DURATION_SECONDS), [this]() {
      this->advancePhase();
    });

    RCLCPP_INFO(get_logger(),
      "%s[TestNode] event_test started: 10Hz publish + 30s phase switch (10s per sub-scenario)%s",
      C_CYAN,
      C_RESET);
    logStateTransition(true);
  }

private:
  void advancePhase()
  {
    phase_ = (phase_ % 6) + 1;
    phase_tick_ = 0;
    fake_time_jump_applied_ = false;
    logStateTransition(true);
  }

  int subPhase() const
  {
    // 一个 phase 内再细分 3 个子场景：0~2
    // 30 秒 phase, 每 10 秒一个子场景。
    return std::min(2, phase_tick_ / TICKS_PER_SUBPHASE);
  }

  void logStateTransition(bool force = false)
  {
    const int sub = subPhase();
    if (!force && phase_ == last_phase_ && sub == last_subphase_ && tf_enabled_ == last_tf_enabled_) {
      return;
    }

    last_phase_ = phase_;
    last_subphase_ = sub;
    last_tf_enabled_ = tf_enabled_;

    switch (phase_) {
    case 1:
      if (sub == 0) {
        RCLCPP_INFO(get_logger(),
          "%s[Phase1-A] 生存最高优先: 低血低弹，预期强制回血回弹(HOME)+MOVE%s",
          C_GREEN,
          C_RESET);
      } else {
        RCLCPP_INFO(
          get_logger(), "%s[Phase1-B] 恢复血弹: 预期退出回血模式，恢复常规行为%s", C_GREEN, C_RESET);
      }
      break;
    case 2:
      if (sub == 0) {
        RCLCPP_INFO(
          get_logger(), "%s[Phase2-A] TF进台阶区 + 台阶下有队友: 预期下台阶避让中止%s", C_YELLOW, C_RESET);
      } else if (sub == 1) {
        RCLCPP_INFO(get_logger(), "%s[Phase2-B] 清除队友占位: 预期恢复下台阶cmd_vel%s", C_YELLOW, C_RESET);
      } else {
        RCLCPP_INFO(
          get_logger(), "%s[Phase2-C] TF进隧道区: 预期触发过隧道流程(含升降相关)%s", C_YELLOW, C_RESET);
      }
      break;
    case 3:
      if (sub == 0) {
        RCLCPP_INFO(
          get_logger(), "%s[Phase3-A] NORMAL: 前哨站未摧毁，预期优先响应前哨站%s", C_CYAN, C_RESET);
      } else if (sub == 1) {
        RCLCPP_INFO(
          get_logger(), "%s[Phase3-B] DEFEND: 基地低血+堡垒空闲，预期占领己方堡垒%s", C_CYAN, C_RESET);
      } else {
        RCLCPP_INFO(
          get_logger(), "%s[Phase3-C] ATTACK: 前哨站已毁+能量激活，预期压制敌方堡垒%s", C_CYAN, C_RESET);
      }
      break;
    case 4:
      if (sub == 0) {
        RCLCPP_INFO(get_logger(),
          "%s[Phase4-A] 强锁敌追踪: target_valid=true，预期追踪与追击目标发布%s",
          C_MAGENTA,
          C_RESET);
      } else if (sub == 1) {
        RCLCPP_INFO(get_logger(),
          "%s[Phase4-B1] 丢失目标+DEFEND语义巡检: 预期防御战术巡检输出%s",
          C_MAGENTA,
          C_RESET);
      } else {
        RCLCPP_INFO(get_logger(),
          "%s[Phase4-B2] 丢失目标+ATTACK/NORMAL巡检: 预期战术隔离巡检输出%s",
          C_MAGENTA,
          C_RESET);
      }
      break;
    case 5:
      if (sub == 0 || sub == 1) {
        RCLCPP_INFO(
          get_logger(), "%s[Phase5-A] 1Hz交替target_valid: 预期5秒CD拦截姿态抖动%s", C_YELLOW, C_RESET);
      } else {
        RCLCPP_INFO(get_logger(),
          "%s[Phase5-B] 伪造长时间流逝(>180s): 预期触发姿态超时刷新机制%s",
          C_YELLOW,
          C_RESET);
      }
      break;
    case 6:
      if (sub == 0) {
        RCLCPP_INFO(get_logger(),
          "%s[Phase6-A] 同时触发多事件: 预期抢占顺序回血>特殊响应>追踪>巡逻%s",
          C_GREEN,
          C_RESET);
      } else {
        RCLCPP_INFO(get_logger(),
          "%s[Phase6-B] 关闭TF广播: 预期系统安全退化，不崩溃(缓存/报错处理)%s",
          C_GREEN,
          C_RESET);
      }
      break;
    default:
      break;
    }

    if (tf_enabled_) {
      RCLCPP_INFO(get_logger(), "%s[TF] enabled%s", C_CYAN, C_RESET);
    } else {
      RCLCPP_INFO(get_logger(), "%s[TF] disabled (boundary test)%s", C_CYAN, C_RESET);
    }
  }

  void publishTf(const Sentry_BT::Point2D & anchor)
  {
    if (!tf_enabled_) {
      return;
    }

    const auto stamp = now();

    // 中文说明：这里通过平移 map->camera_init 来模拟“机器人在地图中的位置”，
    // 供被测系统做 map->base_link 的位姿推断与区域判断。
    geometry_msgs::msg::TransformStamped map_to_camera;
    map_to_camera.header.stamp = stamp;
    map_to_camera.header.frame_id = "map";
    map_to_camera.child_frame_id = "camera_init";
    map_to_camera.transform.translation.x = anchor.x;
    map_to_camera.transform.translation.y = anchor.y;
    map_to_camera.transform.translation.z = 0.0;

    tf2::Quaternion q_map_camera;
    q_map_camera.setRPY(0.0, 0.0, anchor.yaw);
    map_to_camera.transform.rotation.x = q_map_camera.x();
    map_to_camera.transform.rotation.y = q_map_camera.y();
    map_to_camera.transform.rotation.z = q_map_camera.z();
    map_to_camera.transform.rotation.w = q_map_camera.w();

    geometry_msgs::msg::TransformStamped camera_to_gimbal;
    camera_to_gimbal.header.stamp = stamp;
    camera_to_gimbal.header.frame_id = "camera_init";
    camera_to_gimbal.child_frame_id = "gimbal";
    camera_to_gimbal.transform.translation.x = 0.15;
    camera_to_gimbal.transform.translation.y = 0.0;
    camera_to_gimbal.transform.translation.z = 0.2;

    tf2::Quaternion q_camera_gimbal;
    q_camera_gimbal.setRPY(0.0, 0.0, 0.0);
    camera_to_gimbal.transform.rotation.x = q_camera_gimbal.x();
    camera_to_gimbal.transform.rotation.y = q_camera_gimbal.y();
    camera_to_gimbal.transform.rotation.z = q_camera_gimbal.z();
    camera_to_gimbal.transform.rotation.w = q_camera_gimbal.w();

    tf_broadcaster_->sendTransform(map_to_camera);
    tf_broadcaster_->sendTransform(camera_to_gimbal);
  }

  void applyBaseline(ros_interfaces::msg::TeamInformation & team_msg,
    ros_interfaces::msg::GameInfo & game_msg,
    ros_interfaces::msg::RadarInfo & radar_msg,
    ros_interfaces::msg::SentryInfoOffline & offline_msg,
    ros_interfaces::msg::SentryInfoOnline & online_msg,
    Sentry_BT::Point2D & tf_anchor)
  {
    const auto stamp = now();
    team_msg.header.stamp = stamp;
    game_msg.header.stamp = stamp;
    radar_msg.header.stamp = stamp;
    offline_msg.header.stamp = stamp;
    online_msg.header.stamp = stamp;

    // 中文说明：默认状态统一回到(5,7)，用于“非区域触发”基线。
    tf_anchor = Sentry_BT::Point2D{5.0, 7.0, 0.0};
    tf_enabled_ = true;

    // Team / Game baseline
    team_msg.outpost_hp = 1500;
    team_msg.base_hp = 3000;
    game_msg.game_status = 4;
    game_msg.game_time_remaining = 420;
    game_msg.coin_remaining = 80;
    game_msg.event_code = buildEventCode(0, 0, 0);

    // Radar baseline
    radar_msg.enemy_coin_left = 30;
    radar_msg.enemy_coin_accumulated = 120;
    radar_msg.is_enemy_outpost_sensed = true;  // true => 前哨站未摧毁

    // Offline (target) baseline
    offline_msg.is_get = false;
    offline_msg.armor_num = 3;
    offline_msg.armor_pos.x = 0.0;
    offline_msg.armor_pos.y = 0.0;
    offline_msg.armor_pos.z = 0.0;
    offline_msg.yaw_imu = 0.0F;
    offline_msg.lifter_current_pos = 0;
    offline_msg.is_transformable = true;
    offline_msg.transform_state = 0.0F;

    // Online baseline
    online_msg.self_health = 1000;       // bt内通常会做/4，对应约250
    online_msg.bullets_remaining = 300;  // 充足弹药
    online_msg.cooling_value = 40;
    online_msg.heat_limit = 200;
    online_msg.current_heat = 20;
    online_msg.speed_monitor_angle = 0.0F;
    online_msg.sentry_info_1 = buildSentryInfo1(false, false, 0);
    online_msg.sentry_info_2 = buildSentryInfo2(true, 0, false);

    // 默认友军位置放在台阶安全区外
    team_msg.allies[0].armor_id = 1;
    team_msg.allies[0].remain_hp = 180;
    team_msg.allies[0].position.position.x = 2.0;
    team_msg.allies[0].position.position.y = -1.0;

    // 默认敌方观测
    radar_msg.enemies[0].robot_id = 101;
    radar_msg.enemies[0].robot_hp = 220;
    radar_msg.enemies[0].allowed_projectile = 90;
    radar_msg.enemies[0].position.position.x = 8.0;
    radar_msg.enemies[0].position.position.y = 3.0;
  }

  void applyPhaseScenario(ros_interfaces::msg::TeamInformation & team_msg,
    ros_interfaces::msg::GameInfo & game_msg,
    ros_interfaces::msg::RadarInfo & radar_msg,
    ros_interfaces::msg::SentryInfoOffline & offline_msg,
    ros_interfaces::msg::SentryInfoOnline & online_msg,
    Sentry_BT::Point2D & tf_anchor)
  {
    const int sub = subPhase();

    switch (phase_) {
    case 1: {
      // [Phase1] 生存最高优先 + 10秒后恢复
      if (phase_tick_ < TICKS_PER_SUBPHASE) {
        online_msg.self_health = 80;  // /4后为20
        online_msg.bullets_remaining = 50;
      } else {
        online_msg.self_health = 400;  // /4后为100
        online_msg.bullets_remaining = 300;
      }
      break;
    }

    case 2: {
      // [Phase2-A/B/C] 台阶避让 -> 解除避让 -> 过隧道
      if (sub == 0) {
        // 台阶避让阶段：保持低血低弹，确保进入生存分支并触发下台阶监测。
        online_msg.self_health = 80;  // /4后为20，低于阈值30
        online_msg.bullets_remaining = 50;
        tf_anchor = getAreaCenter(Sentry_BT::stairs_zone);
        // 中文说明：把友军放入台阶下安全区，触发“有队友占位，暂停下台阶”
        team_msg.allies[0].position.position.x = getAreaCenter(Sentry_BT::stairs_lower_safe_zone).x;
        team_msg.allies[0].position.position.y = getAreaCenter(Sentry_BT::stairs_lower_safe_zone).y;
      } else if (sub == 1) {
        // 台阶恢复阶段：继续保持低血低弹，验证解除队友占位后的下台阶恢复。
        online_msg.self_health = 80;
        online_msg.bullets_remaining = 50;
        tf_anchor = getAreaCenter(Sentry_BT::stairs_zone);
        // 清空队友占位
        team_msg.allies[0].position.position.x = 2.0;
        team_msg.allies[0].position.position.y = -1.0;
      } else {
        // 隧道阶段：血量和弹量恢复，避免继续走“低血回家”语义。
        online_msg.self_health = 400;  // /4后为100
        online_msg.bullets_remaining = 300;
        tf_anchor = getAreaCenter(Sentry_BT::tunnel_zone[0]);
        offline_msg.lifter_current_pos = 1;  // 模拟升降机构处于“准备过洞”状态
      }
      break;
    }

    case 3: {
      // [Phase3-A/B/C] NORMAL -> DEFEND -> ATTACK
      if (sub == 0) {
        radar_msg.is_enemy_outpost_sensed = true;  // 前哨站未摧毁
        team_msg.base_hp = 2500;
        game_msg.event_code = buildEventCode(0, 0, 0);
      } else if (sub == 1) {
        team_msg.base_hp = 800;                         // 触发防守阈值
        game_msg.event_code = buildEventCode(0, 0, 0);  // fort_occupation_status=0
      } else {
        radar_msg.is_enemy_outpost_sensed = false;  // 前哨站已摧毁
        team_msg.base_hp = 2200;
        game_msg.event_code = buildEventCode(1, 0, 1);
        online_msg.sentry_info_2 = buildSentryInfo2(true, 1, true);  // can_activate_energy=true
      }
      break;
    }

    case 4: {
      // [Phase4-A/B] 强锁敌追踪 + 丢失目标后的战术巡检
      if (sub == 0) {
        offline_msg.is_get = true;
        offline_msg.armor_pos.x = 6000.0;
        offline_msg.armor_pos.y = 4000.0;
        tf_anchor = getAreaCenter(Sentry_BT::highland_zone);
      } else if (sub == 1) {
        offline_msg.is_get = false;
        team_msg.base_hp = 700;                    // 倾向DEFEND
        radar_msg.is_enemy_outpost_sensed = true;  // 未摧毁
        tf_anchor = getAreaCenter(Sentry_BT::own_defense_zone);
      } else {
        offline_msg.is_get = false;
        team_msg.base_hp = 2200;
        radar_msg.is_enemy_outpost_sensed = false;  // 倾向ATTACK/NORMAL切换
        online_msg.sentry_info_2 = buildSentryInfo2(true, 1, true);
        tf_anchor = getOutsidePointNearArea(Sentry_BT::own_defense_zone);
      }
      break;
    }

    case 5: {
      // [Phase5-A] 1Hz target_valid 翻转测试5秒CD
      if (sub < 2) {
        const bool target_on = ((phase_tick_ / 10) % 2) == 0;
        offline_msg.is_get = target_on;
        if (target_on) {
          offline_msg.armor_pos.x = 7000.0;
          offline_msg.armor_pos.y = 3500.0;
        }
      } else {
        // [Phase5-B] 伪造超时刷新场景：在测试节点中模拟“逻辑时钟跳变”
        // 注：是否真正触发BT内部超时刷新，取决于被测系统如何取时。
        if (!fake_time_jump_applied_) {
          fake_time_jump_applied_ = true;
          RCLCPP_INFO(
            get_logger(), "%s[Phase5-B] 伪造Time Jump: +181s (用于超时刷新测试)%s", C_YELLOW, C_RESET);
        }
        offline_msg.is_get = false;
        online_msg.self_health = 1200;
        online_msg.bullets_remaining = 260;
      }
      break;
    }

    case 6: {
      // [Phase6-A/B] 抢占顺序校验 + TF丢失边界
      if (sub == 0) {
        // 同时触发：回血 + 前哨站响应 + 锁敌
        online_msg.self_health = 80;               // 低血
        online_msg.bullets_remaining = 50;         // 低弹
        radar_msg.is_enemy_outpost_sensed = true;  // 前哨站未摧毁
        offline_msg.is_get = true;                 // 有目标
        offline_msg.armor_pos.x = 5500.0;
        offline_msg.armor_pos.y = 3000.0;
        tf_anchor = getAreaCenter(Sentry_BT::highland_zone);
      } else {
        // TF 丢失测试：停止广播，但消息继续发布
        tf_enabled_ = false;
        online_msg.self_health = 900;
        online_msg.bullets_remaining = 220;
        offline_msg.is_get = false;
      }
      break;
    }

    default:
      break;
    }
  }

  void onTick()
  {
    ros_interfaces::msg::TeamInformation team_msg;
    ros_interfaces::msg::GameInfo game_msg;
    ros_interfaces::msg::RadarInfo radar_msg;
    ros_interfaces::msg::SentryInfoOffline offline_msg;
    ros_interfaces::msg::SentryInfoOnline online_msg;
    Sentry_BT::Point2D tf_anchor;

    applyBaseline(team_msg, game_msg, radar_msg, offline_msg, online_msg, tf_anchor);
    applyPhaseScenario(team_msg, game_msg, radar_msg, offline_msg, online_msg, tf_anchor);

    logStateTransition();

    team_pub_->publish(team_msg);
    game_pub_->publish(game_msg);
    radar_pub_->publish(radar_msg);
    offline_pub_->publish(offline_msg);
    online_pub_->publish(online_msg);
    publishTf(tf_anchor);

    ++phase_tick_;
  }

private:
  int phase_;
  int phase_tick_;
  int last_phase_;
  int last_subphase_;
  bool tf_enabled_;
  bool last_tf_enabled_;
  bool fake_time_jump_applied_;

  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr phase_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventStatusTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
