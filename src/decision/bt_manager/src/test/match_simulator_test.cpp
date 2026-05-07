#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/ally_robot_status.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/enemy_robot_status.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/mpc_position_command.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {
constexpr double kScenarioDurationSec = 15.0;
constexpr std::uint8_t kDefaultLifterPos = 0;
constexpr std::uint8_t kDefaultCapacitor = 100;
constexpr std::uint16_t kDefaultGameTime = 420;
constexpr std::uint8_t kDefaultGameStatus = 4;
constexpr std::uint16_t kHealthScale = 4;
}

// ============= Match Step =============
struct MatchStep
{
  std::string name;
  std::string description;
  std::string expected_behavior;

  // Team Info
  std::uint16_t base_hp = 3000;
  std::uint16_t outpost_hp = 1500;

  // Game Info
  std::uint16_t game_time = kDefaultGameTime;
  std::uint16_t coin_remaining = 500;
  std::uint32_t event_code = 0;
  std::uint8_t game_status = kDefaultGameStatus;

  // Radar Info
  bool enemy_outpost_exists = true;
  std::uint16_t enemy_coin_left = 200;
  bool enemy_robot_valid = false;
  std::uint8_t enemy_robot_id = 3;
  std::uint16_t enemy_robot_hp = 300;
  double enemy_pos_x = 0.0;
  double enemy_pos_y = 0.0;

  // Offline Info
  bool target_valid = false;
  double target_x = 0.0;
  double target_y = 0.0;
  std::uint8_t armor_num = 0;
  float yaw_encoder = 0.0f;
  float yaw_imu = 0.0f;
  std::uint8_t lifter_pos = kDefaultLifterPos;
  bool is_transformable = true;
  float transform_state = 0.0f;
  std::uint8_t capacitor = kDefaultCapacitor;

  // Online Info
  float health = 100.0f;
  int bullets = 300;
  int current_heat = 0;
  bool is_disengaged = true;

  // sentry_info bitfields
  std::uint8_t current_stance = 3;  // MOVE
  bool can_activate_energy = false;
  bool can_free_resurrect = false;
  bool can_instant_resurrect = false;
  std::uint16_t instant_resurrect_cost = 200;
  std::uint8_t remote_ammo_exchange_count = 0;
  std::uint16_t remaining_ammo_exchange = 500;

  // Position
  double pos_x = 15.0;
  double pos_y = 0.0;
  double pos_z = 0.0;
  double yaw_deg = 0.0;

  // MPC
  std::vector<std::pair<double, double>> mpc_points;
};

// ============= Simulator Node =============
class MatchSimulatorNode : public rclcpp::Node
{
public:
  MatchSimulatorNode()
  : Node("match_simulator")
  {
    team_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
    game_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
    radar_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);
    offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
    online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
    mpc_pub_ = create_publisher<ros_interfaces::msg::MpcPositionCommand>("/opt_path", 10);

    behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
      "/sentry/behaivor_send", 10,
      [this](const ros_interfaces::msg::Behavior::SharedPtr msg) {
        last_behavior_ = *msg;
      });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_ = *msg;
      });

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    buildSteps();

    publish_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      publishAll();
    });

    scenario_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kScenarioDurationSec)),
      [this]() { advanceStep(); });

    log_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      logStatus();
    });

    RCLCPP_INFO(get_logger(), "=== Match Simulator Started ===");
    RCLCPP_INFO(get_logger(), "%zu steps, %.0fs per step", steps_.size(), kScenarioDurationSec);
    logCurrentStep();
  }

private:
  static MatchStep makeStep(const std::string & n, const std::string & d, const std::string & e)
  {
    MatchStep s;
    s.name = n;
    s.description = d;
    s.expected_behavior = e;
    return s;
  }

  void buildSteps()
  {
    MatchStep s;

    // Step 1
    s = makeStep("1_GameStart", "比赛开局: 己方半场, 满血满弹, 脱战, 移动姿态",
                 "期望: 巡逻模式, MOVE姿态");
    s.pos_x = 15.0; s.pos_y = 0.0;
    s.health = 100.0f; s.bullets = 300; s.is_disengaged = true;
    s.coin_remaining = 500; s.base_hp = 3000; s.outpost_hp = 1500;
    steps_.push_back(s);

    // Step 2
    s = makeStep("2_PatrolToMid", "巡逻: 向中场移动",
                 "期望: 巡逻模式, MOVE姿态");
    s.pos_x = 18.0; s.pos_y = 2.0;
    s.health = 100.0f; s.bullets = 295; s.is_disengaged = true;
    s.coin_remaining = 500; s.base_hp = 3000; s.outpost_hp = 1500;
    steps_.push_back(s);

    // Step 3
    s = makeStep("3_EnemyContact", "遭遇敌人: 目标出现(步兵3号), 进入交战",
                 "期望: 追击模式, 云台锁定>4m则MOVE姿态");
    s.pos_x = 20.0; s.pos_y = 4.0;
    s.health = 87.5f; s.bullets = 280; s.is_disengaged = false;
    s.coin_remaining = 500; s.base_hp = 3000; s.outpost_hp = 1500;
    s.target_valid = true; s.target_x = 24.0; s.target_y = 8.0;
    s.enemy_robot_valid = true; s.enemy_robot_id = 3; s.enemy_robot_hp = 300;
    s.enemy_pos_x = 24.0; s.enemy_pos_y = 8.0;
    steps_.push_back(s);

    // Step 4
    s = makeStep("4_CombatRemoteExchange", "持续交战: 弹药降到120, 金币充足, 触发远程兑换",
                 "期望: resource_tree远程兑换触发, ammo_purchase_total增加");
    s.pos_x = 22.0; s.pos_y = 6.0;
    s.health = 70.0f; s.bullets = 120; s.is_disengaged = false;
    s.coin_remaining = 500; s.base_hp = 3000; s.outpost_hp = 1500;
    s.target_valid = true; s.target_x = 26.0; s.target_y = 8.0;
    s.enemy_robot_valid = true; s.enemy_robot_id = 3; s.enemy_robot_hp = 200;
    s.enemy_pos_x = 26.0; s.enemy_pos_y = 8.0;
    s.remaining_ammo_exchange = 400;
    steps_.push_back(s);

    // Step 5
    s = makeStep("5_TunnelCrossing", "穿越隧道: 进入隧道区域, 升降机构下降",
                 "期望: 姿态树触发变形, MOVE/ATTACK, 小陀螺开启");
    s.pos_x = 8.5; s.pos_y = 4.5;
    s.health = 62.5f; s.bullets = 100; s.is_disengaged = false;
    s.coin_remaining = 350; s.base_hp = 3000; s.outpost_hp = 1500;
    s.lifter_pos = 1;
    s.mpc_points = {{7.0, 4.5}, {5.0, 4.0}, {3.0, 3.0}};
    steps_.push_back(s);

    // Step 6
    s = makeStep("6_OutpostResponse", "前哨站响应: 正常战术, 前哨站未摧毁",
                 "期望: 导航前哨站, 云台前哨站扫描");
    s.pos_x = 17.0; s.pos_y = 9.0;
    s.health = 60.0f; s.bullets = 90; s.is_disengaged = true;
    s.coin_remaining = 350; s.base_hp = 3000; s.outpost_hp = 1200;
    s.lifter_pos = 0;
    steps_.push_back(s);

    // Step 7
    s = makeStep("7_LowAmmoRetreat", "弹药不足撤退: 弹药80<100阈值, 回家补给",
                 "期望: 导航RETREAT模式, 回家点");
    s.pos_x = 15.0; s.pos_y = -2.0;
    s.health = 55.0f; s.bullets = 80; s.is_disengaged = true;
    s.coin_remaining = 8; s.base_hp = 3000; s.outpost_hp = 1200;
    steps_.push_back(s);

    // Step 8
    s = makeStep("8_HomeSupply", "到家补给: 己方防守区, 金币58, 触发普通买弹",
                 "期望: resource_tree普通兑换触发, ammo_purchase_total增加");
    s.pos_x = 3.0; s.pos_y = 3.0;
    s.health = 60.0f; s.bullets = 80; s.is_disengaged = true;
    s.coin_remaining = 58; s.base_hp = 3000; s.outpost_hp = 1200;
    steps_.push_back(s);

    // Step 9
    s = makeStep("9_EnergyActive", "能量机关激活: 大能量机关激活, 可激活",
                 "期望: 能量机关响应, 脱战, 金币检查通过后激活");
    s.pos_x = 18.0; s.pos_y = 0.0;
    s.health = 70.0f; s.bullets = 200; s.is_disengaged = true;
    s.coin_remaining = 200; s.base_hp = 3000; s.outpost_hp = 1200;
    s.game_time = 240; s.event_code = (2u << 5);
    s.can_activate_energy = true;
    steps_.push_back(s);

    // Step 10
    s = makeStep("10_DefenseMode", "防守模式: 己方基地血量800, 堡垒空闲",
                 "期望: 防守战术, 导航己方堡垒");
    s.pos_x = 5.0; s.pos_y = 5.0;
    s.health = 65.0f; s.bullets = 190; s.is_disengaged = true;
    s.coin_remaining = 200; s.base_hp = 800; s.outpost_hp = 600;
    s.event_code = (2u << 25);
    steps_.push_back(s);

    // Step 11
    s = makeStep("11_DeathRevive", "死亡与复活: HP降至0, 可免费复活",
                 "期望: resource_tree FreeRevive触发, revive_request=1");
    s.pos_x = 5.0; s.pos_y = 8.0;
    s.health = 0.0f; s.bullets = 180; s.is_disengaged = true;
    s.coin_remaining = 150; s.base_hp = 600; s.outpost_hp = 400;
    s.enemy_robot_valid = true; s.enemy_pos_x = 5.0; s.enemy_pos_y = 8.0;
    s.can_free_resurrect = true;
    s.current_stance = 2;
    steps_.push_back(s);

    // Step 12
    s = makeStep("12_RecoveryPatrol", "恢复巡逻: 复活后HP恢复, 弹药充足, 回到巡逻",
                 "期望: 巡逻模式, 脱战, MOVE姿态");
    s.pos_x = 10.0; s.pos_y = 2.0;
    s.health = 30.0f; s.bullets = 170; s.is_disengaged = true;
    s.coin_remaining = 100; s.base_hp = 500; s.outpost_hp = 300;
    s.can_free_resurrect = false;
    steps_.push_back(s);
  }

  void publishAll()
  {
    const auto & s = steps_[current_];
    const auto now = this->now();

    publishTeamInfo(now, s);
    publishGameInfo(now, s);
    publishRadarInfo(now, s);
    publishOfflineInfo(now, s);
    publishOnlineInfo(now, s);
    publishOdom(now, s);
    publishMpc(now, s);
    publishTf(now, s);
  }

  void publishTeamInfo(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::TeamInformation msg;
    msg.header.stamp = now;
    msg.base_hp = s.base_hp;
    msg.outpost_hp = s.outpost_hp;

    // Ally infantry 3
    msg.allies[2].robot_id = ros_interfaces::msg::AllyRobotStatus::INFANTRY_3;
    msg.allies[2].robot_hp = 300;
    msg.allies[2].position.position.x = 10.0;
    msg.allies[2].position.position.y = 2.0;
    // Ally infantry 4
    msg.allies[3].robot_id = ros_interfaces::msg::AllyRobotStatus::INFANTRY_4;
    msg.allies[3].robot_hp = 250;
    msg.allies[3].position.position.x = 12.0;
    msg.allies[3].position.position.y = 0.0;

    team_pub_->publish(msg);
  }

  void publishGameInfo(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::GameInfo msg;
    msg.header.stamp = now;
    msg.game_time_remaining = s.game_time;
    msg.coin_remaining = s.coin_remaining;
    msg.event_code = s.event_code;
    msg.game_status = s.game_status;
    game_pub_->publish(msg);
  }

  void publishRadarInfo(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::RadarInfo msg;
    msg.header.stamp = now;
    msg.is_enemy_outpost_sensed = s.enemy_outpost_exists;
    msg.enemy_coin_left = s.enemy_coin_left;
    msg.enemy_coin_accumulated = 800;

    if (s.enemy_robot_valid) {
      msg.enemies[2].robot_id = s.enemy_robot_id;
      msg.enemies[2].robot_hp = s.enemy_robot_hp;
      msg.enemies[2].allowed_projectile = 150;
      msg.enemies[2].position.position.x = s.enemy_pos_x;
      msg.enemies[2].position.position.y = s.enemy_pos_y;
    }

    radar_pub_->publish(msg);
  }

  void publishOfflineInfo(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::SentryInfoOffline msg;
    msg.header.stamp = now;
    msg.is_get = s.target_valid;
    msg.armor_pos.x = static_cast<float>(s.target_x * 1000.0);
    msg.armor_pos.y = static_cast<float>(s.target_y * 1000.0);
    msg.armor_pos.z = 0.0f;
    msg.armor_num = s.armor_num;
    msg.yaw_encoder = s.yaw_encoder;
    msg.yaw_imu = s.yaw_imu;
    msg.lifter_current_pos = s.lifter_pos;
    msg.is_transformable = s.is_transformable;
    msg.transform_state = s.transform_state;
    msg.capacitor_capacity = s.capacitor;
    offline_pub_->publish(msg);
  }

  void publishOnlineInfo(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::SentryInfoOnline msg;
    msg.header.stamp = now;
    msg.self_health = static_cast<std::uint16_t>(s.health * kHealthScale);
    msg.bullets_remaining = static_cast<std::uint16_t>(s.bullets);
    msg.cooling_value = 30;
    msg.heat_limit = 260;
    msg.current_heat = static_cast<std::uint16_t>(s.current_heat);
    msg.sentry_pos.x = s.pos_x;
    msg.sentry_pos.y = s.pos_y;
    msg.sentry_pos.z = 0.0;
    msg.speed_monitor_angle = 0.0f;

    // sentry_info_1: assemble bitfields
    std::uint32_t info1 = 0;
    if (s.can_free_resurrect)      info1 |= (1u << 19);
    if (s.can_instant_resurrect)   info1 |= (1u << 20);
    info1 |= (static_cast<std::uint32_t>(s.instant_resurrect_cost & 0x3FF) << 21);
    info1 |= (static_cast<std::uint32_t>(s.remote_ammo_exchange_count & 0xF) << 11);
    msg.sentry_info_1 = info1;

    // sentry_info_2: is_disengaged + stance + can_activate_energy + remaining_ammo
    std::uint16_t info2 = 0;
    if (s.is_disengaged)           info2 |= 0x0001;
    info2 |= static_cast<std::uint16_t>((s.remaining_ammo_exchange & 0x7FF) << 1);
    info2 |= static_cast<std::uint16_t>((s.current_stance & 0x03) << 12);
    if (s.can_activate_energy)     info2 |= 0x4000;
    msg.sentry_info_2 = info2;

    online_pub_->publish(msg);
  }

  void publishOdom(const rclcpp::Time & now, const MatchStep & s)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now;
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = s.pos_x;
    msg.pose.pose.position.y = s.pos_y;
    msg.pose.pose.position.z = s.pos_z;

    tf2::Quaternion q;
    q.setRPY(0, 0, s.yaw_deg * M_PI / 180.0);
    msg.pose.pose.orientation = tf2::toMsg(q);
    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    odom_pub_->publish(msg);
  }

  void publishMpc(const rclcpp::Time & now, const MatchStep & s)
  {
    ros_interfaces::msg::MpcPositionCommand msg;
    msg.header.stamp = now;
    msg.command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
    for (const auto & pt : s.mpc_points) {
      ros_interfaces::msg::PositionCommand cmd;
      cmd.position.x = pt.first;
      cmd.position.y = pt.second;
      cmd.position.z = 0.0;
      cmd.yaw = 0.0;
      msg.cmds.push_back(cmd);
    }
    mpc_pub_->publish(msg);
  }

  void publishTf(const rclcpp::Time & now, const MatchStep & s)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "map";
    tf.child_frame_id = "camera_init";
    tf.transform.translation.x = s.pos_x;
    tf.transform.translation.y = s.pos_y;
    tf.transform.translation.z = s.pos_z;
    tf2::Quaternion q;
    q.setRPY(0, 0, s.yaw_deg * M_PI / 180.0);
    tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf);
  }

  void advanceStep()
  {
    if (steps_.empty()) return;
    current_ = (current_ + 1) % steps_.size();
    RCLCPP_INFO(get_logger(), "========================================");
    logCurrentStep();
  }

  void logCurrentStep() const
  {
    const auto & s = steps_[current_];
    RCLCPP_INFO(get_logger(), "[%zu/%zu] %s", current_ + 1, steps_.size(), s.name.c_str());
    RCLCPP_INFO(get_logger(), "  %s", s.description.c_str());
    RCLCPP_INFO(get_logger(), "  %s", s.expected_behavior.c_str());
    RCLCPP_INFO(get_logger(), "  HP=%.0f%% Bullets=%d Coin=%d Pos=(%.1f,%.1f) Disengaged=%d",
      s.health, s.bullets, s.coin_remaining, s.pos_x, s.pos_y, s.is_disengaged);
  }

  void logStatus() const
  {
    const auto & s = steps_[current_];
    const auto & b = last_behavior_;
    std::ostringstream oss;
    oss << "[S" << current_ + 1 << "/" << steps_.size() << ":" << s.name << "] ";
    oss << "stance=" << static_cast<int>(b.desired_stance)
        << " lifter=" << static_cast<int>(b.desire_lifter_pos)
        << " gyro=" << b.use_gyro_mode
        << " ammo_req=" << b.ammo_purchase_request
        << " revive=" << static_cast<int>(b.revive_request)
        << " ctrl=" << static_cast<int>(b.control_mode)
        << " | cmd_vel=(" << last_cmd_vel_.linear.x << "," << last_cmd_vel_.linear.y
        << "," << last_cmd_vel_.angular.z << ")";
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  // Publishers
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr mpc_pub_;

  // Subscribers
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr scenario_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  std::vector<MatchStep> steps_;
  std::size_t current_ = 0;

  ros_interfaces::msg::Behavior last_behavior_{};
  geometry_msgs::msg::Twist last_cmd_vel_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MatchSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
