#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/tf_utils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace Sentry_BT {

ros_interface::ros_interface(std::shared_ptr<Blackboard> & blackboard_ptr)
: Node("ros_interface_bt", rclcpp::NodeOptions()), blackboard_(blackboard_ptr)
{
  auto node_ptr = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {
  });
  param_manager_ = std::make_shared<ParamManager>(node_ptr);
  // 订阅全局信息话题
  team_info_sub = this->create_subscription<ros_interfaces::msg::TeamInformation>(
    "/sentry/team_info", 1, [this](const ros_interfaces::msg::TeamInformation::SharedPtr msg) {
      this->teamInfoCallback(msg);
    });

  // 订阅比赛信息话题
  game_info_sub = this->create_subscription<ros_interfaces::msg::GameInfo>(
    "/sentry/game_info", 1, [this](const ros_interfaces::msg::GameInfo::SharedPtr msg) {
      this->gameInfoCallback(msg);
    });

  // 订阅雷达信息话题
  radar_info_sub = this->create_subscription<ros_interfaces::msg::RadarInfo>(
    "/sentry/radar_info", 1, [this](const ros_interfaces::msg::RadarInfo::SharedPtr msg) {
      this->radarInfoCallback(msg);
    });

  // 订阅哨兵离线信息话题
  sentry_offline_sub = this->create_subscription<ros_interfaces::msg::SentryInfoOffline>(
    "/sentry/offline_info", 1, [this](const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg) {
      this->sentryOfflineCallback(msg);
    });

  // 订阅哨兵在线信息话题
  sentry_online_sub = this->create_subscription<ros_interfaces::msg::SentryInfoOnline>(
    "/sentry/online_info", 1, [this](const ros_interfaces::msg::SentryInfoOnline::SharedPtr msg) {
      this->sentryOnlineCallback(msg);
    });

  manual_override_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/sentry/manual_override_goal", 1, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      this->manualOverrideCallback(msg);
    });

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      geometry_msgs::msg::Pose raw_pose = msg->pose.pose;
      auto transform_utils =
        blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
      if (transform_utils) {
        geometry_msgs::msg::Pose base_link_origin;
        base_link_origin.orientation.w = 1.0;

        geometry_msgs::msg::Pose base_link_in_map;
        if (transform_utils->transformPoseToMap(base_link_origin, base_link_in_map, "base_link")) {
          // Keep raw orientation from odom, only override base_link x/y in map frame.
          raw_pose.position.x = base_link_in_map.position.x;
          raw_pose.position.y = base_link_in_map.position.y;
        }
      }
      {
        std::lock_guard<std::mutex> lock(current_pose_mutex_);
        current_pose_ = raw_pose;
      }
      blackboard_->set("current_pose", raw_pose);
    });

  mpc_cmd_sub = this->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
    "/opt_path", 1, [this](const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mpc_msg_mutex_);
      last_mpc_msg_ = msg;
      has_last_mpc_msg_ = true;
    });
  // 订阅外部速度指令
  cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      blackboard_->set("cmd_vel", *msg);
    });
  // 定时发布行为状态（10Hz）
  gimbal_yaw_pub = this->create_publisher<std_msgs::msg::Float32>("/sentry/gimbal_yaw", 10);
  behavior_pub = this->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  area_visualizer_ = std::make_unique<AreaVisualizer>(*this);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    const auto current_mode = blackboard_->get<int>("current_mode");
    const auto desired_stance = blackboard_->get<Sentry_BT::SentryStance>("desired_stance");
    const auto desired_lifter_pos = blackboard_->get<Sentry_BT::LifterPos>("desired_lifter_pos");
    const auto control_mode = blackboard_->get<Sentry_BT::ControlMode>("control_mode");
    const auto use_gyro_mode = blackboard_->get<bool>("use_gyro_mode");
    const auto gyro_vel = blackboard_->get<float>("gyro_vel");
    const auto ammo_purchase_request = blackboard_->get<uint16_t>("ammo_purchase_total");
    const auto yaw_min_deg = blackboard_->get<float>("scan_yaw_min_deg");
    const auto yaw_max_deg = blackboard_->get<float>("scan_yaw_max_deg");
    const auto tunnel_speed_y = blackboard_->get<float>("tunnel_speed_y");

    ros_interfaces::msg::Behavior behavior_msg;
    behavior_msg.desired_stance = static_cast<uint8_t>(desired_stance);
    behavior_msg.control_mode = static_cast<uint8_t>(control_mode);
    behavior_msg.use_gyro_mode = use_gyro_mode;
    behavior_msg.gyro_vel = gyro_vel;
    behavior_msg.desire_lifter_pos = static_cast<uint8_t>(desired_lifter_pos);
    behavior_msg.scan_yaw_min = yaw_min_deg;
    behavior_msg.scan_yaw_max = yaw_max_deg;
    behavior_msg.ammo_purchase_request = ammo_purchase_request;
    behavior_msg.tunnel_speed_y = tunnel_speed_y;
    behavior_pub->publish(behavior_msg);

    const auto cmd_vel = blackboard_->get<geometry_msgs::msg::Twist>("cmd_vel");
    cmd_vel_pub->publish(cmd_vel);
  });

  area_marker_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
    area_visualizer_->publishAreaMarkers(this->now());
  });
  area_visualizer_->publishAreaMarkers(this->now());

  tunnel_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    ros_interfaces::msg::MpcPositionCommand::SharedPtr snapshot;
    {
      std::lock_guard<std::mutex> lock(mpc_msg_mutex_);
      snapshot = last_mpc_msg_;
    }
    bool through = isTroughTunnel(snapshot, tunnel_zone);
    blackboard_->set("through_tunnel", through);
  });
}

geometry_msgs::msg::Pose ros_interface::transformMapPose(
  const geometry_msgs::msg::Pose & input_pose, const std::string & target_frame)
{
  auto transform_utils = blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  geometry_msgs::msg::Pose output_pose;
  if (transform_utils && transform_utils->transformMapPose(input_pose, output_pose, target_frame)) {
    return output_pose;
  }
  return input_pose;
}

// 新增：全局信息回调函数
void ros_interface::teamInfoCallback(const ros_interfaces::msg::TeamInformation::SharedPtr msg)
{
  // 提取基地和前哨站血量
  blackboard_->set<int>("home_health", static_cast<int>(msg->base_hp));
  blackboard_->set<int>("own_outpost_health", static_cast<int>(msg->outpost_hp));

  // 处理队友信息
  ros_interfaces::msg::TeamInformation team_info = *msg;
  std::vector<AllyRobotInfo> allies_info;
  allies_info.reserve(msg->allies.size());
  const auto tf_utils_node =
    blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");

  for (const auto & ally : msg->allies) {
    AllyRobotInfo info;
    info.robot_id = ally.armor_id;  // 将armor_id映射到robot_id
    info.remain_hp = static_cast<int>(ally.remain_hp);
    if (tf_utils_node) {
      if (tf_utils_node->transformPoseToMap(ally.position, info.position, "minimap")) {}
    }

    allies_info.push_back(info);
  }

  // 保存转换后的团队信息
  blackboard_->set<std::vector<AllyRobotInfo>>("allies_info", allies_info);
}

// 新增：比赛信息回调函数
void ros_interface::gameInfoCallback(const ros_interfaces::msg::GameInfo::SharedPtr msg)
{
  // 存储比赛基本信息
  blackboard_->set<int>("game_time_remaining", static_cast<int>(msg->game_time_remaining));
  blackboard_->set<int>("coin_remaining", static_cast<int>(msg->coin_remaining));
  blackboard_->set<int>("game_status", static_cast<int>(msg->game_status));

  // 解码event_code字段
  uint32_t event_code = msg->event_code;

  // 提取bit 3-4：己方小能量机关的激活状态
  uint8_t small_energy_status = (event_code >> 3) & 0x3;
  blackboard_->set<int>("small_energy_status", static_cast<int>(small_energy_status));

  // 提取bit 5-6：己方大能量机关的激活状态
  uint8_t big_energy_status = (event_code >> 5) & 0x3;
  blackboard_->set<int>("big_energy_status", static_cast<int>(big_energy_status));

  // 提取bit 25-26：己方堡垒增益点的占领状态
  uint8_t fort_occupation_status = (event_code >> 25) & 0x3;
  blackboard_->set<int>("fort_occupation_status", static_cast<int>(fort_occupation_status));
}

// 新增：雷达信息回调函数
void ros_interface::radarInfoCallback(const ros_interfaces::msg::RadarInfo::SharedPtr msg)
{
  // 存储敌方信息
  blackboard_->set<int>("enemy_coin_left", static_cast<int>(msg->enemy_coin_left));
  blackboard_->set<int>("enemy_coin_accumulated", static_cast<int>(msg->enemy_coin_accumulated));
  blackboard_->set<bool>("enemy_outpost_destroyed", !(msg->is_enemy_outpost_sensed));

  // 存储所有敌方机器人状态
  ros_interfaces::msg::RadarInfo radar_info = *msg;

  std::vector<EnemyRobotInfo> enemies_info;
  enemies_info.reserve(msg->enemies.size());
  const auto tf_utils_node =
    blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  for (auto & enemy : radar_info.enemies) {
    if (enemy.robot_id > 0) {  // 有效敌方
      EnemyRobotInfo info;
      info.remain_hp = static_cast<int>(enemy.robot_hp);
      info.robot_id = static_cast<int>(enemy.robot_id);
      if (tf_utils_node) {
        tf_utils_node->transformPoseToMap(enemy.position, info.position, "minimap");
      }
      info.allowed_projectile = static_cast<int>(enemy.allowed_projectile);
      enemies_info.push_back(info);
    }
  }

  blackboard_->set<std::vector<EnemyRobotInfo>>("enemies_info", enemies_info);
}

void ros_interface::manualOverrideCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  const auto tf_utils_node =
    blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  geometry_msgs::msg::Point manual_goal_in_map = msg->point;
  if (tf_utils_node) {
    geometry_msgs::msg::Pose input_pose;
    input_pose.position = msg->point;
    input_pose.orientation.w = 1.0;
    geometry_msgs::msg::Pose output_pose;
    if (tf_utils_node->transformPoseToMap(input_pose, output_pose, "minimap")) {
      manual_goal_in_map = output_pose.position;
    }
  }

  const Point2D manual_goal{manual_goal_in_map.x, manual_goal_in_map.y, 0.0};
  blackboard_->set("manual_override_goal", manual_goal);
}

// 新增：哨兵离线信息回调函数
void ros_interface::sentryOfflineCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg)
{
  // 存储哨兵离线状态信息
  blackboard_->set<bool>("target_valid", msg->is_get);
  blackboard_->set<float>("gimbal_yaw", msg->yaw_imu);
  blackboard_->set<Sentry_BT::LifterPos>(
    "lifter_current_pos", static_cast<Sentry_BT::LifterPos>(msg->lifter_current_pos));
  blackboard_->set<bool>("is_transformable", msg->is_transformable);
  blackboard_->set<float>("transform_state", msg->transform_state);
  blackboard_->set<uint8_t>("capacitor_capacity", msg->capacitor_capacity);
  auto tf_utils = blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  tf_utils->updateGimbalYaw(msg->yaw_imu);
  // 存储装甲板位置
  if (msg->is_get)
  // if(false)
  {
    geometry_msgs::msg::Pose target_pose_in, target_pose;

    target_pose_in.position.x = (msg->armor_pos.x) / 1000.0;  // 转换为米
    target_pose_in.position.y = (msg->armor_pos.y) / 1000.0;
    target_pose_in.position.z = (msg->armor_pos.z) / 1000.0;

    if (tf_utils) {
      tf_utils->transformPoseToMap(target_pose_in, target_pose, "gimbal");
    }
    target_pose.orientation.w = 1.0;  // 设置默认朝向
    blackboard_->set<int>("target_armor_id", (int)msg->armor_num);
    blackboard_->set<geometry_msgs::msg::Pose>("target_pose", target_pose);
  }
}

// 新增：哨兵在线信息回调函数
void ros_interface::sentryOnlineCallback(const ros_interfaces::msg::SentryInfoOnline::SharedPtr msg)
{
  // 存储哨兵在线状态信息
  blackboard_->set<float>("health", ((int)msg->self_health / 4));
  blackboard_->set<int>("bullets_remaining", static_cast<int>(msg->bullets_remaining));
  blackboard_->set<int>("cooling_value", static_cast<int>(msg->cooling_value));
  blackboard_->set<int>("heat_limit", static_cast<int>(msg->heat_limit));
  blackboard_->set<int>("current_heat", static_cast<int>(msg->current_heat));
  blackboard_->set<float>("speed_monitor_angle", msg->speed_monitor_angle);

  // 存储哨兵位置
  const auto tf_utils_node =
    blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  geometry_msgs::msg::Point sentry_position = msg->sentry_pos;
  if (tf_utils_node) {
    geometry_msgs::msg::Pose input_pose;
    input_pose.position = msg->sentry_pos;
    input_pose.orientation.w = 1.0;
    geometry_msgs::msg::Pose output_pose;
    if (tf_utils_node->transformPoseToMap(input_pose, output_pose, "minimap")) {
      sentry_position = output_pose.position;
    }
  }
  blackboard_->set<geometry_msgs::msg::Point>("sentry_position", sentry_position);

  // 解码sentry_info_2
  uint16_t sentry_info_2 = msg->sentry_info_2;

  // 提取bit 0：脱战状态
  bool is_disengaged = (sentry_info_2 & 0x0001) != 0;
  // blackboard_->set<bool>("is_disengaged", is_disengaged);

  // 提取bit 12-13：哨兵当前姿态
  uint8_t current_stance = (sentry_info_2 >> 12) & 0x3;
  if (current_stance >= static_cast<uint8_t>(Sentry_BT::SentryStance::ATTACK) &&
      current_stance <= static_cast<uint8_t>(Sentry_BT::SentryStance::MOVE)) {
    // blackboard_->set<Sentry_BT::SentryStance>(
    //   "current_stance", static_cast<Sentry_BT::SentryStance>(current_stance));
  } else {
    // RCLCPP_WARN_THROTTLE(this->get_logger(),
    //   *this->get_clock(),
    //   2000,
    //   "23%u",
    //   current_stance);
  }

  // 提取bit 14：己方能量机关是否能够进入正在激活状态
  bool can_activate_energy = ((sentry_info_2 >> 14) & 0x1) != 0;
  blackboard_->set<bool>("can_activate_energy", can_activate_energy);

  // 解码sentry_info_1的兑换信息
  uint32_t sentry_info_1 = msg->sentry_info_1;

  // // bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量
  // uint16_t ammo_exchanged_local = (sentry_info_1 & 0x07FF);
  // blackboard_->set<int>("ammo_exchanged_local", static_cast<int>(ammo_exchanged_local));

  // // bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数
  // uint8_t remote_ammo_exchange_count = (sentry_info_1 >> 11) & 0xF;
  // blackboard_->set<int>("remote_ammo_exchange_count", static_cast<int>(remote_ammo_exchange_count));

  // // bit 15-18：哨兵机器人成功远程兑换血量的次数
  // uint8_t remote_health_exchange_count = (sentry_info_1 >> 15) & 0xF;
  // blackboard_->set<int>("remote_health_exchange_count", static_cast<int>(remote_health_exchange_count));

  // bit 19：哨兵机器人当前是否可以确认免费复活
  bool can_free_resurrect = ((sentry_info_1 >> 19) & 0x1) != 0;
  blackboard_->set<bool>("can_free_resurrect", can_free_resurrect);

  // bit 20：哨兵机器人当前是否可以兑换立即复活
  bool can_instant_resurrect = ((sentry_info_1 >> 20) & 0x1) != 0;
  blackboard_->set<bool>("can_instant_resurrect", can_instant_resurrect);

  // bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数
  uint16_t instant_resurrect_cost = (sentry_info_1 >> 21) & 0x3FF;
  blackboard_->set<int>("instant_resurrect_cost", static_cast<int>(instant_resurrect_cost));

  // // bit 1-11：队伍17mm允许发弹量的剩余可兑换数
  // uint16_t remaining_ammo_exchange = (sentry_info_2 >> 1) & 0x7FF;
  // blackboard_->set<int>("remaining_ammo_exchange", static_cast<int>(remaining_ammo_exchange));
}

// 判断MPC轨迹是否穿过指定矩形区域
bool ros_interface::isTroughZone(
  const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Area_Square & zone)
{
  if (!msg) {
    return false;
  }
  for (const auto & cmd : msg->cmds) {
    Point2D point{cmd.position.x, cmd.position.y};
    if (zone.contains(point)) {
      return true;
    }
  }
  return false;
}

// 判断MPC轨迹是否穿过指定隧道区域（由入口左端点和出口右端点两个点定义）
bool ros_interface::isTroughTunnel(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg,
  const std::array<Area_Square, 4> & tunnel_areas)
{
  const Point2D current_point{current_pose_.position.x, current_pose_.position.y};
  bool in_transform_zone = false;
  for (const auto & zone : transform_zone) {
    if (zone.contains(current_point)) {
      in_transform_zone = true;
      break;
    }
  }
  bool through_tunnel_now = false;
  for (const auto & zone : tunnel_areas) {
    if (isTroughZone(msg, zone)) {
      through_tunnel_now = true;
      break;
    }
  }
  if (!msg || msg->cmds.empty()) {
    if (!in_transform_zone) {
      tunnel_detect_latched_ = false;
    }
    return in_transform_zone && tunnel_detect_latched_;
  }

  if (!in_transform_zone) {
    tunnel_detect_latched_ = false;
  } else if (through_tunnel_now) {
    tunnel_detect_latched_ = true;
  }

  const bool through_tunnel_stable = in_transform_zone && tunnel_detect_latched_;
  return through_tunnel_stable;
}
geometry_msgs::msg::Pose ros_interface::createPose(float x, float y, float z, float yaw_deg)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // 将yaw角转换为四元数
  float yaw_rad = yaw_deg * M_PI / 180.0F;
  pose.orientation.x = 0.0F;
  pose.orientation.y = 0.0F;
  pose.orientation.z = std::sin(yaw_rad / 2.0F);
  pose.orientation.w = std::cos(yaw_rad / 2.0F);

  return pose;
}
}  // namespace Sentry_BT