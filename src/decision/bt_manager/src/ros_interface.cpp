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
  this->declare_parameter<bool>("publish_area_markers", true);
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
        transform_utils->updateCurrentPose(raw_pose);
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
  behavior_pub = this->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  area_visualizer_ = std::make_unique<AreaVisualizer>(*this);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    const auto current_mode = blackboard_->get<NavMode>("current_mode");
    const auto desired_stance = blackboard_->get<Sentry_BT::SentryStance>("desired_stance");
    const auto desired_lifter_pos = blackboard_->get<Sentry_BT::LifterPos>("desired_lifter_pos");
    const auto use_gyro_mode = blackboard_->get<bool>("use_gyro_mode");
    const auto gyro_vel = blackboard_->get<float>("gyro_vel");
    const auto ammo_purchase_request = static_cast<uint16_t>(blackboard_->get<int>("ammo_purchase_total"));
    const auto yaw_min_deg = blackboard_->get<float>("scan_yaw_min_deg");
    const auto yaw_max_deg = blackboard_->get<float>("scan_yaw_max_deg");
    const auto use_limited_scan = blackboard_->get<bool>("use_limited_scan");
    const auto revive_request = blackboard_->get<uint8_t>("revive_request");
    const auto remote_revive_request = blackboard_->get<uint8_t>("remote_revive_request");
    const auto remote_ammo_request = blackboard_->get<uint8_t>("remote_ammo_request");
    const auto remote_health_request = blackboard_->get<uint8_t>("remote_health_request");
    const auto pitch_mode = blackboard_->get<PitchPos>("pitch_mode");
    const auto not_aim_enemy = blackboard_->get<bool>("not_aim_enemy");

    ros_interfaces::msg::Behavior behavior_msg;
    behavior_msg.desired_stance = static_cast<uint8_t>(desired_stance);
    behavior_msg.use_gyro_mode = use_gyro_mode;
    behavior_msg.gyro_vel = gyro_vel;
    behavior_msg.desire_lifter_pos = static_cast<uint8_t>(desired_lifter_pos);
    behavior_msg.scan_yaw_min = yaw_min_deg;
    behavior_msg.scan_yaw_max = yaw_max_deg;
    behavior_msg.use_limited_scan = use_limited_scan;
    behavior_msg.ammo_purchase_request = ammo_purchase_request;
    behavior_msg.revive_request = revive_request;
    behavior_msg.remote_revive_request = remote_revive_request;
    behavior_msg.remote_ammo_request = remote_ammo_request;
    behavior_msg.remote_health_request = remote_health_request;
    behavior_msg.pitch_mode = static_cast<uint8_t>(pitch_mode);
    behavior_msg.not_aim_enemy = not_aim_enemy;
    behavior_pub->publish(behavior_msg);

    if (revive_request != 0) {
      blackboard_->set<uint8_t>("revive_request", 0);
    }
  });

  area_marker_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
    const bool publish_area_markers = this->get_parameter("publish_area_markers").as_bool();
    if (publish_area_markers) {
      area_visualizer_->publishAreaMarkers(this->now());
      area_markers_published_ = true;
    } else if (area_markers_published_) {
      area_visualizer_->clearAreaMarkers(this->now());
      area_markers_published_ = false;
    }
  });
  if (this->get_parameter("publish_area_markers").as_bool()) {
    area_visualizer_->publishAreaMarkers(this->now());
    area_markers_published_ = true;
  }

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
    info.robot_id = ally.robot_id;
    info.remain_hp = static_cast<int>(ally.robot_hp);
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
  // std::cout << "big_energy_status:" << static_cast<int>(big_energy_status) <<std::endl;
  // 提取bit 25-26：己方堡垒增益点的占领状态
  uint8_t fort_occupation_status = (event_code >> 25) & 0x3;
  blackboard_->set<int>("fort_occupation_status", static_cast<int>(fort_occupation_status));

  geometry_msgs::msg::Pose manual_point_in, manual_point;
  manual_point_in.position.x = msg->manual_point_x;
  manual_point_in.position.y = msg->manual_point_y;
  manual_point_in.position.z = 0.0;
  manual_point_in.orientation.w = 1.0;
  auto tf_utils = blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  if (tf_utils) {
    tf_utils->transformPoseToMap(manual_point_in, manual_point, "minimap");
  }
  Point2D manual_point_2d(manual_point.position.x, manual_point.position.y);
  static Point2D last_manual_point(0.0, 0.0);
  blackboard_->set<Point2D>("manual_override_goal", manual_point_2d);
  const bool manual_point_changed =
    std::hypot(manual_point.position.x - last_manual_point.x,
      manual_point.position.y - last_manual_point.y) >= 0.001;
  if (!manual_point_changed) {
    return;
  }
  last_manual_point = manual_point_2d;
  switch (msg->manual_key) {
  case 65:  // 'A'键切换控制模式
    blackboard_->set<ControlMode>("control_mode", ControlMode::MANUAL_CONTROL);
    break;
  case 66: {
    const auto enemy_outpost_destroyed = blackboard_->get<bool>("enemy_outpost_destroyed");
    blackboard_->set<bool>("enemy_outpost_destroyed", !enemy_outpost_destroyed);
    break;
  }
  case 0:  // '0'键切换回自动模式
    blackboard_->set<ControlMode>("control_mode", ControlMode::AUTO);
    break;
  default:
    break;
  }
  std::cout << "Received manual_key: " << static_cast<int>(msg->manual_key) << std::endl;
}

// 新增：雷达信息回调函数
void ros_interface::radarInfoCallback(const ros_interfaces::msg::RadarInfo::SharedPtr msg)
{
  // 存储敌方信息
  blackboard_->set<int>("enemy_coin_left", static_cast<int>(msg->enemy_coin_left));
  blackboard_->set<int>("enemy_coin_accumulated", static_cast<int>(msg->enemy_coin_accumulated));

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

// 新增：哨兵离线信息回调函数
void ros_interface::sentryOfflineCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg)
{
  // 存储哨兵离线状态信息
  blackboard_->set<bool>("target_valid", msg->is_get);
  blackboard_->set<Sentry_BT::LifterPos>(
    "lifter_current_pos", static_cast<Sentry_BT::LifterPos>(msg->lifter_current_pos));
  blackboard_->set<bool>("is_transformable", msg->is_transformable);
  blackboard_->set<float>("transform_state", msg->transform_state);
  blackboard_->set<uint8_t>("capacitor_capacity", msg->capacitor_capacity);
  const auto current_pose = blackboard_->get<geometry_msgs::msg::Pose>("current_pose");
  auto tf_utils = blackboard_->get<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils");
  float gimbal_yaw_init = msg->yaw_camerainit_to_gimbal;
  if (tf_utils) {
    tf_utils->updateGimbalYawInit(gimbal_yaw_init);
  }
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
  blackboard_->set<float>("gimbal_yaw", msg->speed_monitor_angle);

  // 解码sentry_info_2
  uint16_t sentry_info_2 = msg->sentry_info_2;

  // 提取bit 0：脱战状态
  bool is_disengaged = (sentry_info_2 & 0x0001) != 0;
  blackboard_->set<bool>("is_disengaged", is_disengaged);

  // 提取bit 12-13：哨兵当前姿态
  uint8_t current_stance = (sentry_info_2 >> 12) & 0x3;
  if (current_stance >= static_cast<uint8_t>(Sentry_BT::SentryStance::ATTACK) &&
      current_stance <= static_cast<uint8_t>(Sentry_BT::SentryStance::MOVE)) {
    blackboard_->set<Sentry_BT::SentryStance>(
      "current_stance", static_cast<Sentry_BT::SentryStance>(current_stance));
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

  // bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量
  uint16_t ammo_exchanged_local = (sentry_info_1 & 0x07FF);
  blackboard_->set<int>("ammo_exchanged_local", static_cast<int>(ammo_exchanged_local));

  // bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数
  uint8_t remote_ammo_exchange_count = (sentry_info_1 >> 11) & 0xF;
  blackboard_->set<int>("remote_ammo_exchange_count", static_cast<int>(remote_ammo_exchange_count));

  // bit 15-18：哨兵机器人成功远程兑换血量的次数
  uint8_t remote_health_exchange_count = (sentry_info_1 >> 15) & 0xF;
  blackboard_->set<int>("remote_health_exchange_count", static_cast<int>(remote_health_exchange_count));

  // bit 19：哨兵机器人当前是否可以确认免费复活
  bool can_free_resurrect = ((sentry_info_1 >> 19) & 0x1) != 0;
  blackboard_->set<bool>("can_free_resurrect", can_free_resurrect);

  // bit 20：哨兵机器人当前是否可以兑换立即复活
  bool can_instant_resurrect = ((sentry_info_1 >> 20) & 0x1) != 0;
  blackboard_->set<bool>("can_instant_resurrect", can_instant_resurrect);

  // bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数
  uint16_t instant_resurrect_cost = (sentry_info_1 >> 21) & 0x3FF;
  blackboard_->set<int>("instant_resurrect_cost", static_cast<int>(instant_resurrect_cost));

  // bit 1-11：队伍17mm允许发弹量的剩余可兑换数
  uint16_t remaining_ammo_exchange = (sentry_info_2 >> 1) & 0x7FF;
  blackboard_->set<int>("remaining_ammo_exchange", static_cast<int>(remaining_ammo_exchange));
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
  const auto current_pose = blackboard_->get<geometry_msgs::msg::Pose>("current_pose");
  const Point2D current_point{current_pose.position.x, current_pose.position.y};
  bool in_transform_zone = false;
  for (std::size_t i = 0; i < transform_zone.size(); ++i) {
    if (transform_zone[i].contains(current_point)) {
      in_transform_zone = true;
      break;
    }
  }

  int nearest_tunnel_idx = -1;
  double nearest_dist2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < tunnel_zone.size(); ++i) {
    const auto & zone = tunnel_zone[i];
    const double center_x = (zone.top_left.x + zone.bottom_right.x) * 0.5;
    const double center_y = (zone.top_left.y + zone.bottom_right.y) * 0.5;
    const double dx = current_point.x - center_x;
    const double dy = current_point.y - center_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < nearest_dist2) {
      nearest_dist2 = dist2;
      nearest_tunnel_idx = static_cast<int>(i);
    }
  }
  // std::cout << "Current position: (" << current_point.x << ", " << current_point.y << "),in_transform_zone: "
  //           << in_transform_zone
  //           << ", nearest_tunnel_idx: " << nearest_tunnel_idx << std::endl;
  bool current_in_tunnel = false;
  for (const auto & zone : tunnel_zone) {
    if (zone.contains(current_point)) {
      current_in_tunnel = true;
      break;
    }
  }
  blackboard_->set("current_in_tunnel", current_in_tunnel);
  blackboard_->set("in_transform_zone", in_transform_zone);
  blackboard_->set("nearest_tunnel_idx", nearest_tunnel_idx);

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
    blackboard_->set("through_tunnel", in_transform_zone && tunnel_detect_latched_);
    return in_transform_zone && tunnel_detect_latched_;
  }

  if (!in_transform_zone) {
    tunnel_detect_latched_ = false;
  } else if (through_tunnel_now) {
    tunnel_detect_latched_ = true;
  }

  const bool through_tunnel_stable = in_transform_zone && tunnel_detect_latched_;
  blackboard_->set("through_tunnel", through_tunnel_stable);
  return through_tunnel_stable;
}
}  // namespace Sentry_BT
