#include "bt_manager/ros_interface.hpp"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <string>

namespace Sentry_BT
{
  ros_interface::ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr)
    : Node("ros_interface_" +
               std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 10000),
           rclcpp::NodeOptions().use_global_arguments(false))
    , blackboard_(blackboard_ptr)
  {
    auto node_ptr = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
    param_manager_ = std::make_shared<ParamManager>(node_ptr);

    // 订阅全局信息话题
    team_info_sub = this->create_subscription<ros_interfaces::msg::TeamInformation>(
        "/sentry/team_info",
        1,
        [this](const ros_interfaces::msg::TeamInformation::SharedPtr msg) { this->teamInfoCallback(msg); });

    // 订阅比赛信息话题
    game_info_sub = this->create_subscription<ros_interfaces::msg::GameInfo>(
        "/sentry/game_info",
        1,
        [this](const ros_interfaces::msg::GameInfo::SharedPtr msg) { this->gameInfoCallback(msg); });

    // 订阅雷达信息话题
    radar_info_sub = this->create_subscription<ros_interfaces::msg::RadarInfo>(
        "/sentry/radar_info",
        1,
        [this](const ros_interfaces::msg::RadarInfo::SharedPtr msg) { this->radarInfoCallback(msg); });

    // 订阅哨兵离线信息话题
    sentry_offline_sub = this->create_subscription<ros_interfaces::msg::SentryInfoOffline>(
        "/sentry/offline_info",
        1,
        [this](const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg) { this->sentryOfflineCallback(msg); });

    // 订阅哨兵在线信息话题
    sentry_online_sub = this->create_subscription<ros_interfaces::msg::SentryInfoOnline>(
        "/sentry/online_info",
        1,
        [this](const ros_interfaces::msg::SentryInfoOnline::SharedPtr msg) { this->sentryOnlineCallback(msg); });

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
                                                                  1,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    // 更新当前位置
                                                                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                                                                    current_pose_ = msg->pose.pose;
                                                                  }); 
            
      // 订阅MPC轨迹指令
    mpc_cmd_sub = this->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
        "/opt_path", 1, [this](const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg)
        {
          //std::cout << "Received MPC command with horizon: " << msg->mpc_horizon << std::endl;
          blackboard_->set("through_tunnel", isTroughTunnel(msg, Point2D{9.46, 2.65}, Point2D{10.40, 1.80}));
        });
    // 订阅外部速度指令
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          blackboard_->set("cmd_vel", *msg);
        });
    // 定时发布行为状态（10Hz）
    gimbal_yaw_pub = this->create_publisher<std_msgs::msg::Float32>("/sentry/gimbal_yaw", 10);
    behavior_pub = this->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          const auto current_mode = blackboard_->get<int>("current_mode");
          const auto desired_stance = blackboard_->get<Sentry_BT::SentryStance>("desired_stance");
          const auto desired_lifter_pos = blackboard_->get<int>("desired_lifter_pos");
          const auto current_pose = getCurrentPose();

          const bool is_reach_outpost_enemy =
              current_mode == Sentry_BT::NavMode::RESPONSE &&
              std::hypot(current_pose.position.x - nav_points[2].x, current_pose.position.y - nav_points[2].y) < 1.0;

          const bool is_reach_outpost_own =
              std::hypot(current_pose.position.x - nav_points[0].x, current_pose.position.y - nav_points[0].y) < 1.0;

          blackboard_->set<bool>("outpost_msg", is_reach_outpost_enemy);

          ros_interfaces::msg::Behavior behavior_msg;
          behavior_msg.desired_stance = static_cast<int8_t>(desired_stance);
          behavior_msg.is_reach_outpost_enemy = is_reach_outpost_enemy;
          behavior_msg.is_reach_outpost_own = is_reach_outpost_own;
          behavior_msg.desire_lifter_pos = desired_lifter_pos;
          behavior_pub->publish(behavior_msg);
        });
  }

  geometry_msgs::msg::Pose ros_interface::getCurrentPose() const
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    auto transforme_utils = std::make_shared<Sentry_BT::TransformUtils>();
    geometry_msgs::msg::Pose transformed_pose;
    if(transforme_utils->transformPoseToMap(current_pose_, transformed_pose, "camera_init"))
    {      
      return transformed_pose;
    }
    return current_pose_;
  }

  // 新增：全局信息回调函数
  void ros_interface::teamInfoCallback(const ros_interfaces::msg::TeamInformation::SharedPtr msg)
  {
    // 保存完整的团队信息
    blackboard_->set<ros_interfaces::msg::TeamInformation>("team_info", *msg);
    
    // 提取基地和前哨站血量
    blackboard_->set<int>("home_health", static_cast<int>(msg->base_hp));
    blackboard_->set<int>("own_outpost_health", static_cast<int>(msg->outpost_hp));
    
    // 处理队友信息
    for (const auto& ally : msg->allies) {
      switch (ally.armor_id) {
        case ros_interfaces::msg::AllyRobotStatus::HERO:
          blackboard_->set<int>("hero_hp", static_cast<int>(ally.remain_hp));
          blackboard_->set<geometry_msgs::msg::Point>("hero_position", ally.position);
          break;
          
        case ros_interfaces::msg::AllyRobotStatus::ENGINEER:
          blackboard_->set<int>("engineer_hp", static_cast<int>(ally.remain_hp));
          blackboard_->set<geometry_msgs::msg::Point>("engineer_position", ally.position);
          break;
          
        case ros_interfaces::msg::AllyRobotStatus::INFANTRY_3:
          blackboard_->set<int>("infantry3_hp", static_cast<int>(ally.remain_hp));
          blackboard_->set<geometry_msgs::msg::Point>("infantry3_position", ally.position);
          break;
          
        case ros_interfaces::msg::AllyRobotStatus::INFANTRY_4:
          blackboard_->set<int>("infantry4_hp", static_cast<int>(ally.remain_hp));
          blackboard_->set<geometry_msgs::msg::Point>("infantry4_position", ally.position);
          break;
          
        default:
          break;
      }
    }
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

    // 可以存储原始event_code以备后用
    blackboard_->set<uint32_t>("event_code_raw", event_code);
  }

  // 新增：雷达信息回调函数
  void ros_interface::radarInfoCallback(const ros_interfaces::msg::RadarInfo::SharedPtr msg)
  {
    // 存储敌方信息
    blackboard_->set<int>("enemy_coin_left", static_cast<int>(msg->enemy_coin_left));
    blackboard_->set<int>("enemy_coin_accumulated", static_cast<int>(msg->enemy_coin_accumulated));
    blackboard_->set<bool>("enemy_outpost_destroyed", !(msg->is_enemy_outpost_sensed));
    
    // 存储所有敌方机器人状态
    for (size_t i = 0; i < msg->enemies.size(); ++i) {
      const auto& enemy = msg->enemies[i];
      if (enemy.robot_id > 0) {  // 有效敌方
        std::string id_str = std::to_string(enemy.robot_id);
        blackboard_->set<int>("enemy_hp_" + id_str, static_cast<int>(enemy.robot_hp));
        blackboard_->set<int>("enemy_ammo_" + id_str, static_cast<int>(enemy.allowed_projectile));
        
      // 存储敌方位置
      blackboard_->set<geometry_msgs::msg::Point>("enemy_position_" + id_str, enemy.position);
      }
    }
    
    // 可以存储完整的雷达信息
    blackboard_->set<ros_interfaces::msg::RadarInfo>("radar_info", *msg);
  }

  // 新增：哨兵离线信息回调函数
  void ros_interface::sentryOfflineCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg)
  {
    // 存储哨兵离线状态信息
    blackboard_->set<bool>("target_valid", msg->is_get);
    blackboard_->set<float>("gimbal_yaw", msg->yaw_imu);
    blackboard_->set<int>("lifter_current_pos", static_cast<int>(msg->lifter_current_pos));
    blackboard_->set<bool>("is_transformable", msg->is_transformable);
    blackboard_->set<float>("transform_state", msg->transform_state);
    
    // 存储装甲板位置
    if(msg->is_get)
    // if(false)
    {
      geometry_msgs::msg::Pose target_pose_in, target_pose;
      static bool has_last_logged_pose = false;
      static geometry_msgs::msg::Pose last_target_pose_in;
      static geometry_msgs::msg::Pose last_target_pose;

      target_pose_in.position.x = (msg->armor_pos.x) / 1000.0;  // 转换为米
      target_pose_in.position.y = (msg->armor_pos.y) / 1000.0;
      target_pose_in.position.z = (msg->armor_pos.z) / 1000.0;//?
      TransformPose(target_pose_in, target_pose);

      // Quiet logging: only print when target input/output pose changes significantly.
      const double input_diff = std::hypot(
          target_pose_in.position.x - last_target_pose_in.position.x,
          target_pose_in.position.y - last_target_pose_in.position.y);
      const double output_diff = std::hypot(
          target_pose.position.x - last_target_pose.position.x,
          target_pose.position.y - last_target_pose.position.y);
      const bool should_log = !has_last_logged_pose || input_diff > 0.5 || output_diff > 0.5;

      if(should_log)
      {
        // std::cout << "Target pose: " << target_pose.position.x << ", " << target_pose.position.y << std::endl;
        // std::cout << "target pose in: " << target_pose_in.position.x << "," << target_pose_in.position.y
        //           << "," << target_pose_in.position.z << std::endl;
        last_target_pose_in = target_pose_in;
        last_target_pose = target_pose;
        has_last_logged_pose = true;
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
    geometry_msgs::msg::Point sentry_position;
    sentry_position.x = msg->sentry_pos.x;
    sentry_position.y = msg->sentry_pos.y;
    sentry_position.z = 0.0;
    blackboard_->set<geometry_msgs::msg::Point>("sentry_position", sentry_position);
    
    // 存储原始信息
    blackboard_->set<uint32_t>("sentry_info_1_raw", msg->sentry_info_1);
    blackboard_->set<uint16_t>("sentry_info_2_raw", msg->sentry_info_2);
    
    // 解码sentry_info_2
    uint16_t sentry_info_2 = msg->sentry_info_2;
    
    // 提取bit 0：脱战状态
    bool is_disengaged = (sentry_info_2 & 0x0001) != 0;
    blackboard_->set<bool>("is_disengaged", is_disengaged);
    
    // 提取bit 12-13：哨兵当前姿态
    uint8_t current_stance = (sentry_info_2 >> 12) & 0x3;
    blackboard_->set<Sentry_BT::SentryStance>("current_stance", static_cast<Sentry_BT::SentryStance>(current_stance));
    
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

  bool ros_interface::TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose)
  {
    // 创建TransformUtils实例
    auto transform_utils = std::make_shared<Sentry_BT::TransformUtils>();

    // 执行坐标转换
    bool success = transform_utils->transformPoseToMap(input_pose, output_pose, "gimbal");

    return success;
  }

  // 判断MPC轨迹是否穿过指定矩形区域
  bool ros_interface::isTroughZone(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Area_Square& zone)
  {
    for (const auto& cmd : msg->cmds)
    {
      Point2D point{cmd.position.x, cmd.position.y};
      if (zone.contains(point))
      {
        return true;
      }
    }
    return false;
  }

  // 判断MPC轨迹是否穿过指定隧道区域（由入口左端点和出口右端点两个点定义）
  bool ros_interface::isTroughTunnel(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Point2D& tunnel_entry, const Point2D& tunnel_exit)
  {
    Area_Square inflated_zone;
    Point2D point_of_robot{msg->cmds[0].position.x, msg->cmds[0].position.y};
    inflated_zone.top_left.x = std::min(tunnel_entry.x, tunnel_exit.x) - 0.9;  // 扩大一定的安全距离
    inflated_zone.top_left.y = std::max(tunnel_entry.y, tunnel_exit.y) + 1.2;
    inflated_zone.bottom_right.x = std::max(tunnel_entry.x, tunnel_exit.x) + 1.3;
    inflated_zone.bottom_right.y = std::min(tunnel_entry.y, tunnel_exit.y) - 0.66;
    bool flag1 = isTroughZone(msg, inflated_zone);
    bool flag2 = isTroughZone(msg, Area_Square{tunnel_entry, tunnel_exit});
    bool flag3 = inflated_zone.contains(point_of_robot);
    if (flag1)
    {
      // std::cout << "MPC trajectory is close to tunnel zone." << std::endl;
      if (flag2)
      {
        // std::cout << "MPC trajectory is through the tunnel." << std::endl;
        return true;
      }
      else if (flag3)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    return false;
  }
    void ros_interface::publishCmdVel(double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub->publish(cmd_vel);
  }
  void ros_interface::publishCmdVel(const geometry_msgs::msg::Twist& cmd_vel)
  {
    cmd_vel_pub->publish(cmd_vel);
  }

}  // namespace Sentry_BT