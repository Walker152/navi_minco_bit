#pragma once

#include "header.hpp"

#include "ros_interfaces/msg/ally_robot_status.hpp"
#include "ros_interfaces/msg/behavior.hpp"
#include "ros_interfaces/msg/enemy_robot_status.hpp"
#include "ros_interfaces/msg/game_info.hpp"
#include "ros_interfaces/msg/radar_info.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "ros_interfaces/msg/sentry_info_online.hpp"
#include "ros_interfaces/msg/team_information.hpp"

#include "com.hpp"
#include "utils/custom_protocol.hpp"
#include "utils/protocol.hpp"

namespace ns_com {

class ComInterfaceRos : public rclcpp::Node
{
public:
  using Ptr = std::shared_ptr<ComInterfaceRos>;
  explicit ComInterfaceRos(const std::string & name) : rclcpp::Node(name) { initRos(); }

  void bindCommunication()
  {
    Communication::setRosInterface(std::static_pointer_cast<ComInterfaceRos>(this->shared_from_this()));
    Communication::init();
  }

  void publishTeamInfo(const TeamInfo & in)
  {
    if (!team_info_pub_)
      return;
    ros_interfaces::msg::TeamInformation msg;
    for (int i = 0; i < 4; i++) {
      msg.allies[i].armor_id = in.ally_status[i].robot_id;
      msg.allies[i].remain_hp = in.ally_status[i].robot_hp;
      msg.allies[i].position.position.x = static_cast<double>(in.ally_status[i].robot_pos_x);
      msg.allies[i].position.position.y = static_cast<double>(in.ally_status[i].robot_pos_y);
    }
    msg.base_hp = in.base_hp;
    msg.outpost_hp = in.outpost_hp;
    msg.header.stamp = now();
    team_info_pub_->publish(msg);
  }

  void publishGameInfo(const GameInfo & in)
  {
    if (!game_info_pub_)
      return;
    ros_interfaces::msg::GameInfo msg;
    msg.game_time_remaining = in.game_time_remaining;
    msg.coin_remaining = in.coin_remaining;
    msg.event_code = in.event_code;
    msg.game_status = in.game_status;
    msg.header.stamp = now();
    game_info_pub_->publish(msg);
  }

  void publishSentryInfoOnline(const SentryInfoOnline & in)
  {
    if (!online_info_pub_)
      return;
    ros_interfaces::msg::SentryInfoOnline msg;
    msg.self_health = in.self_health;
    msg.bullets_remaining = in.bullets_remaining;
    msg.cooling_value = in.cooling_value;
    msg.heat_limit = in.heat_limit;
    msg.current_heat = in.current_heat;
    msg.sentry_pos.x = static_cast<double>(in.sentry_pos_x);
    msg.sentry_pos.y = static_cast<double>(in.sentry_pos_y);
    msg.speed_monitor_angle = in.speed_monitor_angle;
    msg.sentry_info_1 = in.sentry_info_1;
    msg.sentry_info_2 = in.sentry_info_2;
    msg.header.stamp = now();
    online_info_pub_->publish(msg);
  }

  void publishSentryInfoOffline(const SentryInfoOffline & in)
  {
    if (!offline_info_pub_)
      return;
    ros_interfaces::msg::SentryInfoOffline msg;
    msg.is_get = in.is_get;
    msg.armor_pos.x = static_cast<double>(in.armor_pos[0]);
    msg.armor_pos.y = static_cast<double>(in.armor_pos[1]);
    msg.armor_pos.z = static_cast<double>(in.armor_pos[2]);
    msg.armor_num = in.armor_num;
    msg.yaw_imu = in.yaw_imu;
    msg.lifter_current_pos = in.lifter_current_pos;
    msg.is_transformable = in.is_transformable;
    msg.transform_state = in.transform_state;
    transform_state = in.transform_state;
    msg.header.stamp = now();
    msg.capacitor_capacity = in.capacitor_capacity;
    offline_info_pub_->publish(msg);
  }

  void publishRadarInfo(const RadarInfo & in)
  {
    if (!radar_info_pub_)
      return;
    ros_interfaces::msg::RadarInfo msg;
    for (int i = 0; i < 6; ++i) {
      msg.enemies[i].robot_id = in.enemy_status[i].robot_id;
      msg.enemies[i].robot_hp = in.enemy_status[i].robot_hp;
      msg.enemies[i].allowed_projectile = in.enemy_status[i].allowed_projectile;
      msg.enemies[i].position.position.x =
        static_cast<double>(in.enemy_status[i].robot_pos_x) / 100.0;  // 协议里是cm，转换成m
      msg.enemies[i].position.position.y =
        static_cast<double>(in.enemy_status[i].robot_pos_y) / 100.0;  // 协议里是cm，转换成m
    }
    msg.enemy_coin_left = in.enemy_coin_left;
    msg.enemy_coin_accumulated = in.enemy_coin_accumulated;
    msg.is_enemy_outpost_sensed = in.is_enemy_outpost_sensed;
    msg.header.stamp = now();
    radar_info_pub_->publish(msg);
  }

private:
  ros_interfaces::msg::Behavior behavior_;
  void initRos()
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;

    comm_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = sub_cb_group_;

    chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_mpc",
      1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        sendChassisCtrlCB(msg);
      },
      sub_opt);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init",
      1,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCB(msg);
      },
      sub_opt);
    astar_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/astar_path_vis",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        astarPathCB(msg);
      },
      sub_opt);
    behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
      "/sentry/behaivor_send",
      1,
      [this](ros_interfaces::msg::Behavior::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        behavior_ = *msg;
      },
      sub_opt);
    game_info_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
    offline_info_pub_ =
      create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
    online_info_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
    team_info_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
    radar_info_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);

    map_frame_ = this->declare_parameter<std::string>("global_path.map_frame", "map");
    minimap_frame_ = this->declare_parameter<std::string>("global_path.minimap_frame", "minimap");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    com_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(3), std::bind(&ComInterfaceRos::communicationLoop, this), comm_cb_group_);
    path_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&ComInterfaceRos::sendGlobalPathLoop, this),
      comm_cb_group_);
    RCLCPP_INFO(this->get_logger(), "ComInterfaceRos initialized");
  }

  void communicationLoop()
  {
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float vw_rpm = 0.0f;
    float current_vx = 0.0f;
    float current_vy = 0.0f;
    float current_vw = 0.0f;
    float tunnel_speed_x = 0.0f;
    float tunnel_speed_y = 0.0f;
    uint8_t desire_stance = 0;
    uint8_t desire_lifter_pos = 0;
    bool use_gyro_mode = false;
    bool through_tunnel = false;
    bool current_in_tunnel = false;

    float gyro_vel = 0.0f;
    bool is_aim_outpost = false;
    geometry_msgs::msg::Quaternion odom_q;

    {
      // Snapshot shared state to avoid data races.
      std::lock_guard<std::mutex> lk(state_mutex_);
      vx_mps = cmd_vel_.linear.x;
      vy_mps = cmd_vel_.linear.y;
      vw_rpm = static_cast<float>(cmd_vel_.angular.z * 60.0 / (2.0 * M_PI));
      // vw_rpm = 80.0f;
      desire_stance = behavior_.desired_stance;
      desire_lifter_pos = behavior_.desire_lifter_pos;
      use_gyro_mode = behavior_.use_gyro_mode;
      gyro_vel = behavior_.gyro_vel;
      tunnel_speed_x = behavior_.tunnel_speed_x;
      tunnel_speed_y = behavior_.tunnel_speed_y;
      through_tunnel = behavior_.through_tunnel;
      current_in_tunnel = behavior_.current_in_tunnel;

      // if (use_gyro_mode) {
        vw_rpm = gyro_vel;
        // if(current_in_tunnel) {
        //   vx_mps = tunnel_speed_x;
        //   vy_mps = tunnel_speed_y;
        // }
      // }
      // if (transform_state >= 0.85f) {
      //   vx_mps *= 1.0;
      //   vy_mps *= 1.0;
      //   vw_rpm = 0.0;
      // }
      odom_q = odom_.pose.pose.orientation;
      current_vx = odom_.twist.twist.linear.x;
      current_vy = odom_.twist.twist.linear.y;
      current_vw = odom_.twist.twist.angular.z;
      // std::cout << "vw_rpm: " << vw_rpm << ", use_gyro_mode: " << use_gyro_mode << ", gyro_vel: " << gyro_vel << std::endl;
      // vx_mps = 2.5;
      // vy_mps = 0.0;
      // vw_rpm = 0.0f;
    }

    tf2::Quaternion q;
    tf2::fromMsg(odom_q, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const float current_yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
    auto now = this->now();
    ChassisTarget target(vx_mps,
      vy_mps,
      vw_rpm,
      current_yaw_deg,
      current_vx,
      current_vy,
      current_vw,
      is_aim_outpost,
      desire_stance,
      desire_lifter_pos);
    auto flag = Communication::send2stm32<ChassisTarget>(target, ENUM_PACKET_NAV_DATA);
    if (flag == 0) {
      static auto last_send_time = this->now();
      auto now_time = this->now();
      if ((now_time - last_send_time).seconds() >= 1.0) {
        LOG_DEBUG_BLOCK(std::string(CYAN) + "[COM][ChassisCmd] ",
          NV(target.vx_mps),
          NV(target.vy_mps),
          NV(target.vw_rpm),
          NV(target.current_yaw),
          NV(target.is_aim_outpost),
          NV(static_cast<int>(target.desire_stance)),
          NV(static_cast<int>(target.desire_lifter_pos)));
        last_send_time = now_time;
      }
    }
  }

  void sendGlobalPathLoop()
  {
    GlobalPath global_path;
    {
      std::lock_guard<std::mutex> lk(path_mutex_);
      global_path = pending_global_path_;
    }

    GlobalPathX global_path_x{};
    GlobalPathY global_path_y{};
    global_path_x.start_x = global_path.start_x;
    global_path_y.start_y = global_path.start_y;
    for (size_t i = 0; i < 49; ++i) {
      global_path_x.delta_x[i] = global_path.delta_x[i];
      global_path_y.delta_y[i] = global_path.delta_y[i];
    }

    // (void)Communication::send2stm32<GlobalPathX>(global_path_x, ENUM_PACKET_GLOBAL_PATH_X);
    // (void)Communication::send2stm32<GlobalPathY>(global_path_y, ENUM_PACKET_GLOBAL_PATH_Y);
  }

  static void mapToMinimapPoint(const tf2::Transform & tf_map_to_minimap,
    const double map_x,
    const double map_y,
    double & mini_x,
    double & mini_y)
  {
    const tf2::Vector3 p_map(map_x, map_y, 0.0);
    const tf2::Vector3 p_minimap = tf_map_to_minimap * p_map;
    mini_x = p_minimap.x();
    mini_y = p_minimap.y();
  }

  GlobalPath buildGlobalPathPacket(const nav_msgs::msg::Path & path)
  {
    GlobalPath out{};
    constexpr size_t kSampleNum = 49;
    constexpr double kCoordScaleDm = 10.0;  // convert meter-based coords to decimeter for protocol

    std::array<double, kSampleNum> sample_x{};
    std::array<double, kSampleNum> sample_y{};

    const size_t n = path.poses.size();
    size_t valid_num = 0;
    if (n == 0) {
      valid_num = 0;
    } else if (n > kSampleNum) {
      valid_num = kSampleNum;
      // Uniformly sample along full polyline arc length.
      std::vector<double> s(n, 0.0);
      for (size_t i = 1; i < n; ++i) {
        const double x0 = path.poses[i - 1].pose.position.x;
        const double y0 = path.poses[i - 1].pose.position.y;
        const double x1 = path.poses[i].pose.position.x;
        const double y1 = path.poses[i].pose.position.y;
        s[i] = s[i - 1] + std::hypot(x1 - x0, y1 - y0);
      }

      const double total_len = s.back();
      if (total_len <= 1e-6) {
        const double x = path.poses.front().pose.position.x;
        const double y = path.poses.front().pose.position.y;
        sample_x.fill(x);
        sample_y.fill(y);
      } else {
        for (size_t k = 0; k < kSampleNum; ++k) {
          const double target = total_len * static_cast<double>(k) / static_cast<double>(kSampleNum - 1);
          auto it = std::lower_bound(s.begin(), s.end(), target);
          if (it == s.begin()) {
            sample_x[k] = path.poses.front().pose.position.x;
            sample_y[k] = path.poses.front().pose.position.y;
            continue;
          }
          if (it == s.end()) {
            sample_x[k] = path.poses.back().pose.position.x;
            sample_y[k] = path.poses.back().pose.position.y;
            continue;
          }

          const size_t idx = static_cast<size_t>(std::distance(s.begin(), it));
          const double seg_s0 = s[idx - 1];
          const double seg_s1 = s[idx];
          const double seg_len = seg_s1 - seg_s0;
          const double ratio = (seg_len > 1e-9) ? ((target - seg_s0) / seg_len) : 0.0;

          const double x0 = path.poses[idx - 1].pose.position.x;
          const double y0 = path.poses[idx - 1].pose.position.y;
          const double x1 = path.poses[idx].pose.position.x;
          const double y1 = path.poses[idx].pose.position.y;
          sample_x[k] = x0 + (x1 - x0) * ratio;
          sample_y[k] = y0 + (y1 - y0) * ratio;
        }
      }
    } else {
      valid_num = n;
      // Copy all points in order; trailing packet entries remain zero.
      for (size_t k = 0; k < n; ++k) {
        sample_x[k] = path.poses[k].pose.position.x;
        sample_y[k] = path.poses[k].pose.position.y;
      }
    }

    if (valid_num == 0) {
      return out;
    }

    const auto tf_stamped = tf_buffer_->lookupTransform(minimap_frame_, map_frame_, tf2::TimePointZero);
    tf2::Transform tf_map_to_minimap;
    tf2::fromMsg(tf_stamped.transform, tf_map_to_minimap);

    const auto to_uint16 = [](const long v) -> uint16_t {
      return static_cast<uint16_t>(std::clamp(v, 0L, 65535L));
    };
    const auto to_int8 = [](const long v) -> int8_t {
      return static_cast<int8_t>(std::clamp(v, -128L, 127L));
    };

    // Quantize absolute minimap coordinates first, then build deltas from quantized points.
    // This avoids cumulative drift from independently rounded segment deltas.
    std::array<long, kSampleNum> qx_dm{};
    std::array<long, kSampleNum> qy_dm{};
    for (size_t i = 0; i < valid_num; ++i) {
      double mini_x = 0.0;
      double mini_y = 0.0;
      mapToMinimapPoint(tf_map_to_minimap, sample_x[i], sample_y[i], mini_x, mini_y);
      qx_dm[i] = static_cast<long>(std::lround(mini_x * kCoordScaleDm));
      qy_dm[i] = static_cast<long>(std::lround(mini_y * kCoordScaleDm));
    }

    out.start_x = to_uint16(qx_dm[0]);
    out.start_y = to_uint16(qy_dm[0]);
    out.delta_x[0] = 0;
    out.delta_y[0] = 0;
    for (size_t i = 1; i < valid_num; ++i) {
      const long dx = qx_dm[i] - qx_dm[i - 1];
      const long dy = qy_dm[i] - qy_dm[i - 1];
      out.delta_x[i] = to_int8(dx);
      out.delta_y[i] = to_int8(dy);
    }
    for (size_t i = valid_num; i < kSampleNum; ++i) {
      out.delta_x[i] = 0;
      out.delta_y[i] = 0;
    }
    return out;
  }

  void astarPathCB(const nav_msgs::msg::Path::ConstSharedPtr & pathPtr)
  {
    GlobalPath packet = buildGlobalPathPacket(*pathPtr);
    std::lock_guard<std::mutex> lk(path_mutex_);
    pending_global_path_ = packet;
  }

  void sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr & velPtr)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    cmd_vel_ = *velPtr;
  }

  void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr & odomPtr)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    odom_ = *odomPtr;
  }

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr astar_path_sub_;
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;

  // Publishers
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_info_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_info_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_info_pub_;
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_info_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_info_pub_;

  // Communication Timer
  rclcpp::TimerBase::SharedPtr com_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  // Callback groups (enable concurrency with MultiThreadedExecutor)
  rclcpp::CallbackGroup::SharedPtr comm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  // Shared state mutex
  std::mutex state_mutex_;
  std::mutex path_mutex_;

  // Latest global path packet cache
  GlobalPath pending_global_path_{};

  // TF query for map -> minimap transform
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_{"map"};
  std::string minimap_frame_{"minimap"};

  // State
  geometry_msgs::msg::Twist cmd_vel_;
  nav_msgs::msg::Odometry odom_;
  float transform_state = 0.0f;
};

}  // namespace ns_com
