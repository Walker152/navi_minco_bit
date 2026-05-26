#pragma once
// #define COMMUNICATION_DEBUG
#include "header.hpp"
#include "thread"
#include <cstdlib>
#include <deque>
#include <limits>

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
      msg.allies[i].robot_id = in.ally_status[i].robot_id;
      msg.allies[i].robot_hp = in.ally_status[i].robot_hp;
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
    msg.manual_point_x = in.manual_point_x;
    msg.manual_point_y = in.manual_point_y;
    msg.manual_key = in.manual_key;
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
    msg.energy_ratio = in.energy_ratio;
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
    msg.yaw_camerainit_to_gimbal = in.yaw_camerainit_to_gimbal;
    msg.lifter_current_pos = in.lifter_current_pos;
    msg.is_transformable = in.is_transformable;
    msg.transform_state = in.transform_state;
    transform_state = in.transform_state;
    const auto stamp = now();
    msg.header.stamp = stamp;
    msg.capacitor_capacity = in.capacitor_capacity;
    offline_info_pub_->publish(msg);

    {
      std::lock_guard<std::mutex> lk(imu_mutex_);
      if (chassis_imu_history_.size() >= kImuHistoryCapacity) {
        chassis_imu_history_.pop_front();
      }
      chassis_imu_history_.push_back(ImuYawSample{stamp, in.chassis_imu_yaw});
    }
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
    odom_.pose.pose.orientation.w = 1.0;
    send_enable_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);

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
    cmd_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cmd_force_mpc",
      1,
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
        sendCmdWrenchCB(msg);
      },
      sub_opt);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init",
      1,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCB(msg);
      },
      sub_opt);
    // astar_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    //   "/astar_path_vis",
    //   rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    //   [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
    //     astarPathCB(msg);
    //   },
    //   sub_opt);
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
    imu_yaw_window_ms_ = this->declare_parameter<int64_t>("communication.imu_yaw_window_ms", 20);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    com_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&ComInterfaceRos::communicationLoop, this), comm_cb_group_);
    path_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&ComInterfaceRos::sendGlobalPathLoop, this),
      comm_cb_group_);
    RCLCPP_INFO(this->get_logger(), "ComInterfaceRos initialized");
  }

  void communicationLoop()
  {
    if (std::chrono::steady_clock::now() < send_enable_time_) {
      return;
    }

    // Chassis control variables
    float vx_mps = 0.0f;
    float vy_mps = 0.0f;
    float vw_rpm = 0.0f;
    float current_vx = 0.0f;
    float current_vy = 0.0f;
    float current_vw = 0.0f;
    float fx_global = 0.0f;
    float fy_global = 0.0f;
    float fw_global = 0.0f;
    float delta_yaw = 0.0f;

    // Behavior-related variables
    uint8_t pitch_mode = 0;
    float scan_yaw_min_deg_ = 0.0f;
    float scan_yaw_max_deg_ = 0.0f;
    uint8_t desire_stance = 0;
    uint8_t desire_lifter_pos = 0;
    uint8_t comfirm_revive = 0;
    uint16_t ammo_purchase_request = 0;
    uint8_t ammo_req = 0;
    uint8_t revive_req = 0;
    uint8_t health_req = 0;
    bool use_limited_scan = false;
    bool not_aim_enemy = true;

    bool use_gyro_mode = false;
    float gyro_vel = 0.0f;
    geometry_msgs::msg::Quaternion odom_q;
    {
      // Snapshot shared state to avoid data races.
      std::lock_guard<std::mutex> lk(state_mutex_);
      vx_mps = static_cast<float>(cmd_vel_.linear.x);
      // vx_mps = 3.0f;
      vy_mps = static_cast<float>(cmd_vel_.linear.y);
      // vy_mps = 0.0f;

      vw_rpm = static_cast<float>(cmd_vel_.angular.z * 60.0 / (2.0 * M_PI));

      current_vx = odom_.twist.twist.linear.x;
      current_vy = odom_.twist.twist.linear.y;
      current_vw = odom_.twist.twist.angular.z;
      odom_q = odom_.pose.pose.orientation;
      fx_global = cmd_wrench_.force.x;
      fy_global = cmd_wrench_.force.y;
      fw_global = cmd_wrench_.torque.z;
      delta_yaw = delta_yaw_;

      pitch_mode = behavior_.pitch_mode;
      desire_stance = behavior_.desired_stance;
      desire_lifter_pos = behavior_.desire_lifter_pos;
      use_gyro_mode = behavior_.use_gyro_mode;
      gyro_vel = behavior_.gyro_vel;
      scan_yaw_min_deg_ = behavior_.scan_yaw_min;
      scan_yaw_max_deg_ = behavior_.scan_yaw_max;
      ammo_purchase_request = behavior_.ammo_purchase_request;
      comfirm_revive = behavior_.revive_request;
      ammo_req = behavior_.remote_ammo_request;
      revive_req = behavior_.remote_revive_request;
      health_req = behavior_.remote_health_request;
      use_limited_scan = behavior_.use_limited_scan;
      not_aim_enemy = behavior_.not_aim_enemy;
      if (use_gyro_mode) {
        vw_rpm = gyro_vel;
      }

      // vw_rpm = -80.0f;
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
      fx_global,
      fy_global,
      fw_global,
      1,
      delta_yaw);

    BehaviorData behavior_data(pitch_mode,
      desire_stance,
      desire_lifter_pos,
      scan_yaw_min_deg_,
      scan_yaw_max_deg_,
      ammo_purchase_request,
      comfirm_revive,
      revive_req,
      ammo_req,
      health_req,
      0,
      not_aim_enemy);
    auto flag = Communication::send2stm32<ChassisTarget>(target, ENUM_PACKET_NAV_DATA);
#ifdef COMMUNICATION_DEBUG
    if (flag == 0) {
      static auto last_send_time = this->now();
      auto now_time = this->now();
      if ((now_time - last_send_time).seconds() >= 1.0) {
        LOG_DEBUG_BLOCK(std::string(CYAN) + "[COM][ChassisCmd] ",
          NV(target.vx_mps),
          NV(target.vy_mps),
          NV(target.vw_rpm),
          NV(target.current_yaw),
          NV(target.current_vx),
          NV(target.current_vy),
          NV(target.current_vw),
          NV(target.fx_global),
          NV(target.fy_global),
          NV(target.fw_global));
        last_send_time = now_time;
      }
    }
#endif
    std::this_thread::sleep_for(
      std::chrono::milliseconds(2));  // Avoid sending two packets in the same millisecond, which can cause
                                      // issues for STM32's UART DMA parsing.
    auto flag2 = Communication::send2stm32<BehaviorData>(behavior_data, ENUM_PACKET_BEHAVIOR_DATA);
#ifdef COMMUNICATION_DEBUG
    if (flag2 == 0) {
      static auto last_send_time = this->now();
      auto now_time = this->now();
      if ((now_time - last_send_time).seconds() >= 1.0) {
        LOG_DEBUG_BLOCK(std::string(YELLOW) + "[COM][BehaviorData] ",
          NV(behavior_data.pitch_mode),
          NV(static_cast<int>(behavior_data.desire_stance)),
          NV(static_cast<int>(behavior_data.desire_lifter_pos)),
          NV(behavior_data.scan_yaw_min_deg),
          NV(behavior_data.scan_yaw_max_deg),
          NV(behavior_data.ammo_purchase_request),
          NV(behavior_data.revive_request),
          NV(behavior_data.remote_revive_request),
          NV(behavior_data.remote_ammo_request),
          NV(behavior_data.remote_health_request),
          NV(behavior_data.use_limited_scan));
        last_send_time = now_time;
      }
    }
#endif
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

  bool tryBuildGlobalPathPacket(const nav_msgs::msg::Path & path, GlobalPath & out)
  {
    out = GlobalPath{};
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
      return true;
    }

    if (!tf_buffer_->canTransform(minimap_frame_, map_frame_, tf2::TimePointZero)) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for TF %s -> %s before packing global path",
        map_frame_.c_str(),
        minimap_frame_.c_str());
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(minimap_frame_, map_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for TF %s -> %s before packing global path: %s",
        map_frame_.c_str(),
        minimap_frame_.c_str(),
        ex.what());
      return false;
    }

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
    return true;
  }

  void astarPathCB(const nav_msgs::msg::Path::ConstSharedPtr & pathPtr)
  {
    GlobalPath packet{};
    if (!tryBuildGlobalPathPacket(*pathPtr, packet)) {
      return;
    }
    std::lock_guard<std::mutex> lk(path_mutex_);
    pending_global_path_ = packet;
  }

  void sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr & velPtr)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    cmd_vel_ = *velPtr;
  }

  void sendCmdWrenchCB(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr & wrenchPtr)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    cmd_wrench_ = wrenchPtr->wrench;
  }

  void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr & odomPtr)
  {
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      odom_ = *odomPtr;
    }
    updateDeltaYaw(odomPtr->header.stamp, odomPtr->pose.pose.orientation);
  }

  void updateDeltaYaw(
    const builtin_interfaces::msg::Time & stamp_msg, const geometry_msgs::msg::Quaternion & orientation)
  {
    const rclcpp::Time stamp(stamp_msg);
    const int64_t window_ns = imu_yaw_window_ms_ * 1000000L;
    float matched_imu_yaw = 0.0f;
    bool found = false;

    {
      std::lock_guard<std::mutex> lk(imu_mutex_);
      int64_t best_abs_ns = std::numeric_limits<int64_t>::max();
      for (const auto & sample : chassis_imu_history_) {
        const int64_t dt_ns = (sample.stamp - stamp).nanoseconds();
        const int64_t abs_ns = std::llabs(dt_ns);
        if (abs_ns <= window_ns && abs_ns < best_abs_ns) {
          best_abs_ns = abs_ns;
          matched_imu_yaw = sample.yaw;
          found = true;
        }
      }
    }

    float delta = 0.0f;
    if (found) {
      tf2::Quaternion q;
      tf2::fromMsg(orientation, q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      const float odom_yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
      delta = odom_yaw_deg - matched_imu_yaw;
      while (delta > 180.0f) {
        delta -= 360.0f;
      }
      while (delta < -180.0f) {
        delta += 360.0f;
      }
    }

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      delta_yaw_ = delta;
      // std::cout << "Delta yaw updated: " << delta_yaw_ << " degrees (matched_imu_yaw=" << matched_imu_yaw
      //           << ", found=" << found << ")" << std::endl;
    }
  }

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_wrench_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr astar_path_sub_;
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
  std::mutex imu_mutex_;

  struct ImuYawSample
  {
    rclcpp::Time stamp;
    float yaw;
  };
  static constexpr size_t kImuHistoryCapacity = 200;
  std::deque<ImuYawSample> chassis_imu_history_;
  int64_t imu_yaw_window_ms_{20};

  // Latest global path packet cache
  GlobalPath pending_global_path_{};

  // TF query for map -> minimap transform
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_{"map"};
  std::string minimap_frame_{"minimap"};

  // State
  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Wrench cmd_wrench_;
  nav_msgs::msg::Odometry odom_;
  std::chrono::steady_clock::time_point send_enable_time_{std::chrono::steady_clock::now()};
  float transform_state = 0.0f;
  float delta_yaw_{0.0f};
};

}  // namespace ns_com
