#pragma once

// ROS2 Normal Libraries
#include <rclcpp/rclcpp.hpp>

// ROS2 Message Libraries
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <memory>
#include <mutex>

// Custom Messages
#include "ros_interfaces/msg/behavior.hpp"
#include "ros_interfaces/msg/game_info.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"
#include "ros_interfaces/msg/radar_info.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "ros_interfaces/msg/sentry_info_online.hpp"
#include "ros_interfaces/msg/team_information.hpp"

// Project Headers
#include "bt_manager/area_visualizer.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/param_manager.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include "bt_manager/utils/tf_utils.hpp"
namespace Sentry_BT {
class ros_interface : public rclcpp::Node
{
private:
  rclcpp::Subscription<ros_interfaces::msg::TeamInformation>::SharedPtr team_info_sub;
  rclcpp::Subscription<ros_interfaces::msg::GameInfo>::SharedPtr game_info_sub;
  rclcpp::Subscription<ros_interfaces::msg::RadarInfo>::SharedPtr radar_info_sub;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOffline>::SharedPtr sentry_offline_sub;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOnline>::SharedPtr sentry_online_sub;

  rclcpp::Publisher<ros_interfaces::msg::Behavior>::SharedPtr behavior_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<ros_interfaces::msg::MpcPositionCommand>::SharedPtr mpc_cmd_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  std::unique_ptr<AreaVisualizer> area_visualizer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr area_marker_timer_;
  mutable std::mutex mpc_msg_mutex_;
  ros_interfaces::msg::MpcPositionCommand::SharedPtr last_mpc_msg_;
  bool has_last_mpc_msg_{false};
  rclcpp::TimerBase::SharedPtr tunnel_check_timer_;
  std::shared_ptr<Blackboard> blackboard_;
  std::shared_ptr<ParamManager> param_manager_;
  bool area_markers_published_{false};

  geometry_msgs::msg::Pose current_pose_;
  mutable std::mutex current_pose_mutex_;
  bool tunnel_detect_latched_{false};

  // 回调函数声明
  void teamInfoCallback(const ros_interfaces::msg::TeamInformation::SharedPtr msg);
  void gameInfoCallback(const ros_interfaces::msg::GameInfo::SharedPtr msg);
  void radarInfoCallback(const ros_interfaces::msg::RadarInfo::SharedPtr msg);
  void sentryOfflineCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg);
  void sentryOnlineCallback(const ros_interfaces::msg::SentryInfoOnline::SharedPtr msg);

public:
  ros_interface(std::shared_ptr<Blackboard> & blackboard_ptr);
  ~ros_interface() override = default;

  std::shared_ptr<ParamManager> getParamManager() const { return param_manager_; }

  bool isTroughZone(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Area_Square & zone);
  bool isTroughTunnel(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg,
    const std::array<Area_Square, 4> & tunnel_areas);
};
}  // namespace Sentry_BT
