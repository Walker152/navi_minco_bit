#pragma once

#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <std_msgs/msg/int32.hpp>

namespace Sentry_BT {
class SetCoordinate : public BT::SyncActionNode
{
public:
  SetCoordinate(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetTargetCoordinate : public BT::SyncActionNode
{
public:
  SetTargetCoordinate(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetManualOverrideGoal : public BT::SyncActionNode
{
public:
  SetManualOverrideGoal(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SelectPatrolPoint : public BT::SyncActionNode
{
public:
  SelectPatrolPoint(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int wait_time_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

// 速度控制节点
class DirectVelocityControl : public BT::StatefulActionNode
{
public:
  DirectVelocityControl(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double linear_y_;
  double angular_z_;
  double timeout_;
  rclcpp::Time start_time_;
  rclcpp::Time last_pub_time_;
  Area_Square internal_area_{ Point2D(8.5, 3.0), Point2D(10.0, 0.0) }; 
};

//设定固定位置节点
class SetStairsPosition : public BT::SyncActionNode
{
public:
  SetStairsPosition(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class DescendStairsAction : public BT::StatefulActionNode
{
public:
  DescendStairsAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class AccumulateAmmoPurchase : public BT::SyncActionNode
{
public:
  AccumulateAmmoPurchase(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class ChangeMapAction : public BT::SyncActionNode
{
public:
  ChangeMapAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// 控制过隧道
class ControlThroughTunnel : public BT::StatefulActionNode
{
public:
  ControlThroughTunnel(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool is_through_tunnel_;
  bool initial_small_gyoro_;
  bool initial_yaw_opt_;
  const std::vector<std::string> required_params_ = {
    "FollowPath.use_small_gyro_mode", "FollowPath.enable_yaw_opt"};
  double current_yaw_;
  const double target_yaw_ = -1.57;  // 目标偏航角
  bool lifter_ready_ = false;
  bool yaw_ready_ = false;
  // rclcpp::Time last_pub_time_;
  bool wait_param_service();
  bool wait_param_available();
  bool has_params(std::vector<std::string> param_names);
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};

//等待台阶下可能的队友
class WaitForNoAlliesInStairsArea : public BT::StatefulActionNode
{
public:
  WaitForNoAlliesInStairsArea(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  Area_Square stairs_bottom_area_; // 台阶下区域
  std::shared_ptr<ros_interface> ros_iface_;
};
}  // namespace Sentry_BT