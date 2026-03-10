#pragma once

#include <cstdint>
#include <chrono>
#include <memory>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros_interfaces/msg/event_status.hpp"
#include "ros_interfaces/msg/nav.hpp"
#include "ros_interfaces/msg/behavior.hpp"

#include "com.hpp"
#include "utils/custom_protocol.hpp"
#include "utils/protocol.hpp"

namespace ns_com
{

  class ComInterfaceRos : public rclcpp::Node
  {
  public:
    using Ptr = std::shared_ptr<ComInterfaceRos>;

    explicit ComInterfaceRos(const std::string& name)
      : rclcpp::Node(name)
    {
      initRos();
    }

    void bindCommunication()
    {
      Communication::setRosInterface(std::static_pointer_cast<ComInterfaceRos>(this->shared_from_this()));
      Communication::init();
    }

    void publishNav(const NavRes& in)
    {
      if(!nav_pub_)
        return;
      ros_interfaces::msg::Nav msg;
      msg.target_x = in.x;
      msg.target_y = in.y;
      msg.nav_mode = ros_interfaces::msg::Nav::MODE_SINGLE_POINT;
      msg.header.stamp = now();
      nav_pub_->publish(msg);
    }

    void publishEventStatus(const EventStatus& in)
    {
      if(!event_status_pub_)
        return;
      ros_interfaces::msg::EventStatus msg;
      msg.self_health = in.self_health;
      msg.num_shoot = in.num_shoot;
      msg.own_outpost_health = in.own_outpost_health;
      msg.buff_active = in.buff_active;
      msg.enemy_outpost_destroyed = in.enemy_outpost_destroyed;
      msg.enemy_detected.is_detect = in.is_get;
      msg.enemy_detected.position.x = in.x;
      msg.enemy_detected.position.y = in.y;
      msg.enemy_detected.position.z = in.z;
      msg.enemy_detected.armor_id = in.armor_id;
      msg.current_stance = in.current_stance;
      msg.game_status = in.game_status;
      msg.gimbal_yaw = in.gimbal_yaw;
      msg.lifter_pos_now = in.lifter_pos_now;
      msg.header.stamp = now();
      event_status_pub_->publish(msg);
    }

  private:
    std_msgs::msg::Bool outpost_msg_;
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
          "/cmd_vel_nav", 1,
          [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { sendChassisCtrlCB(msg); },
          sub_opt);
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/aft_mapped_to_init", 1,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCB(msg); },
          sub_opt);
      outpost_msg_sub_ = create_subscription<std_msgs::msg::Bool>(
          "/sentry/outpost_status", 1,
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            outpost_msg_ = *msg;
          },
          sub_opt);
      behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
          "/sentry/behaivor_send", 1,
          [this](ros_interfaces::msg::Behavior::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            behavior_ = *msg;
          },
          sub_opt);
      nav_pub_ = create_publisher<ros_interfaces::msg::Nav>("/NavRequest", 10);
      event_status_pub_ = create_publisher<ros_interfaces::msg::EventStatus>("/sentry/event_status", 10);
      
      com_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&ComInterfaceRos::communicationLoop, this),
        comm_cb_group_);
      RCLCPP_INFO(this->get_logger(), "ComInterfaceRos initialized");
    }

    void communicationLoop()
    {
      float vx_mps = 0.0f;
      float vy_mps = 0.0f;
      float vw_rpm = 0.0f;
      float gimbal_yaw = 0.0f;
      uint8_t desire_stance = 0;
      uint8_t desire_lifter_pos = 0;
      bool outpost_msg = false;
      double odom_x = 0.0;
      double odom_y = 0.0;
      geometry_msgs::msg::Quaternion odom_q;

      {
        // Snapshot shared state to avoid data races.
        std::lock_guard<std::mutex> lk(state_mutex_);
        vx_mps = cmd_vel_.linear.x;
        vy_mps = cmd_vel_.linear.y;
        vw_rpm = static_cast<float>(cmd_vel_.angular.z * 60.0 / (2.0 * M_PI));
        // vw_rpm = 40.0f;
        // desire_stance = ros_interfaces::msg::Behavior::STANCE_MOVE; // TODO: 这里暂时写死，后续根据实际情况修改
        // desire_lifter_pos = LifterPos::BOTTOM; // TODO: 这里暂时写死，后续根据实际情况修改
        desire_stance = behavior_.desired_stance; // 之前写死了，现作修改
        desire_lifter_pos = behavior_.desire_lifter_pos; // 之前写死了，现作修改
        outpost_msg = outpost_msg_.data;
        odom_x = odom_.pose.pose.position.x;
        odom_y = odom_.pose.pose.position.y;
        odom_q = odom_.pose.pose.orientation;
        
      }
      
      uint8_t _is_use_mid360 = 0;
      if(odom_x >  2.5)
      {
        // vw_rpm = 30.0;
      }
      tf2::Quaternion q;
      tf2::fromMsg(odom_q, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      const float current_yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
      auto now = this->now();
      // std::cout << "vx: " << vx_mps << " m/s, vy: " << vy_mps << " m/s, vw: " << vw_rpm << " rpm,"
      //           << "current_yaw:" << current_yaw_deg << " deg " << "time:" << now.seconds() << "s" << std::endl;
      ChassisTarget target(vx_mps,
                           vy_mps,
                           vw_rpm,
                           odom_x,
                           odom_y,
                           current_yaw_deg,
                           outpost_msg,
                           desire_stance,
                           desire_lifter_pos);
      auto flag = Communication::send2stm32<ChassisTarget>(target);
      if(flag == 0)
      {
        static auto last_send_time = this->now();
        auto now_time = this->now();
        if((now_time - last_send_time).seconds() >= 1.0)
        {
          LOG_DEBUG_BLOCK(std::string(CYAN) + "[COM][ChassisCmd] ",
                          NV(target.vx_mps),
                          NV(target.vy_mps),
                          NV(target.vw_rpm),
                          NV(target.current_x),
                          NV(target.current_y),
                          NV(target.current_yaw),
                          NV(target.is_aim_outpost),
                          NV(static_cast<int>(target.desire_stance)),
                          NV(static_cast<int>(target.desire_lifter_pos)));
          last_send_time = now_time;
        }
      }
    }
    void sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr& velPtr)
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      cmd_vel_ = *velPtr;
    }

    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr& odomPtr)
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      odom_ = *odomPtr;
    }

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr outpost_msg_sub_;
    rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;

    // Publishers
    rclcpp::Publisher<ros_interfaces::msg::Nav>::SharedPtr nav_pub_;
    rclcpp::Publisher<ros_interfaces::msg::EventStatus>::SharedPtr event_status_pub_;

    // Communication Timer
    rclcpp::TimerBase::SharedPtr com_timer_;

    // Callback groups (enable concurrency with MultiThreadedExecutor)
    rclcpp::CallbackGroup::SharedPtr comm_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

    // Shared state mutex
    std::mutex state_mutex_;

    // State
    geometry_msgs::msg::Twist cmd_vel_;
    nav_msgs::msg::Odometry odom_;
  };

}  // namespace ns_com
