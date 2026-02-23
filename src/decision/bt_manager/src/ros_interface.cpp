#include "bt_manager/ros_interface.hpp"

#include <chrono>
#include <string>

namespace Sentry_BT
{
  ros_interface::ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr)
    : Node("ros_interface_" +
               std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 10000),
           rclcpp::NodeOptions().use_global_arguments(false))
    , blackboard_(blackboard_ptr)
  {
    // 订阅事件状态话题
    event_sub = this->create_subscription<ros_interfaces::msg::EventStatus>(
        "/sentry/event_status",
        1,
        [this](const ros_interfaces::msg::EventStatus::SharedPtr msg) { this->eventCallback(msg); });

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
                                                                  1,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    // 更新当前位置
                                                                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                                                                    current_pose_ = msg->pose.pose;
                                                                  }); 

    // 定时发布前哨站状态
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          auto current_mode = blackboard_->get<int>("current_mode");
          std_msgs::msg::Bool outpost_msg;
          if(current_mode == Sentry_BT::NavMode::RESPONSE &&
             std::hypot(current_pose_.position.x - nav_points[2].x, current_pose_.position.y - nav_points[2].y) < 1.0)
          {
            outpost_msg.data = true;
            blackboard_->set<bool>("outpost_msg", true);
            outpost_pub->publish(outpost_msg);
          }
          else if(std::hypot(current_pose_.position.x - nav_points[2].x, current_pose_.position.y - nav_points[2].y) >=
                  1.0)
          {
            outpost_msg.data = false;
            blackboard_->set<bool>("outpost_msg", false);
            outpost_pub->publish(outpost_msg);
          }
        });
    outpost_pub = this->create_publisher<std_msgs::msg::Bool>("/sentry/outpost_status", 10);
    stance_pub = this->create_publisher<std_msgs::msg::Int32>("/sentry/want_position", 10);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void ros_interface::publishCmdVel(double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub->publish(cmd_vel);
  }

  void ros_interface::publishPosition(int target_stance)
  {
    std_msgs::msg::Int32 stance_msg;
    stance_msg.data = target_stance;
    stance_pub->publish(stance_msg);
  }

  geometry_msgs::msg::Pose ros_interface::getCurrentPose() const
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    return current_pose_;
  }

  void ros_interface::eventCallback(const ros_interfaces::msg::EventStatus::SharedPtr msg)
  {
    // 更新黑板中的数据
    blackboard_->set<float>("health", ((int)msg->self_health / 4));
    blackboard_->set<int>("own_outpost_health", msg->own_outpost_health);
    blackboard_->set<bool>("enemy_outpost_destroyed", msg->enemy_outpost_destroyed);
    blackboard_->set<bool>("bonus_active", msg->buff_active);
    blackboard_->set<bool>("target_valid", msg->enemy_detected.is_detect);
    if(msg->position >= 1 && msg->position <= 3)
    {
      blackboard_->set<Sentry_BT::SentryStance>("current_stance",
                                                static_cast<Sentry_BT::SentryStance>(msg->position - 1));
    }
    // 更新目标位置
    if(msg->enemy_detected.is_detect)
    // if(false)
    {
      geometry_msgs::msg::Pose target_pose_in, target_pose;
      target_pose_in.position.x = (msg->enemy_detected.position.x) / 1000.0;  // 转换为米
      target_pose_in.position.y = (msg->enemy_detected.position.y) / 1000.0;
      target_pose_in.position.z = (msg->enemy_detected.position.z) / 1000.0;
      TransformPose(target_pose_in, target_pose);
      target_pose.orientation.w = 1.0;  // 设置默认朝向
      blackboard_->set<int>("target_armor_id", (int)msg->enemy_detected.armor_id);
      blackboard_->set<geometry_msgs::msg::Pose>("target_pose", target_pose);
    }

  }
  bool ros_interface::TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose)
  {
    // 创建TransformUtils实例
    auto transform_utils = std::make_shared<Sentry_BT::TransformUtils>();

    // 执行坐标转换
    bool success = transform_utils->transformPoseToBaseLink(input_pose, output_pose);

    return success;
  }

}  // namespace Sentry_BT