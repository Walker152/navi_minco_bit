#include "bt_manager/ros_interface.hpp"

#include <cmath>
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

    // 定时发布行为状态（10Hz）
    behavior_pub = this->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          const auto current_mode = blackboard_->get<int>("current_mode");
          const auto desired_stance = blackboard_->get<Sentry_BT::SentryStance>("desired_stance");
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
          behavior_msg.desire_lifter_pos = ros_interfaces::msg::Behavior::LIFTER_BOTTOM; // TODO: 这里暂时写死，后续根据实际情况修改
          behavior_pub->publish(behavior_msg);
        });
  }

  void ros_interface::publishCmdVel(double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub->publish(cmd_vel);
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
    blackboard_->set<Sentry_BT::SentryStance>("current_stance", static_cast<Sentry_BT::SentryStance>(msg->current_stance));

    // 3月9日新增数据，不知道有什么用，先放在黑板里
    blackboard_->set<int>("game_status", static_cast<int>(msg->game_status));
    blackboard_->set<int>("lifter_pos_now", static_cast<int>(msg->lifter_pos_now));
    blackboard_->set<float>("gimbal_yaw", msg->gimbal_yaw);
    blackboard_->set<uint16_t>("num_shoot", msg->num_shoot);

    // if(msg->position >= 1 && msg->position <= 3)
    // {
    //   blackboard_->set<Sentry_BT::SentryStance>("current_stance",
    //                                             static_cast<Sentry_BT::SentryStance>(msg->position - 1));
    // }
    // 更新目标位置
    if(msg->enemy_detected.is_detect)
    // if(false)
    {
      geometry_msgs::msg::Pose target_pose_in, target_pose;
      target_pose_in.position.x = (msg->enemy_detected.position.x) / 1000.0;  // 转换为米
      target_pose_in.position.y = (msg->enemy_detected.position.y) / 1000.0;
      target_pose_in.position.z = (msg->enemy_detected.position.z) / 1000.0;
      TransformPose(target_pose_in, target_pose);
      std::cout << "Target pose: " << target_pose.position.x << ", " << target_pose.position.y << std::endl;
      std::cout<<"target pose in: "<<target_pose_in.position.x<<","<<target_pose_in.position.y<<","<<target_pose_in.position.z<<std::endl;
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