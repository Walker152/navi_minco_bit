#include "bt_manager/ros_interface.hpp"

namespace Sentry_BT
{
  ros_interface::ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr)
    : Node("ros_interface_node")
    , blackboard_(blackboard_ptr)
  {
    // 订阅事件状态话题
    event_sub = this->create_subscription<robot_msgs::msg::EventStatus>(
        "/sentry/event_status",
        10,
        [this](const robot_msgs::msg::EventStatus::SharedPtr msg) { this->eventCallback(msg); });

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom",
                                                                10,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    // 更新当前位置
                                                                    current_pose_ = msg->pose.pose;
                                                                  });

    // 定时发布前哨站状态
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]()
        {
          auto current_mode = blackboard_->get<int>("current_mode");
          std_msgs::msg::Bool outpost_msg;
          if(current_mode == Sentry_BT::NavMode::RESPONSE &&
             std::hypot(current_pose_.position.x - nav_points[2].x, current_pose_.position.y - nav_points[2].y) < 1.0)
          {
            outpost_msg.data = true;
            outpost_pub->publish(outpost_msg);
          }
        });
    outpost_pub = this->create_publisher<std_msgs::msg::Bool>("/sentry/outpost_status", 10);
    // 创建导航客户端
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // 等待导航服务器可用
    while(!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_INFO(this->get_logger(), "等待导航服务器...");
    }
    RCLCPP_INFO(this->get_logger(), "导航服务器已连接");
  }

  void ros_interface::eventCallback(const robot_msgs::msg::EventStatus::SharedPtr msg)
  {
    // 更新黑板中的数据
    blackboard_->set<float>("health", ((int)msg->self_health) / 4.0);
    blackboard_->set<bool>("own_outpost_destroyed", msg->own_outpost_destroyed);
    blackboard_->set<int>("enemy_outpost_health", msg->enemy_outpost_health);
    blackboard_->set<bool>("bonus_active", msg->buff_active);
    blackboard_->set<bool>("target_valid", msg->enemy_detected.is_get);

    // 更新目标位置
    if(msg->enemy_detected.is_get)
    {
      geometry_msgs::msg::Pose target_pose_in, target_pose;
      target_pose_in.position.x = msg->enemy_detected.position.x;
      target_pose_in.position.y = msg->enemy_detected.position.y;
      target_pose_in.position.z = msg->enemy_detected.position.z;
      TransformPose(target_pose_in, target_pose);
      target_pose.orientation.w = 1.0;  // 设置默认朝向
      blackboard_->set<int>("target_armor_id", (int)msg->enemy_detected.armor_id);
      blackboard_->set<geometry_msgs::msg::Pose>("target_pose", target_pose);
    }

    // RCLCPP_INFO(this->get_logger(),
    //             "EventStatus received: health=%.2f, outpost=%s, buff=%s, target_locked=%s",
    //             msg->self_health,
    //             msg->own_outpost_destroyed ? "true" : "false",
    //             msg->buff_active ? "true" : "false",
    //             msg->enemy_detected.is_get ? "true" : "false");
  }

  bool ros_interface::publishNavigationGoal(const Sentry_BT::Point2D& goal)
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal.x;
    goal_msg.pose.pose.position.y = goal.y;
    goal_msg.pose.pose.orientation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "发送导航目标点: (%.2f, %.2f)", goal.x, goal.y);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
    {
      auto goal_handle = future.get();
      if(!goal_handle)
      {
        RCLCPP_INFO(get_logger(), "目标点被服务器拒绝");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::FAILURE));
      }
      else
      {
        RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::RUNNING));
      }
    };

    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
               const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
      (void)goal_handle;
      (void)feedback;
      // RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
    };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
    {
      if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "导航成功");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
      }
      else if(result.code == rclcpp_action::ResultCode::CANCELED)
      {
        RCLCPP_INFO(this->get_logger(), "导航被取消");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
      }
      else if(result.code == rclcpp_action::ResultCode::ABORTED)
      {
        RCLCPP_INFO(this->get_logger(), "导航中止");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "导航失败");
        blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::FAILURE));
      }
    };

    // 发送导航目标点
    nav_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  bool ros_interface::TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose)
  {
    // 创建TransformUtils实例
    auto transform_utils = std::make_shared<Sentry_BT::TransformUtils>();

    // 执行坐标转换
    bool success = transform_utils->transformPoseToBaseLink(input_pose, output_pose);

    if(success)
    {
      RCLCPP_INFO(this->get_logger(),
                  "坐标转换成功: (%.2f, %.2f) -> (%.2f, %.2f)",
                  input_pose.position.x,
                  input_pose.position.y,
                  output_pose.position.x,
                  output_pose.position.y);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "坐标转换失败");
      output_pose = input_pose;  // 失败时返回原始坐标
    }

    return success;
  }
}  // namespace Sentry_BT