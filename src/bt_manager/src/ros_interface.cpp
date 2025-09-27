#include "bt_manager/ros_interface.hpp"

namespace Sentry_BT
{
ros_interface::ros_interface(std::shared_ptr<Blackboard> blackboard_ptr)
: Node("ros_interface_node"),
  blackboard_(blackboard_ptr)
{
    // 发布静态变换

    // 订阅事件状态话题
    event_sub = this->create_subscription<robot_msgs::msg::EventStatus>(
        "/sentry/event_status", 10,
        [this](const robot_msgs::msg::EventStatus::SharedPtr msg) {
            this->eventCallback(msg);
        });
        
    // 创建导航客户端
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    // 等待导航服务器可用
    while (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(this->get_logger(), "等待导航服务器...");
    }
    RCLCPP_INFO(this->get_logger(), "导航服务器已连接");
}   

ros_interface::~ros_interface()
{   
}

void ros_interface::eventCallback(const robot_msgs::msg::EventStatus::SharedPtr msg)
{
    // 更新黑板中的数据
    blackboard_->set("health", msg->self_health);
    blackboard_->set("outpost_destroyed", msg->own_outpost_destroyed);
    blackboard_->set("bonus_active", msg->buff_active);
    blackboard_->set("target_valid", msg->enemy_detected.is_get);
    
    // 更新目标位置
    if (msg->enemy_detected.is_get) {
        geometry_msgs::msg::Pose target_pose_in, target_pose;
        target_pose_in.position.x = msg->enemy_detected.position.x;
        target_pose_in.position.y = msg->enemy_detected.position.y;
        target_pose_in.position.z = msg->enemy_detected.position.z;
        TransformPose(target_pose_in, target_pose);
        target_pose.orientation.w = 1.0; // 设置默认朝向
        blackboard_->set("target_armor_id", msg->enemy_detected.armor_id);
        blackboard_->set("target_pose", target_pose);
    }
    
    RCLCPP_INFO(this->get_logger(), "EventStatus received: health=%.2f, outpost=%s, buff=%s, target_locked=%s",
                msg->self_health,
                msg->own_outpost_destroyed ? "true" : "false",
                msg->buff_active ? "true" : "false",
                msg->enemy_detected.is_get ? "true" : "false");
}

bool ros_interface::publishNavigationGoal(const Sentry_BT::Point2D & goal)
{
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = goal.x;
  goal_msg.pose.pose.position.y = goal.y;
  goal_msg.pose.pose.orientation.w = 1.0;
  RCLCPP_INFO(this->get_logger(), "发送导航目标点: (%.2f, %.2f)", goal.x, goal.y);
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(get_logger(), "目标点被服务器拒绝");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::FAILURE));
        } else {
          RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::RUNNING));
        }
      };
      
  send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        (void)goal_handle; 
        (void)feedback;
        // RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
      };
      
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "导航成功");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_INFO(this->get_logger(), "导航被取消");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_INFO(this->get_logger(), "导航中止");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::IDLE));
        } else {
          RCLCPP_INFO(this->get_logger(), "导航失败");
          blackboard_->set("nav_status", static_cast<int>(Sentry_BT::NavStatus::FAILURE));
        }
      };

  // 发送导航目标点
  nav_client_->async_send_goal(goal_msg, send_goal_options);
  return true;
}

bool ros_interface::TransformPose(const geometry_msgs::msg::Pose & input_pose, geometry_msgs::msg::Pose & output_pose)
{
    try
    {
      tf2_ros::Buffer tf_buffer(this->get_clock());
      tf2_ros::TransformListener tf_listener(tf_buffer);
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer.lookupTransform("base_link", "gimbal", tf2::TimePointZero, tf2::durationFromSec(1.0));
      tf2::doTransform(input_pose, output_pose, transform_stamped);
      RCLCPP_INFO(this->get_logger(), "坐标转换成功: (%.2f, %.2f) -> (%.2f, %.2f)", 
                  input_pose.position.x, input_pose.position.y,
                  output_pose.position.x, output_pose.position.y);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    
    output_pose = input_pose;
    return true;
}
} // namespace Sentry_BT