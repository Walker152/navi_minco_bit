#include "bt_manager/utils/tf_utils.hpp"

#include <chrono>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Sentry_BT
{

  TransformUtils::TransformUtils()
    : Node("transform_utils_" +
               std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 10000),
           rclcpp::NodeOptions().use_global_arguments(false))
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gimbal_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sentry/gimbal_yaw",
        10,
        [this](const std_msgs::msg::Float32::ConstSharedPtr& msg) { this->publishDynamicTransform(msg); }); // 用dynamic版本
  }
  void TransformUtils::publishDynamicTransform(const std_msgs::msg::Float32::ConstSharedPtr& msg)
  {
    // 创建从base_link到gimbal的动态变换
    geometry_msgs::msg::TransformStamped dynamic_transform;

    // 设置时间戳为当前时间
    dynamic_transform.header.stamp = this->now();
    dynamic_transform.header.frame_id = "base_link";
    dynamic_transform.child_frame_id = "gimbal";

    // 设置变换参数（根据实际机器人结构调整）
    dynamic_transform.transform.translation.x = 0.0;
    dynamic_transform.transform.translation.y = 0.0;
    dynamic_transform.transform.translation.z = 0.5;

    tf2::Quaternion quat;
    quat.setRPY(-M_PI / 2, 0, -M_PI / 2 + msg->data / 180 * M_PI);
    dynamic_transform.transform.rotation = tf2::toMsg(quat);

    // 发布动态变换
    tf_broadcaster_->sendTransform(dynamic_transform);
    // RCLCPP_INFO(this->get_logger(), "Published dynamic transform from base_link to gimbal");
  }

  bool TransformUtils::transformPoseToBaseLink(const geometry_msgs::msg::Pose& input_pose,
                                               geometry_msgs::msg::Pose& output_pose)
  {
    try
    {
      // 查找从gimbal到map的变换
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("map", "gimbal", tf2::TimePointZero, tf2::durationFromSec(1.0));

      tf2::doTransform(input_pose, output_pose, transform_stamped);

      // RCLCPP_INFO(this->get_logger(), "Transformed pose from gimbal to map");
      return true;
    }
    catch(const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }
  }

}  // namespace Sentry_BT