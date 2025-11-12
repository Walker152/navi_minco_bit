#include "bt_manager/tf_utils.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Sentry_BT
{

  TransformUtils::TransformUtils()
    : Node("transform_utils_node")
  {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gimbal_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sentry/gimbal_yaw",
        10,
        [this](const std_msgs::msg::Float32::ConstSharedPtr& msg) { this->publishStaticTransform(msg); });
  }
  void TransformUtils::publishStaticTransform(const std_msgs::msg::Float32::ConstSharedPtr& msg)
  {
    // 创建从base_link到gimbal的静态变换
    geometry_msgs::msg::TransformStamped static_transform;

  // 设置时间戳为当前时间延迟1秒
  static_transform.header.stamp = this->now() + rclcpp::Duration(1, 0);
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = "gimbal";

    // 设置变换参数（根据实际机器人结构调整）
    static_transform.transform.translation.x = 0.0;  // 云台在底盘中心的X偏移
    static_transform.transform.translation.y = 0.0;  // 云台在底盘中心的Y偏移
    static_transform.transform.translation.z = 0.5;  // 云台在底盘中心上方的Z高度
    // 设置旋转
    tf2::Quaternion quat;
    quat.setRPY(-M_PI / 2, 0, -M_PI / 2 + msg->data / 180 * M_PI);
    static_transform.transform.rotation = tf2::toMsg(quat);

    // 发布静态变换
    static_broadcaster_->sendTransform(static_transform);
    // RCLCPP_ERROR(this->get_logger(), "Published static transform from base_link to gimbal");
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
