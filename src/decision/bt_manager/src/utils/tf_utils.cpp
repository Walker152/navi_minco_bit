#include "bt_manager/utils/tf_utils.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Sentry_BT {

TransformUtils::TransformUtils()
: Node("transform_utils_" +
         std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 10000),
    rclcpp::NodeOptions().use_global_arguments(false))
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  gimbal_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/sentry/gimbal_yaw", 10, [this](const std_msgs::msg::Float32::ConstSharedPtr & msg) {
      this->updateGimbalYaw(msg);
    });

  tf_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
    this->publishDynamicTransform();
  });

  publishDynamicTransform();
}

void TransformUtils::updateGimbalYaw(const std_msgs::msg::Float32::ConstSharedPtr & msg)
{
  latest_gimbal_yaw_deg_.store(msg->data, std::memory_order_relaxed);
}

void TransformUtils::publishDynamicTransform()
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

  const float yaw_deg = latest_gimbal_yaw_deg_.load(std::memory_order_relaxed);
  tf2::Quaternion quat;
  quat.setRPY(-M_PI / 2, 0, -M_PI / 2 + yaw_deg / 180 * M_PI);
  dynamic_transform.transform.rotation = tf2::toMsg(quat);

  // 发布动态变换
  tf_broadcaster_->sendTransform(dynamic_transform);
  // RCLCPP_INFO(this->get_logger(), "Published dynamic transform from base_link to gimbal");
}

bool TransformUtils::transformPoseToMap(const geometry_msgs::msg::Pose & input_pose,
  geometry_msgs::msg::Pose & output_pose,
  const std::string & child_frame)
{
  try {
    // 查找从child_frame到map的变换（非阻塞，获取最新）
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform("map", child_frame, tf2::TimePointZero);

    tf2::doTransform(input_pose, output_pose, transform_stamped);

    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
}

bool TransformUtils::transformMapPose(const geometry_msgs::msg::Pose & input_pose,
  geometry_msgs::msg::Pose & output_pose,
  const std::string & target_frame)
{
  try {
    // 查找从map到target_frame的变换（非阻塞，获取最新）
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(target_frame, "map", tf2::TimePointZero);

    tf2::doTransform(input_pose, output_pose, transform_stamped);

    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
}
}  // namespace Sentry_BT