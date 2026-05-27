#include "bt_manager/utils/tf_utils.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/duration.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
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

  tf_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
    this->publishDynamicTransform();
  });

  publishDynamicTransform();
}

void TransformUtils::updateCurrentPose(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(current_pose_mutex_);
  current_pose_ = pose;
}
void TransformUtils::updateGimbalYawInit(float yaw)
{
  NormalizeAngle(yaw);
  latest_gimbal_yaw_deg_.store(yaw, std::memory_order_relaxed);
  const double yaw_rad = static_cast<double>(yaw) * M_PI / 180.0;
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);
  {
    std::lock_guard<std::mutex> lock(gimbal_to_map_mutex_);
    gimbal_to_map_rotation_ = {cos_yaw, -sin_yaw, sin_yaw, cos_yaw};
  }
  gimbal_yaw_initialized_.store(true, std::memory_order_relaxed);
}

float TransformUtils::getCurrentYawDeg()
{
  geometry_msgs::msg::Pose pose_copy;
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    pose_copy = current_pose_;
  }

  float yaw_deg =
    tf2::getYaw(tf2::Quaternion(
      pose_copy.orientation.x, pose_copy.orientation.y, pose_copy.orientation.z, pose_copy.orientation.w)) *
    180.0f / static_cast<float>(M_PI);
  NormalizeAngle(yaw_deg);
  return yaw_deg;
}

void TransformUtils::NormalizeAngle(float & angle)
{
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
}
void TransformUtils::publishDynamicTransform()
{
  // 创建从base_link到gimbal的动态变换
  geometry_msgs::msg::TransformStamped dynamic_transform;

  // 设置时间戳为当前时间
  dynamic_transform.header.stamp = this->now() - rclcpp::Duration(0, 10000000);
  dynamic_transform.header.frame_id = "base_link";
  dynamic_transform.child_frame_id = "gimbal";

  // 设置变换参数（根据实际机器人结构调整）
  dynamic_transform.transform.translation.x = 0.0;
  dynamic_transform.transform.translation.y = 0.0;
  dynamic_transform.transform.translation.z = 0.5;

  const float yaw_deg = latest_gimbal_yaw_deg_.load(std::memory_order_relaxed);
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw_deg / 180 * M_PI);
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

bool TransformUtils::transformGimbalToMap(const geometry_msgs::msg::Pose & input_pose,
  geometry_msgs::msg::Pose & output_pose)
{
  geometry_msgs::msg::Pose current_pose_copy;
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose_copy = current_pose_;
  }

  std::array<double, 4> rotation;
  {
    std::lock_guard<std::mutex> lock(gimbal_to_map_mutex_);
    rotation = gimbal_to_map_rotation_;
  }

  output_pose.position.x = current_pose_copy.position.x +
                           rotation[0] * input_pose.position.x +
                           rotation[1] * input_pose.position.y;
  output_pose.position.y = current_pose_copy.position.y +
                           rotation[2] * input_pose.position.x +
                           rotation[3] * input_pose.position.y;
  output_pose.position.z = current_pose_copy.position.z + input_pose.position.z;

  output_pose.orientation = input_pose.orientation;
  if (output_pose.orientation.x == 0.0 && output_pose.orientation.y == 0.0 &&
      output_pose.orientation.z == 0.0 && output_pose.orientation.w == 0.0) {
    output_pose.orientation.w = 1.0;
  }

  return true;
}

bool TransformUtils::waitForTransform(const std::string & target_frame, const std::string & source_frame)
{
  try {
    return tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
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

void TransformUtils::transformYaw(
  float yaw_in, float & yaw_out, const std::string & target_frame, const std::string & source_frame)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, 0, yaw_in * M_PI / 180.0f);
    geometry_msgs::msg::Quaternion rotation_msg = tf2::toMsg(rotation_quat);

    geometry_msgs::msg::Pose pose_in;
    pose_in.orientation = rotation_msg;

    geometry_msgs::msg::Pose pose_out;
    tf2::doTransform(pose_in, pose_out, transform_stamped);

    yaw_out =
      tf2::getYaw(tf2::Quaternion(
        pose_out.orientation.x, pose_out.orientation.y, pose_out.orientation.z, pose_out.orientation.w)) *
      180.0f / static_cast<float>(M_PI);
    NormalizeAngle(yaw_out);
  } catch (const tf2::TransformException & ex) {
    yaw_out = yaw_in;  // 如果转换失败，返回输入角度
  }
}
}  // namespace Sentry_BT
