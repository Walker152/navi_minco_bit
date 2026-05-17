#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace msg_convert {

class CloudRegisteredCropFilterNode : public rclcpp::Node
{
public:
  CloudRegisteredCropFilterNode() : Node("cloud_registered_crop_filter")
  {
    const auto input_topic = this->declare_parameter<std::string>("input_topic", "/cloud_registered");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/cloud_registered_filtered");
    const auto queue_size = this->declare_parameter<int>("queue_size", 10);

    center_x_ = static_cast<float>(this->declare_parameter<double>("position.x", 0.0));
    center_y_ = static_cast<float>(this->declare_parameter<double>("position.y", 0.0));
    center_z_ = static_cast<float>(this->declare_parameter<double>("position.z", 0.0));
    position_frame_ = this->declare_parameter<std::string>("position_frame", "camera_init");

    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    remove_inside_ = this->declare_parameter<bool>("remove_inside", true);
    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(
        this->get_logger(), "Unknown filter_mode='%s', fallback to transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_buffer_->setUsingDedicatedThread(true);

    size_x_ = static_cast<float>(this->declare_parameter<double>("box_size.x", 12.0));
    size_y_ = static_cast<float>(this->declare_parameter<double>("box_size.y", 6.0));
    size_z_ = static_cast<float>(this->declare_parameter<double>("box_size.z", 3.5));

    if (size_x_ <= 0.0f || size_y_ <= 0.0f || size_z_ <= 0.0f) {
      RCLCPP_WARN(this->get_logger(),
        "box_size must be positive. Using absolute value: (%.3f, %.3f, %.3f)",
        size_x_,
        size_y_,
        size_z_);
      size_x_ = std::abs(size_x_);
      size_y_ = std::abs(size_y_);
      size_z_ = std::abs(size_z_);
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    RCLCPP_INFO(this->get_logger(),
      "Crop filter started. input=%s output=%s center=(%.3f, %.3f, %.3f) frame=%s mode=%s remove_inside=%s "
      "box_size=(%.3f, %.3f, %.3f)",
      input_topic.c_str(),
      output_topic.c_str(),
      center_x_,
      center_y_,
      center_z_,
      position_frame_.c_str(),
      filter_mode_.c_str(),
      remove_inside_ ? "true" : "false",
      size_x_,
      size_y_,
      size_z_);
  }

private:
  static Eigen::Matrix4f transformToMatrix(const geometry_msgs::msg::TransformStamped & tf)
  {
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;

    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    const Eigen::Quaternionf quat(
      static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z));
    m.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    m(0, 3) = static_cast<float>(t.x);
    m(1, 3) = static_cast<float>(t.y);
    m(2, 3) = static_cast<float>(t.z);
    return m;
  }

  bool lookupTransform(const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      tf_out = tf_buffer_->lookupTransform(
        target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup failed: %s <- %s, reason: %s",
        target_frame.c_str(),
        source_frame.c_str(),
        ex.what());
      return false;
    }
  }

  static Eigen::Vector3f transformPoint(const Eigen::Matrix4f & tf, const Eigen::Vector3f & p)
  {
    const Eigen::Vector4f hp(p.x(), p.y(), p.z(), 1.0f);
    const Eigen::Vector4f out = tf * hp;
    return out.head<3>();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_input);

    if (cloud_input->empty()) {
      sensor_msgs::msg::PointCloud2 filtered_msg = *msg;
      cloud_pub_->publish(filtered_msg);
      return;
    }

    const std::string cloud_frame = msg->header.frame_id;
    std::string filter_frame = position_frame_;
    if (filter_frame.empty()) {
      filter_frame = cloud_frame;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_filter = cloud_input;
    Eigen::Vector3f center(center_x_, center_y_, center_z_);

    if (filter_mode_ == "transform_cloud") {
      if (filter_frame != cloud_frame) {
        geometry_msgs::msg::TransformStamped tf_cloud_to_filter;
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_cloud_to_filter)) {
          return;
        }

        cloud_for_filter = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_input, *cloud_for_filter, transformToMatrix(tf_cloud_to_filter));
      }
    } else {  // transform_center
      if (filter_frame != cloud_frame) {
        geometry_msgs::msg::TransformStamped tf_filter_to_cloud;
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_filter_to_cloud)) {
          return;
        }
        center = transformPoint(transformToMatrix(tf_filter_to_cloud), center);
      }
      filter_frame = cloud_frame;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

    const float half_x = 0.5f * size_x_;
    const float half_y = 0.5f * size_y_;
    const float half_z = 0.5f * size_z_;

    const float min_x = center.x() - half_x;
    const float max_x = center.x() + half_x;
    const float min_y = center.y() - half_y;
    const float max_y = center.y() + half_y;
    const float min_z = center.z() - half_z;
    const float max_z = center.z() + half_z;

    cloud_out->points.reserve(cloud_for_filter->points.size());
    for (const auto & p : cloud_for_filter->points) {
      if (!pcl::isFinite(p)) {
        continue;
      }

      const bool inside =
        (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y) && (p.z >= min_z && p.z <= max_z);

      const bool keep = remove_inside_ ? !inside : inside;
      if (keep) {
        cloud_out->points.push_back(p);
      }
    }
    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_out, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = filter_frame;
    cloud_pub_->publish(filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  float center_x_{0.0f};
  float center_y_{0.0f};
  float center_z_{0.0f};
  std::string position_frame_;
  std::string filter_mode_;
  bool remove_inside_{true};
  float size_x_{12.0f};
  float size_y_{6.0f};
  float size_z_{3.5f};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace msg_convert

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<msg_convert::CloudRegisteredCropFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
