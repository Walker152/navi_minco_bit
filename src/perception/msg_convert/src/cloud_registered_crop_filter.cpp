#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace msg_convert {

struct BoxParam {
  float center_x;
  float center_y;
  float center_z;
  float size_x;
  float size_y;
  float size_z;
  bool remove_inside;
};

class CloudRegisteredCropFilterNode : public rclcpp::Node
{
public:
  CloudRegisteredCropFilterNode() : Node("cloud_registered_crop_filter")
  {
    const auto input_topic = this->declare_parameter<std::string>("input_topic", "/cloud_registered");
    const auto output_topic = this->declare_parameter<std::string>("output_topic", "/cloud_registered_filtered");
    const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto queue_size = this->declare_parameter<int>("queue_size", 10);

    position_frame_ = this->declare_parameter<std::string>("position_frame", "camera_init");
    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    z_truncation_offset_ = this->declare_parameter<double>("z_truncation_offset", 0.0);

    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(this->get_logger(), "未知的 filter_mode='%s', 回退使用 transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_buffer_->setUsingDedicatedThread(true);

    // 读取多个CropBox的数组参数
    auto cx = this->declare_parameter<std::vector<double>>("crop_boxes.centers_x", {0.0});
    auto cy = this->declare_parameter<std::vector<double>>("crop_boxes.centers_y", {0.0});
    auto cz = this->declare_parameter<std::vector<double>>("crop_boxes.centers_z", {0.0});
    auto sx = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_x", {12.0});
    auto sy = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_y", {6.0});
    auto sz = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_z", {3.5});
    auto rm = this->declare_parameter<std::vector<bool>>("crop_boxes.remove_inside", {true});

    size_t num_boxes = std::min({cx.size(), cy.size(), cz.size(), sx.size(), sy.size(), sz.size(), rm.size()});
    if (num_boxes == 0) {
      RCLCPP_WARN(this->get_logger(), "未配置任何有效的 CropBox 参数!");
    }

    for (size_t i = 0; i < num_boxes; ++i) {
      BoxParam b;
      b.center_x = static_cast<float>(cx[i]);
      b.center_y = static_cast<float>(cy[i]);
      b.center_z = static_cast<float>(cz[i]);
      b.size_x = std::abs(static_cast<float>(sx[i]));
      b.size_y = std::abs(static_cast<float>(sy[i]));
      b.size_z = std::abs(static_cast<float>(sz[i]));
      b.remove_inside = rm[i];
      boxes_.push_back(b);
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));
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

  // 稳健的 TF 查询函数：支持精确时间查找失败时自动回退到最新 TF
  bool lookupTransformRobust(const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & tf_out)
  {
    try {
      // 1. 优先尝试点云的精确时间戳
      tf_out = tf_buffer_->lookupTransform(
        target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      try {
        // 2. 如果精确时间查询失败，回退请求最新可用的 TF
        tf_out = tf_buffer_->lookupTransform(
          target_frame, source_frame, tf2::TimePointZero, rclcpp::Duration::from_seconds(0.05));
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF 严格时间匹配失败，已回退到最新可用 TF: %s <- %s",
          target_frame.c_str(), source_frame.c_str());
        return true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF 查询彻底失败: %s <- %s, 原因: %s",
          target_frame.c_str(), source_frame.c_str(), ex2.what());
        return false;
      }
    }
  }

  static Eigen::Vector3f transformPoint(const Eigen::Matrix4f & tf, const Eigen::Vector3f & p)
  {
    const Eigen::Vector4f hp(p.x(), p.y(), p.z(), 1.0f);
    const Eigen::Vector4f out = tf * hp;
    return out.head<3>();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_z_ = msg->pose.pose.position.z;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_input);

    if (cloud_input->empty()) {
      cloud_pub_->publish(*msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_input;

    // ==========================================================
    // 1. Z轴高度截断 (PassThrough)
    // ==========================================================
    if (has_odom_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      double z_min = current_odom_z_ + z_truncation_offset_;
      double z_max = std::numeric_limits<float>::max();
      pass.setFilterLimits(z_min, z_max);
      pass.filter(*cloud_passthrough);
      cloud_filtered = cloud_passthrough;
    }

    // ==========================================================
    // 2. 坐标系转换与多区域裁减计算 (严格兼容你的原始逻辑)
    // ==========================================================
    const std::string cloud_frame = msg->header.frame_id;
    std::string filter_frame = position_frame_;
    if (filter_frame.empty()) {
      filter_frame = cloud_frame;
    }

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    bool need_transform = (filter_frame != cloud_frame);

    if (need_transform) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      if (filter_mode_ == "transform_cloud") {
        // target: filter_frame, source: cloud_frame
        if (!lookupTransformRobust(filter_frame, cloud_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, transform_matrix);
        cloud_filtered = cloud_transformed;
      } else {
        // transform_center 模式下：仅获取把 filter_frame 转到 cloud_frame 的变换矩阵
        // target: cloud_frame, source: filter_frame
        if (!lookupTransformRobust(cloud_frame, filter_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);
      }
    }

    for (const auto & box : boxes_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(cloud_filtered);

      float bx = box.center_x;
      float by = box.center_y;
      float bz = box.center_z;

      // 如果是 transform_center 模式，仅将定义的中心点坐标转换到云本身的坐标系中
      if (need_transform && filter_mode_ == "transform_center") {
        Eigen::Vector3f center(bx, by, bz);
        center = transformPoint(transform_matrix, center);
        bx = center.x();
        by = center.y();
        bz = center.z();
      }

      const float half_x = 0.5f * box.size_x;
      const float half_y = 0.5f * box.size_y;
      const float half_z = 0.5f * box.size_z;

      // 由于无论是 transform_cloud 还是 transform_center，现在的参考系与点云坐标系都在这里实现了对齐，直接设 Min/Max
      crop.setMin(Eigen::Vector4f(bx - half_x, by - half_y, bz - half_z, 1.0f));
      crop.setMax(Eigen::Vector4f(bx + half_x, by + half_y, bz + half_z, 1.0f));
      crop.setNegative(box.remove_inside);

      crop.filter(*cloud_cropped);
      cloud_filtered = cloud_cropped;
    }

    // ==========================================================
    // 3. 发布最终结果
    // ==========================================================
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = (filter_mode_ == "transform_cloud") ? filter_frame : cloud_frame;
    cloud_pub_->publish(filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::vector<BoxParam> boxes_;
  std::string position_frame_;
  std::string filter_mode_;
  
  double z_truncation_offset_{0.0};
  double current_odom_z_{0.0};
  bool has_odom_{false};

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