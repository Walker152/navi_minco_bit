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
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/cloud_registered_filtered");
    const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto queue_size = this->declare_parameter<int>("queue_size", 10);

    position_frame_ = this->declare_parameter<std::string>("position_frame", "camera_init");
    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    
    // Z高度截断偏移量 (截断阈值 = 实际odom_z + z_truncation_offset_)
    z_truncation_offset_ = this->declare_parameter<double>("z_truncation_offset", 0.0);

    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(
        this->get_logger(), "Unknown filter_mode='%s', fallback to transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    // 【完全保留你的原版 TF 初始化，使用独立线程】
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_buffer_->setUsingDedicatedThread(true);

    // 读取多个 CropBox 参数
    auto cx = this->declare_parameter<std::vector<double>>("crop_boxes.centers_x", {0.0});
    auto cy = this->declare_parameter<std::vector<double>>("crop_boxes.centers_y", {0.0});
    auto cz = this->declare_parameter<std::vector<double>>("crop_boxes.centers_z", {0.0});
    auto sx = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_x", {12.0});
    auto sy = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_y", {6.0});
    auto sz = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_z", {3.5});
    auto rm = this->declare_parameter<std::vector<bool>>("crop_boxes.remove_inside", {true});

    size_t num_boxes = std::min({cx.size(), cy.size(), cz.size(), sx.size(), sy.size(), sz.size(), rm.size()});
    if (num_boxes == 0) {
      RCLCPP_WARN(this->get_logger(), "No valid CropBox parameters found!");
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

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    RCLCPP_INFO(this->get_logger(),
      "Multi-Crop filter started. input=%s output=%s frame=%s mode=%s z_offset=%.3f, num_boxes=%zu",
      input_topic.c_str(), output_topic.c_str(), position_frame_.c_str(), filter_mode_.c_str(), 
      z_truncation_offset_, boxes_.size());
  }

private:
  // 【完全保留原版逻辑】
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

  // 【完全回退到你验证过且能稳定运行的原版 TF 查找方法】
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

  // 【完全保留原版逻辑】
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
      sensor_msgs::msg::PointCloud2 filtered_msg = *msg;
      cloud_pub_->publish(filtered_msg);
      return;
    }

    // --- 1. 使用 PassThrough 进行 Z轴高度截断 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_filter = cloud_input;
    if (has_odom_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_for_filter);
      pass.setFilterFieldName("z");
      double z_min = current_odom_z_ + z_truncation_offset_;
      double z_max = std::numeric_limits<float>::max();
      pass.setFilterLimits(z_min, z_max);
      pass.filter(*cloud_passthrough);
      cloud_for_filter = cloud_passthrough;
    }

    const std::string cloud_frame = msg->header.frame_id;
    std::string filter_frame = position_frame_;
    if (filter_frame.empty()) {
      filter_frame = cloud_frame;
    }

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    bool need_transform = (filter_frame != cloud_frame);

    // --- 2. 严格遵循原版的 TF 获取与应用逻辑 ---
    if (filter_mode_ == "transform_cloud") {
      if (need_transform) {
        geometry_msgs::msg::TransformStamped tf_cloud_to_filter;
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_cloud_to_filter)) {
          return; // 原版行为：查不到就跳过这一帧
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_for_filter, *cloud_transformed, transformToMatrix(tf_cloud_to_filter));
        cloud_for_filter = cloud_transformed;
      }
    } else {  // transform_center
      if (need_transform) {
        geometry_msgs::msg::TransformStamped tf_filter_to_cloud;
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_filter_to_cloud)) {
          return; // 原版行为：查不到就跳过这一帧
        }
        transform_matrix = transformToMatrix(tf_filter_to_cloud);
      }
      filter_frame = cloud_frame;
    }

    // --- 3. 循环处理多个 CropBox 区域 (严格复现原版的 AABB 判断) ---
    for (const auto & box : boxes_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(cloud_for_filter);

      Eigen::Vector3f center(box.center_x, box.center_y, box.center_z);

      // 如果是 transform_center，原版代码是将设定的中心点经过TF投影到当前点云的坐标系下
      if (filter_mode_ == "transform_center" && need_transform) {
        center = transformPoint(transform_matrix, center);
      }

      const float half_x = 0.5f * box.size_x;
      const float half_y = 0.5f * box.size_y;
      const float half_z = 0.5f * box.size_z;

      // 原版是对变换后的 center 直接进行基于当前坐标轴的正交加减。
      // 使用 CropBox 直接设置这些正交的边界，实现与原版逐点遍历绝对等价的效果，避免引入不符合预期的旋转(OBB)。
      crop.setMin(Eigen::Vector4f(center.x() - half_x, center.y() - half_y, center.z() - half_z, 1.0f));
      crop.setMax(Eigen::Vector4f(center.x() + half_x, center.y() + half_y, center.z() + half_z, 1.0f));
      crop.setNegative(box.remove_inside);

      crop.filter(*cloud_cropped);
      cloud_for_filter = cloud_cropped;
    }

    // --- 4. 发布结果 ---
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_for_filter, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = filter_frame;
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