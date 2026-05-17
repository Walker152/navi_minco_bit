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

#include <chrono>
#include <tf2/exceptions.h>

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
    
    // Z高度截断参数，增加开关防止默认误杀地面
    enable_z_truncation_ = this->declare_parameter<bool>("enable_z_truncation", false);
    z_truncation_offset_ = this->declare_parameter<double>("z_truncation_offset", 0.0);

    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(
        this->get_logger(), "Unknown filter_mode='%s', fallback to transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    // 独立线程监听TF (完美复刻原版)
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_buffer_->setUsingDedicatedThread(true);

    // 1. 读取原版的单区域旧参数 (保障向后兼容，不用修改已有yaml)
    double legacy_cx = this->declare_parameter<double>("position.x", 0.0);
    double legacy_cy = this->declare_parameter<double>("position.y", 0.0);
    double legacy_cz = this->declare_parameter<double>("position.z", 0.0);
    double legacy_sx = this->declare_parameter<double>("box_size.x", 12.0);
    double legacy_sy = this->declare_parameter<double>("box_size.y", 6.0);
    double legacy_sz = this->declare_parameter<double>("box_size.z", 3.5);
    bool legacy_rm = this->declare_parameter<bool>("remove_inside", true);

    // 2. 尝试读取多区域新参数
    std::vector<double> def_arr;
    std::vector<bool> def_bool;
    auto cx = this->declare_parameter<std::vector<double>>("crop_boxes.centers_x", def_arr);
    auto cy = this->declare_parameter<std::vector<double>>("crop_boxes.centers_y", def_arr);
    auto cz = this->declare_parameter<std::vector<double>>("crop_boxes.centers_z", def_arr);
    auto sx = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_x", def_arr);
    auto sy = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_y", def_arr);
    auto sz = this->declare_parameter<std::vector<double>>("crop_boxes.sizes_z", def_arr);
    auto rm = this->declare_parameter<std::vector<bool>>("crop_boxes.remove_inside", def_bool);

    size_t num_boxes = std::min({cx.size(), cy.size(), cz.size(), sx.size(), sy.size(), sz.size(), rm.size()});

    if (num_boxes > 0) {
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
      RCLCPP_INFO(this->get_logger(), "已加载 %zu 个 CropBox 配置.", boxes_.size());
    } else {
      // 回退使用遗留单体配置
      BoxParam b;
      b.center_x = static_cast<float>(legacy_cx);
      b.center_y = static_cast<float>(legacy_cy);
      b.center_z = static_cast<float>(legacy_cz);
      b.size_x = std::abs(static_cast<float>(legacy_sx));
      b.size_y = std::abs(static_cast<float>(legacy_sy));
      b.size_z = std::abs(static_cast<float>(legacy_sz));
      b.remove_inside = legacy_rm;
      boxes_.push_back(b);
      RCLCPP_INFO(this->get_logger(), "未找到多区域参数，已回退加载单区域旧版 YAML 配置.");
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
      "Multi-Crop filter started. frame=%s mode=%s z_trunc=%s(offset=%.3f), num_boxes=%zu",
      position_frame_.c_str(), filter_mode_.c_str(), 
      enable_z_truncation_ ? "ON" : "OFF", z_truncation_offset_, boxes_.size());
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
      // 1. 优先尝试严格匹配点云时间戳
      tf_out = tf_buffer_->lookupTransform(
        target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.05));
      return true;
    } catch (const tf2::ExtrapolationException & ex) {
      // 2. 专门捕获由于点云时间戳早于/晚于 TF 树时间而导致的报错
      // 自动降级：使用 tf2::TimePointZero 获取最新可用 TF，避免丢帧
      try {
        tf_out = tf_buffer_->lookupTransform(
          target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(50));
        return true; 
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
          *this->get_clock(), 2000,
          "TF 最新时间回退查询彻底失败: %s <- %s, 原因: %s",
          target_frame.c_str(), source_frame.c_str(), ex2.what());
        return false;
      }
    } catch (const tf2::TransformException & ex) {
      // 3. 捕获其他硬性 TF 错误 (如未连接的树等)
      RCLCPP_WARN_THROTTLE(this->get_logger(),
        *this->get_clock(), 2000,
        "TF 基础查询失败: %s <- %s, 原因: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
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
      sensor_msgs::msg::PointCloud2 filtered_msg = *msg;
      cloud_pub_->publish(filtered_msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_filter = cloud_input;

    // --- 1. 使用 PassThrough 进行 Z轴高度截断 (默认关闭防误杀) ---
    if (enable_z_truncation_ && has_odom_) {
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

    // --- 2. 坐标系转换逻辑 ---
    if (filter_mode_ == "transform_cloud") {
      if (need_transform) {
        geometry_msgs::msg::TransformStamped tf_cloud_to_filter;
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_cloud_to_filter)) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_for_filter, *cloud_transformed, transformToMatrix(tf_cloud_to_filter));
        cloud_for_filter = cloud_transformed;
      }
    } else {  // transform_center
      if (need_transform) {
        geometry_msgs::msg::TransformStamped tf_filter_to_cloud;
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_filter_to_cloud)) return;
        transform_matrix = transformToMatrix(tf_filter_to_cloud);
      }
      filter_frame = cloud_frame;
    }

    // --- 3. 循环处理多个 CropBox ---
    for (const auto & box : boxes_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(cloud_for_filter);

      Eigen::Vector3f center(box.center_x, box.center_y, box.center_z);

      if (filter_mode_ == "transform_center" && need_transform) {
        center = transformPoint(transform_matrix, center);
      }

      const float half_x = 0.5f * box.size_x;
      const float half_y = 0.5f * box.size_y;
      const float half_z = 0.5f * box.size_z;

      crop.setMin(Eigen::Vector4f(center.x() - half_x, center.y() - half_y, center.z() - half_z, 1.0f));
      crop.setMax(Eigen::Vector4f(center.x() + half_x, center.y() + half_y, center.z() + half_z, 1.0f));
      crop.setNegative(box.remove_inside);

      crop.filter(*cloud_cropped);
      cloud_for_filter = cloud_cropped;
    }

    // 复刻原版: 强制将发布属性标记为 Dense 格式，防止 RViz 异常
    cloud_for_filter->width = static_cast<uint32_t>(cloud_for_filter->points.size());
    cloud_for_filter->height = 1;
    cloud_for_filter->is_dense = true;

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
  
  bool enable_z_truncation_{false};
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