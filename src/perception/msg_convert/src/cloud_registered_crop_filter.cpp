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

// 定义单个CropBox的配置结构体
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
    
    // Z高度截断偏移量 (截断阈值 = 实际odom_z + z_truncation_offset_)
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

    // 以最小数组长度为准，防止越界
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
      
      RCLCPP_INFO(this->get_logger(),
        "加载 CropBox [%zu]: 中心(%.2f, %.2f, %.2f), 尺寸(%.2f, %.2f, %.2f), 移除内部: %s",
        i, b.center_x, b.center_y, b.center_z, b.size_x, b.size_y, b.size_z, b.remove_inside ? "true" : "false");
    }

    // 订阅里程计信息用于获取实际高度
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    // 订阅输入点云
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    // 发布过滤后的点云
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    RCLCPP_INFO(this->get_logger(),
      "Crop Filter 节点启动. input=%s output=%s odom=%s frame=%s mode=%s z_offset=%.2f",
      input_topic.c_str(), output_topic.c_str(), odom_topic.c_str(), 
      position_frame_.c_str(), filter_mode_.c_str(), z_truncation_offset_);
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
        *this->get_clock(), 2000,
        "TF 查询失败: %s <- %s, 原因: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
    }
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
    // 1. 使用 PassThrough 进行 Z轴实际高度截断
    // ==========================================================
    if (has_odom_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      // 截断 Z 轴在 (odom_z + offset) 以下的部分，只保留以上的数据
      double z_min = current_odom_z_ + z_truncation_offset_;
      double z_max = std::numeric_limits<float>::max();
      pass.setFilterLimits(z_min, z_max);
      pass.filter(*cloud_passthrough);
      
      cloud_filtered = cloud_passthrough;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "尚未收到 Odom 消息，暂时跳过高度截断.");
    }

    // ==========================================================
    // 2. 使用 CropBox 进行多区域边界裁减
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
        // 将点云变换到 filter_frame 下处理
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, transform_matrix);
        cloud_filtered = cloud_transformed;
      } else {
        // transform_center 模式: 保持点云在其原坐标系，但计算 filter_frame 到 cloud_frame 的变换
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);
      }
    }

    // 依次经过每一个 CropBox 过滤器
    for (const auto & box : boxes_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(cloud_filtered);

      const float half_x = 0.5f * box.size_x;
      const float half_y = 0.5f * box.size_y;
      const float half_z = 0.5f * box.size_z;

      // 在 filter_frame 下的局部边界
      crop.setMin(Eigen::Vector4f(box.center_x - half_x, box.center_y - half_y, box.center_z - half_z, 1.0f));
      crop.setMax(Eigen::Vector4f(box.center_x + half_x, box.center_y + half_y, box.center_z + half_z, 1.0f));
      crop.setNegative(box.remove_inside);

      if (need_transform && filter_mode_ == "transform_center") {
        // 将 CropBox 施加一个逆变换: 点云保持在 cloud_frame 中, 而 Box 是定义在 filter_frame 中。
        // 通过设置该变换矩阵，可以让点在判断前先变换到 filter_frame，实现 OBB (有向包围盒) 的精确裁剪。
        crop.setTransform(Eigen::Affine3f(transform_matrix));
      }

      crop.filter(*cloud_cropped);
      cloud_filtered = cloud_cropped; // 将输出作为下一个过滤器的输入
    }

    // ==========================================================
    // 3. 发布最终过滤结果
    // ==========================================================
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    // 如果采用 transform_cloud 模式，云数据当前在 filter_frame
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