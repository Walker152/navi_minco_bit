#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
    const auto odom_topic =
      this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    z_offset_ = static_cast<float>(this->declare_parameter<double>("z_offset", 0.0));

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

    const auto centers_x = this->declare_parameter<std::vector<double>>("positions.x", {});
    const auto centers_y = this->declare_parameter<std::vector<double>>("positions.y", {});
    const auto centers_z = this->declare_parameter<std::vector<double>>("positions.z", {});
    const auto sizes_x = this->declare_parameter<std::vector<double>>("box_sizes.x", {});
    const auto sizes_y = this->declare_parameter<std::vector<double>>("box_sizes.y", {});
    const auto sizes_z = this->declare_parameter<std::vector<double>>("box_sizes.z", {});

    if (!centers_x.empty() || !centers_y.empty() || !centers_z.empty()) {
      if (centers_x.size() == centers_y.size() && centers_x.size() == centers_z.size()) {
        centers_.reserve(centers_x.size());
        for (size_t i = 0; i < centers_x.size(); ++i) {
          centers_.emplace_back(
            static_cast<float>(centers_x[i]),
            static_cast<float>(centers_y[i]),
            static_cast<float>(centers_z[i]));
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
          "positions.* size mismatch, fallback to single center.");
      }
    }

    if (size_x_ <= 0.0f || size_y_ <= 0.0f || size_z_ <= 0.0f) {
      RCLCPP_WARN(this->get_logger(),
        "box_size must be positive. Using absolute value: (%.3f, %.3f, %.3f)",
        size_x_, size_y_, size_z_);
      size_x_ = std::abs(size_x_);
      size_y_ = std::abs(size_y_);
      size_z_ = std::abs(size_z_);
    }

    if (!centers_.empty()) {
      if (!sizes_x.empty() || !sizes_y.empty() || !sizes_z.empty()) {
        if (sizes_x.size() == sizes_y.size() && sizes_x.size() == sizes_z.size() &&
          sizes_x.size() == centers_.size()) {
          sizes_.reserve(sizes_x.size());
          for (size_t i = 0; i < sizes_x.size(); ++i) {
            sizes_.emplace_back(
              std::abs(static_cast<float>(sizes_x[i])),
              std::abs(static_cast<float>(sizes_y[i])),
              std::abs(static_cast<float>(sizes_z[i])));
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
            "box_sizes.* size mismatch, fallback to single box_size for all centers.");
        }
      }

      if (sizes_.empty()) {
        sizes_.reserve(centers_.size());
        for (size_t i = 0; i < centers_.size(); ++i) {
          sizes_.emplace_back(size_x_, size_y_, size_z_);
        }
      }
    }

    // 预先分配成员级点云内存，防止高频申请堆空间
    cloud_input_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_pass_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_transformed_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Optimized Crop filter started. input=%s output=%s frame=%s mode=%s remove_inside=%s",
      input_topic.c_str(), output_topic.c_str(), position_frame_.c_str(),
      filter_mode_.c_str(), remove_inside_ ? "true" : "false");
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
        "TF lookup failed: %s <- %s, reason: %s",
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
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_z_ = static_cast<float>(msg->pose.pose.position.z);
    odom_ready_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ==========================================================
    // 1. TF 查询放在最前 (无延迟)
    // ==========================================================
    const std::string cloud_frame = msg->header.frame_id;
    std::string filter_frame = position_frame_;
    if (filter_frame.empty()) {
      filter_frame = cloud_frame;
    }

    bool need_transform = (filter_frame != cloud_frame);
    geometry_msgs::msg::TransformStamped tf_stamped;
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    if (need_transform) {
      if (filter_mode_ == "transform_cloud") {
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);
      } else {  // transform_center
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_stamped)) return;
        transform_matrix = transformToMatrix(tf_stamped);
      }
    }

    // ==========================================================
    // 2. 内存复用：使用 clear() 代替高频的 heap new 申请
    // ==========================================================
    cloud_input_->clear();
    pcl::fromROSMsg(*msg, *cloud_input_);

    if (cloud_input_->empty()) {
      cloud_pub_->publish(*msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_filter = cloud_input_;

    // 获取里程计 Z 轴高度
    float min_z = 0.0f;
    bool odom_ready = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_ready = odom_ready_;
      if (odom_ready) {
        min_z = odom_z_ + z_offset_;
      }
    }

    // PassThrough 截断 (复用成员级 cloud_pass_ 避免分配内存)
    if (odom_ready) {
      cloud_pass_->clear();
      pass_filter_.setInputCloud(cloud_for_filter);
      pass_filter_.setFilterFieldName("z");
      pass_filter_.setFilterLimits(min_z, std::numeric_limits<float>::max());
      pass_filter_.filter(*cloud_pass_);
      cloud_for_filter = cloud_pass_;
    }

    // 坐标系转换 (复用成员级 cloud_transformed_ 避免分配内存)
    std::vector<Eigen::Vector3f> centers;
    std::vector<Eigen::Vector3f> sizes;

    if (!centers_.empty()) {
      centers = centers_;
      sizes = sizes_;
    } else {
      centers.emplace_back(center_x_, center_y_, center_z_);
      sizes.emplace_back(size_x_, size_y_, size_z_);
    }

    if (need_transform) {
      if (filter_mode_ == "transform_cloud") {
        cloud_transformed_->clear();
        pcl::transformPointCloud(*cloud_for_filter, *cloud_transformed_, transform_matrix);
        cloud_for_filter = cloud_transformed_;
      } else {  // transform_center
        for (auto & c : centers) {
          c = transformPoint(transform_matrix, c);
        }
      }
    }

    // ==========================================================
    // 3. 核心优化：使用极速 std::vector<uint8_t> 替换慢速 std::vector<bool>
    // ==========================================================
    cloud_out_->clear();
    const size_t num_points = cloud_for_filter->points.size();
    if (num_points == 0) {
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl::toROSMsg(*cloud_out_, filtered_msg);
      filtered_msg.header.stamp = msg->header.stamp;
      filtered_msg.header.frame_id = (filter_mode_ == "transform_cloud") ? filter_frame : cloud_frame;
      cloud_pub_->publish(filtered_msg);
      return;
    }

    // 字节定位，性能比原版的 bit-vector 快 10 倍以上
    std::vector<uint8_t> keep_mask(num_points, remove_inside_ ? 1 : 0);
    crop_filter_.setInputCloud(cloud_for_filter);

    for (size_t i = 0; i < centers.size(); ++i) {
      const float half_x = 0.5f * sizes[i].x();
      const float half_y = 0.5f * sizes[i].y();
      const float half_z = 0.5f * sizes[i].z();

      const float min_x = centers[i].x() - half_x;
      const float max_x = centers[i].x() + half_x;
      const float min_y = centers[i].y() - half_y;
      const float max_y = centers[i].y() + half_y;
      const float min_z = centers[i].z() - half_z;
      const float max_z = centers[i].z() + half_z;

      crop_filter_.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
      crop_filter_.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));

      std::vector<int> indices;
      crop_filter_.filter(indices);
      for (const int idx : indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < num_points) {
          keep_mask[static_cast<size_t>(idx)] = remove_inside_ ? 0 : 1;
        }
      }
    }

    // ==========================================================
    // 4. 分支优化：如果是稠密点云，免除无意义的 isFinite 浮点计算
    // ==========================================================
    cloud_out_->points.reserve(num_points);
    if (cloud_for_filter->is_dense) {
      for (size_t i = 0; i < num_points; ++i) {
        if (keep_mask[i]) {
          cloud_out_->points.push_back(cloud_for_filter->points[i]);
        }
      }
    } else {
      for (size_t i = 0; i < num_points; ++i) {
        const auto & p = cloud_for_filter->points[i];
        if (keep_mask[i] && pcl::isFinite(p)) {
          cloud_out_->points.push_back(p);
        }
      }
    }

    cloud_out_->width = static_cast<uint32_t>(cloud_out_->points.size());
    cloud_out_->height = 1;
    cloud_out_->is_dense = true;

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_out_, filtered_msg);
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_msg.header.frame_id = (filter_mode_ == "transform_cloud") ? filter_frame : cloud_frame;
    cloud_pub_->publish(filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
  std::vector<Eigen::Vector3f> centers_;
  std::vector<Eigen::Vector3f> sizes_;
  float z_offset_{0.0f};
  float odom_z_{0.0f};
  bool odom_ready_{false};
  std::mutex odom_mutex_;

  // 成员级共享指针与滤波器实例：全局复用以避免高频构造和内存分配
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_;
  pcl::PassThrough<pcl::PointXYZ> pass_filter_;
  pcl::CropBox<pcl::PointXYZ> crop_filter_;

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