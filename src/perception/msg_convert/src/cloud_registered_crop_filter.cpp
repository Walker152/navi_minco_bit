#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <atomic>

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
  struct CropRegion
  {
    Eigen::Vector3f center{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f half_size{6.0f, 3.0f, 1.75f};
  };

  CloudRegisteredCropFilterNode() : Node("cloud_registered_crop_filter")
  {
    const auto input_topic = this->declare_parameter<std::string>("input_topic", "/cloud_registered");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/cloud_registered_filtered");
    const auto queue_size = this->declare_parameter<int>("queue_size", 10);

    position_frame_ = this->declare_parameter<std::string>("position_frame", "camera_init");

    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    remove_inside_ = this->declare_parameter<bool>("remove_inside", true);
    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(
        this->get_logger(), "Unknown filter_mode='%s', fallback to transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    z_offset_ = this->declare_parameter<float>("z_offset", -0.10f);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_buffer_->setUsingDedicatedThread(true);

    const auto centers_x = this->declare_parameter<std::vector<double>>("centers_x", std::vector<double>{});
    const auto centers_y = this->declare_parameter<std::vector<double>>("centers_y", std::vector<double>{});
    const auto centers_z = this->declare_parameter<std::vector<double>>("centers_z", std::vector<double>{});
    const auto boundaries_x =
      this->declare_parameter<std::vector<double>>("boundaries_x", std::vector<double>{});
    const auto boundaries_y =
      this->declare_parameter<std::vector<double>>("boundaries_y", std::vector<double>{});
    const auto boundaries_z =
      this->declare_parameter<std::vector<double>>("boundaries_z", std::vector<double>{});

    const std::size_t region_count = std::min(centers_x.size(),
      std::min(centers_y.size(),
        std::min(centers_z.size(),
          std::min(boundaries_x.size(), std::min(boundaries_y.size(), boundaries_z.size())))));

    crop_regions_.reserve(region_count);
    for (std::size_t i = 0; i < region_count; ++i) {
      const float bx = static_cast<float>(boundaries_x[i]);
      const float by = static_cast<float>(boundaries_y[i]);
      const float bz = static_cast<float>(boundaries_z[i]);
      CropRegion region;
      region.center = Eigen::Vector3f(static_cast<float>(centers_x[i]),
        static_cast<float>(centers_y[i]),
        static_cast<float>(centers_z[i]));
      region.half_size = Eigen::Vector3f(std::abs(bx), std::abs(by), std::abs(bz)) * 0.5f;
      crop_regions_.push_back(region);
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init",
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Crop filter started. input=%s output=%s frame=%s mode=%s remove_inside=%s z_offset=%.2f regions=%zu",
      input_topic.c_str(),
      output_topic.c_str(),
      position_frame_.c_str(),
      filter_mode_.c_str(),
      remove_inside_ ? "true" : "false",
      z_offset_,
      crop_regions_.size());
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

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    current_odom_z_.store(static_cast<float>(msg->pose.pose.position.z));
    odom_received_.store(true);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!odom_received_.load()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for odometry...");
      return;
    }

    const float dynamic_min_z = current_odom_z_.load() + z_offset_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_input);

    if (cloud_input->empty()) {
      sensor_msgs::msg::PointCloud2 filtered_msg = *msg;
      cloud_pub_->publish(filtered_msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ>::Ptr pass(new pcl::PassThrough<pcl::PointXYZ>());
    pass->setInputCloud(cloud_input);
    pass->setFilterFieldName("z");
    pass->setFilterLimits(dynamic_min_z, 10000.0f);
    pass->filter(*cloud_z_filtered);

    const std::string cloud_frame = msg->header.frame_id;
    std::string filter_frame = position_frame_;
    if (filter_frame.empty()) {
      filter_frame = cloud_frame;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_filter = cloud_z_filtered;
    std::vector<CropRegion> active_regions = crop_regions_;

    if (filter_mode_ == "transform_cloud") {
      if (filter_frame != cloud_frame) {
        geometry_msgs::msg::TransformStamped tf_cloud_to_filter;
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_cloud_to_filter)) {
          return;
        }

        cloud_for_filter = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        const Eigen::Matrix4f tf_matrix = transformToMatrix(tf_cloud_to_filter);
        pcl::transformPointCloud(*cloud_z_filtered, *cloud_for_filter, tf_matrix);
      }
    } else {  // transform_center
      if (filter_frame != cloud_frame) {
        geometry_msgs::msg::TransformStamped tf_filter_to_cloud;
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_filter_to_cloud)) {
          return;
        }
        const auto tf = transformToMatrix(tf_filter_to_cloud);
        for (auto & region : active_regions) {
          region.center = transformPoint(tf, region.center);
        }
      }
      filter_frame = cloud_frame;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

    if (remove_inside_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = cloud_for_filter;
      for (const auto & region : active_regions) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ>::Ptr crop_box(new pcl::CropBox<pcl::PointXYZ>());
        crop_box->setInputCloud(current_cloud);
        const Eigen::Vector4f min_pt(region.center.x() - region.half_size.x(),
                                     region.center.y() - region.half_size.y(),
                                     region.center.z() - region.half_size.z(), 1.0f);
        const Eigen::Vector4f max_pt(region.center.x() + region.half_size.x(),
                                     region.center.y() + region.half_size.y(),
                                     region.center.z() + region.half_size.z(), 1.0f);
        crop_box->setMin(min_pt);
        crop_box->setMax(max_pt);
        crop_box->setNegative(true);
        crop_box->filter(*box_filtered);
        current_cloud = box_filtered;
      }
      cloud_out = current_cloud;
    } else {
      cloud_out->header = cloud_for_filter->header;
      std::vector<int> combined_indices;
      for (const auto & region : active_regions) {
        pcl::CropBox<pcl::PointXYZ>::Ptr crop_box(new pcl::CropBox<pcl::PointXYZ>());
        crop_box->setInputCloud(cloud_for_filter);
        const Eigen::Vector4f min_pt(region.center.x() - region.half_size.x(),
                                     region.center.y() - region.half_size.y(),
                                     region.center.z() - region.half_size.z(), 1.0f);
        const Eigen::Vector4f max_pt(region.center.x() + region.half_size.x(),
                                     region.center.y() + region.half_size.y(),
                                     region.center.z() + region.half_size.z(), 1.0f);
        crop_box->setMin(min_pt);
        crop_box->setMax(max_pt);
        std::vector<int> indices;
        crop_box->filter(indices);
        combined_indices.insert(combined_indices.end(), indices.begin(), indices.end());
      }
      std::sort(combined_indices.begin(), combined_indices.end());
      combined_indices.erase(std::unique(combined_indices.begin(), combined_indices.end()),
                             combined_indices.end());
      cloud_out->points.reserve(combined_indices.size());
      for (int idx : combined_indices) {
        cloud_out->points.push_back(cloud_for_filter->points[idx]);
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

  std::vector<CropRegion> crop_regions_;
  std::string position_frame_;
  std::string filter_mode_;
  bool remove_inside_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::atomic<float> current_odom_z_{0.0f};
  std::atomic<bool> odom_received_{false};
  float z_offset_;
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
