#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nav_msgs/msg/odometry.hpp"
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
    const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");

    position_frame_ = this->declare_parameter<std::string>("position_frame", "map");

    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    remove_inside_ = this->declare_parameter<bool>("remove_inside", true);
    passthrough_enabled_ = this->declare_parameter<bool>("passthrough.enabled", true);
    passthrough_offset_ = static_cast<float>(this->declare_parameter<double>("passthrough.z_offset", 0.0));
    if (filter_mode_ != "transform_cloud" && filter_mode_ != "transform_center") {
      RCLCPP_WARN(
        this->get_logger(), "Unknown filter_mode='%s', fallback to transform_cloud", filter_mode_.c_str());
      filter_mode_ = "transform_cloud";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_buffer_->setUsingDedicatedThread(true);

    const auto centers_x = this->declare_parameter<std::vector<double>>("regions.center_x", {});
    const auto centers_y = this->declare_parameter<std::vector<double>>("regions.center_y", {});
    const auto centers_z = this->declare_parameter<std::vector<double>>("regions.center_z", {});
    const auto bounds_x = this->declare_parameter<std::vector<double>>("regions.bound_x", {});
    const auto bounds_y = this->declare_parameter<std::vector<double>>("regions.bound_y", {});
    const auto bounds_z = this->declare_parameter<std::vector<double>>("regions.bound_z", {});
    loadRegions(centers_x, centers_y, centers_z, bounds_x, bounds_y, bounds_z);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::cloudCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(queue_size)));

    RCLCPP_INFO(this->get_logger(),
        "Crop filter started. input=%s output=%s mode=%s remove_inside=%s regions=%zu frame=%s "
        "passthrough=%s odom=%s z_offset=%.3f",
      input_topic.c_str(),
      output_topic.c_str(),
      filter_mode_.c_str(),
      remove_inside_ ? "true" : "false",
      regions_.size(),
      position_frame_.c_str(),
      passthrough_enabled_ ? "true" : "false",
      odom_topic.c_str(),
      passthrough_offset_);
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

  struct Region
  {
    Eigen::Vector3f center;
    Eigen::Vector3f size;
    std::string frame;
  };

  struct RegionBounds
  {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
  };

  void loadRegions(const std::vector<double> & centers_x,
    const std::vector<double> & centers_y,
    const std::vector<double> & centers_z,
    const std::vector<double> & bounds_x,
    const std::vector<double> & bounds_y,
    const std::vector<double> & bounds_z)
  {
    regions_.clear();
    if (centers_x.empty() || centers_y.empty() || centers_z.empty() ||
      bounds_x.empty() || bounds_y.empty() || bounds_z.empty()) {
      RCLCPP_WARN(this->get_logger(), "Region parameters are empty. No filtering will be applied.");
      return;
    }

    const size_t region_count = std::min({
      centers_x.size(),
      centers_y.size(),
      centers_z.size(),
      bounds_x.size(),
      bounds_y.size(),
      bounds_z.size()});
    if (region_count == 0) {
      RCLCPP_WARN(this->get_logger(), "Region parameters are empty. No filtering will be applied.");
      return;
    }

    regions_.reserve(region_count);
    for (size_t i = 0; i < region_count; ++i) {
      Region region;
      region.center = Eigen::Vector3f(
        static_cast<float>(centers_x[i]),
        static_cast<float>(centers_y[i]),
        static_cast<float>(centers_z[i]));
      region.size = Eigen::Vector3f(
        static_cast<float>(bounds_x[i]),
        static_cast<float>(bounds_y[i]),
        static_cast<float>(bounds_z[i]));
      if (region.size.x() <= 0.0f || region.size.y() <= 0.0f || region.size.z() <= 0.0f) {
        RCLCPP_WARN(this->get_logger(),
          "Region %zu box_size must be positive. Using absolute value: (%.3f, %.3f, %.3f)",
          i,
          region.size.x(),
          region.size.y(),
          region.size.z());
        region.size = region.size.cwiseAbs();
      }
      region.frame = position_frame_;
      regions_.push_back(region);
    }
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

    if (regions_.empty()) {
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
    std::string effective_mode = filter_mode_;
    const std::string common_frame = position_frame_.empty() ? cloud_frame : position_frame_;

    if (effective_mode == "transform_cloud") {
      filter_frame = common_frame.empty() ? cloud_frame : common_frame;
      if (filter_frame != cloud_frame) {
        geometry_msgs::msg::TransformStamped tf_cloud_to_filter;
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_cloud_to_filter)) {
          return;
        }

        cloud_for_filter = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_input, *cloud_for_filter, transformToMatrix(tf_cloud_to_filter));
      }
    } else {  // transform_center
      filter_frame = cloud_frame;
    }

    if (passthrough_enabled_) {
      if (!have_odom_z_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Passthrough enabled but no odom received yet. Skipping passthrough.");
      } else {
        const float max_z = last_odom_z_ + passthrough_offset_;
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_for_filter);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::max(), max_z);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>());
        pass.filter(*cloud_pass);
        cloud_for_filter = cloud_pass;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<RegionBounds> bounds;
    bounds.reserve(regions_.size());
    for (const auto & region : regions_) {
      Eigen::Vector3f center = region.center;
      if (effective_mode != "transform_cloud") {
        const std::string region_frame = region.frame.empty() ? cloud_frame : region.frame;
        if (region_frame != cloud_frame) {
          geometry_msgs::msg::TransformStamped tf_region_to_cloud;
          if (!lookupTransform(cloud_frame, region_frame, msg->header.stamp, tf_region_to_cloud)) {
            return;
          }
          center = transformPoint(transformToMatrix(tf_region_to_cloud), center);
        }
      }

      const float half_x = 0.5f * region.size.x();
      const float half_y = 0.5f * region.size.y();
      const float half_z = 0.5f * region.size.z();
      bounds.push_back(RegionBounds{
        center.x() - half_x,
        center.x() + half_x,
        center.y() - half_y,
        center.y() + half_y,
        center.z() - half_z,
        center.z() + half_z});
    }

    cloud_out->points.reserve(cloud_for_filter->points.size());
    for (const auto & p : cloud_for_filter->points) {
      if (!pcl::isFinite(p)) {
        continue;
      }

      bool inside_any = false;
      for (const auto & region : bounds) {
        if ((p.x >= region.min_x && p.x <= region.max_x) &&
          (p.y >= region.min_y && p.y <= region.max_y) &&
          (p.z >= region.min_z && p.z <= region.max_z)) {
          inside_any = true;
          break;
        }
      }

      const bool keep = remove_inside_ ? !inside_any : inside_any;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string position_frame_;
  std::string filter_mode_;
  bool remove_inside_{true};
  bool passthrough_enabled_{true};
  float passthrough_offset_{0.0f};
  float last_odom_z_{0.0f};
  bool have_odom_z_{false};
  std::vector<Region> regions_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_z_ = static_cast<float>(msg->pose.pose.position.z);
    have_odom_z_ = true;
  }
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
