#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
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
#include "visualization_msgs/msg/marker_array.hpp"

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
    z_offset_ = static_cast<float>(this->declare_parameter<double>("z_offset", 0.0));

    center_x_ = static_cast<float>(this->declare_parameter<double>("position.x", 0.0));
    center_y_ = static_cast<float>(this->declare_parameter<double>("position.y", 0.0));
    center_z_ = static_cast<float>(this->declare_parameter<double>("position.z", 0.0));
    position_frame_ = this->declare_parameter<std::string>("position_frame", "camera_init");

    filter_mode_ = this->declare_parameter<std::string>("filter_mode", "transform_cloud");
    remove_inside_ = this->declare_parameter<bool>("remove_inside", true);
    log_stats_ = this->declare_parameter<bool>("log_stats", true);
    stats_log_period_ms_ = this->declare_parameter<int>("stats_log_period_ms", 1000);
    publish_visualization_ = this->declare_parameter<bool>("publish_visualization", false);
    visualization_topic_ =
      this->declare_parameter<std::string>("visualization_topic", "/cloud_registered_crop_filter/markers");
    visualization_period_ms_ = this->declare_parameter<int>("visualization_period_ms", 200);
    z_plane_size_ = static_cast<float>(this->declare_parameter<double>("z_plane_size", 12.0));
    z_plane_thickness_ = static_cast<float>(this->declare_parameter<double>("z_plane_thickness", 0.03));
    box_padding_ = static_cast<float>(this->declare_parameter<double>("box_padding", 0.0));
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

    const auto centers_x =
      this->declare_parameter<std::vector<double>>("positions.x", std::vector<double>{});
    const auto centers_y =
      this->declare_parameter<std::vector<double>>("positions.y", std::vector<double>{});
    const auto centers_z =
      this->declare_parameter<std::vector<double>>("positions.z", std::vector<double>{});
    const auto sizes_x = this->declare_parameter<std::vector<double>>("box_sizes.x", std::vector<double>{});
    const auto sizes_y = this->declare_parameter<std::vector<double>>("box_sizes.y", std::vector<double>{});
    const auto sizes_z = this->declare_parameter<std::vector<double>>("box_sizes.z", std::vector<double>{});

    if (!centers_x.empty() || !centers_y.empty() || !centers_z.empty()) {
      if (centers_x.size() == centers_y.size() && centers_x.size() == centers_z.size()) {
        centers_.reserve(centers_x.size());
        for (size_t i = 0; i < centers_x.size(); ++i) {
          centers_.emplace_back(static_cast<float>(centers_x[i]),
            static_cast<float>(centers_y[i]),
            static_cast<float>(centers_z[i]));
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "positions.* size mismatch, fallback to single center.");
      }
    }

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

    if (!centers_.empty()) {
      if (!sizes_x.empty() || !sizes_y.empty() || !sizes_z.empty()) {
        if (sizes_x.size() == sizes_y.size() && sizes_x.size() == sizes_z.size() &&
            sizes_x.size() == centers_.size()) {
          sizes_.reserve(sizes_x.size());
          for (size_t i = 0; i < sizes_x.size(); ++i) {
            sizes_.emplace_back(std::abs(static_cast<float>(sizes_x[i])),
              std::abs(static_cast<float>(sizes_y[i])),
              std::abs(static_cast<float>(sizes_z[i])));
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(), "box_sizes.* size mismatch, fallback to single box_size for all centers.");
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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::QoS(rclcpp::KeepLast(queue_size)),
      std::bind(&CloudRegisteredCropFilterNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Optimized Crop filter started. input=%s output=%s frame=%s mode=%s remove_inside=%s log_stats=%s "
      "visualization=%s",
      input_topic.c_str(),
      output_topic.c_str(),
      position_frame_.c_str(),
      filter_mode_.c_str(),
      remove_inside_ ? "true" : "false",
      log_stats_ ? "true" : "false",
      publish_visualization_ ? visualization_topic_.c_str() : "false");
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

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_x_ = static_cast<float>(msg->pose.pose.position.x);
    odom_y_ = static_cast<float>(msg->pose.pose.position.y);
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
        if (!lookupTransform(filter_frame, cloud_frame, msg->header.stamp, tf_stamped))
          return;
        transform_matrix = transformToMatrix(tf_stamped);
      } else {  // transform_center
        if (!lookupTransform(cloud_frame, filter_frame, msg->header.stamp, tf_stamped))
          return;
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
    const size_t raw_points = cloud_input_->points.size();

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

    double box_padding_param = box_padding_;
    this->get_parameter("box_padding", box_padding_param);
    const float box_padding = std::max(0.0f, static_cast<float>(box_padding_param));
    std::vector<Eigen::Vector3f> effective_sizes = sizes;
    for (auto & size : effective_sizes) {
      size.x() += 2.0f * box_padding;
      size.y() += 2.0f * box_padding;
      size.z() += 2.0f * box_padding;
    }

    const std::string crop_frame = (filter_mode_ == "transform_cloud") ? filter_frame : cloud_frame;
    if (shouldPublishVisualization()) {
      publishVisualization(
        msg->header.stamp, crop_frame, cloud_frame, centers, effective_sizes, odom_ready, min_z);
    }

    const size_t filter_input_points = cloud_for_filter->points.size();
    const bool should_log_stats = shouldLogStats();
    std::vector<size_t> box_hit_counts;
    if (should_log_stats) {
      box_hit_counts.reserve(centers.size());
    }

    // ==========================================================
    // 3. 使用显式 AABB 判断，避免 CropBox 索引路径留下边界残点
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

    std::vector<uint8_t> inside_mask(num_points, 0);
    if (should_log_stats) {
      box_hit_counts.assign(centers.size(), 0);
    }

    for (size_t box_i = 0; box_i < centers.size(); ++box_i) {
      const float half_x = 0.5f * effective_sizes[box_i].x();
      const float half_y = 0.5f * effective_sizes[box_i].y();
      const float half_z = 0.5f * effective_sizes[box_i].z();

      const float box_min_x = centers[box_i].x() - half_x;
      const float box_max_x = centers[box_i].x() + half_x;
      const float box_min_y = centers[box_i].y() - half_y;
      const float box_max_y = centers[box_i].y() + half_y;
      const float box_min_z = centers[box_i].z() - half_z;
      const float box_max_z = centers[box_i].z() + half_z;

      for (size_t point_i = 0; point_i < num_points; ++point_i) {
        const auto & p = cloud_for_filter->points[point_i];
        if (!pcl::isFinite(p)) {
          continue;
        }

        const bool inside = p.x >= box_min_x && p.x <= box_max_x && p.y >= box_min_y && p.y <= box_max_y &&
                            p.z >= box_min_z && p.z <= box_max_z;
        if (inside) {
          inside_mask[point_i] = 1;
          if (should_log_stats) {
            box_hit_counts[box_i]++;
          }
        }
      }
    }

    // ==========================================================
    // 4. 分支优化：如果是稠密点云，免除无意义的 isFinite 浮点计算
    // ==========================================================
    cloud_out_->points.reserve(num_points);
    if (cloud_for_filter->is_dense) {
      for (size_t i = 0; i < num_points; ++i) {
        const bool keep_point = remove_inside_ ? !inside_mask[i] : inside_mask[i];
        if (keep_point) {
          cloud_out_->points.push_back(cloud_for_filter->points[i]);
        }
      }
    } else {
      for (size_t i = 0; i < num_points; ++i) {
        const auto & p = cloud_for_filter->points[i];
        const bool keep_point = remove_inside_ ? !inside_mask[i] : inside_mask[i];
        if (keep_point && pcl::isFinite(p)) {
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

    if (should_log_stats) {
      logFilterStats(raw_points,
        filter_input_points,
        cloud_out_->points.size(),
        cloud_frame,
        filter_frame,
        need_transform,
        odom_ready,
        min_z,
        centers,
        effective_sizes,
        box_hit_counts);
    }
  }

  bool shouldLogStats()
  {
    if (!log_stats_) {
      return false;
    }
    if (stats_log_period_ms_ <= 0) {
      return true;
    }

    const int64_t now_ns = this->now().nanoseconds();
    if (last_stats_log_time_ns_ == 0 ||
        now_ns - last_stats_log_time_ns_ >= static_cast<int64_t>(stats_log_period_ms_) * 1000000LL) {
      last_stats_log_time_ns_ = now_ns;
      return true;
    }
    return false;
  }

  void logFilterStats(const size_t raw_points,
    const size_t filter_input_points,
    const size_t output_points,
    const std::string & cloud_frame,
    const std::string & filter_frame,
    const bool need_transform,
    const bool odom_ready,
    const float min_z,
    const std::vector<Eigen::Vector3f> & centers,
    const std::vector<Eigen::Vector3f> & sizes,
    const std::vector<size_t> & box_hit_counts)
  {
    const size_t changed_points =
      remove_inside_ ? (filter_input_points >= output_points ? filter_input_points - output_points : 0)
                     : output_points;

    std::ostringstream oss;
    oss << "[CropStats] raw=" << raw_points << " after_z=" << filter_input_points
        << " output=" << output_points << " changed=" << changed_points << " mode=" << filter_mode_
        << " cloud_frame=" << cloud_frame << " filter_frame=" << filter_frame
        << " need_tf=" << (need_transform ? "true" : "false") << " odom_z_filter=";
    if (odom_ready) {
      oss << min_z;
    } else {
      oss << "disabled";
    }

    for (size_t i = 0; i < centers.size(); ++i) {
      const float half_x = 0.5f * sizes[i].x();
      const float half_y = 0.5f * sizes[i].y();
      const float half_z = 0.5f * sizes[i].z();
      const size_t hits = i < box_hit_counts.size() ? box_hit_counts[i] : 0;
      oss << " | box" << i << " center=(" << centers[i].x() << "," << centers[i].y() << ","
          << centers[i].z() << ") size=(" << sizes[i].x() << "," << sizes[i].y() << "," << sizes[i].z()
          << ") min=(" << centers[i].x() - half_x << "," << centers[i].y() - half_y << ","
          << centers[i].z() - half_z << ") max=(" << centers[i].x() + half_x << ","
          << centers[i].y() + half_y << "," << centers[i].z() + half_z << ") hits=" << hits;
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  bool shouldPublishVisualization()
  {
    this->get_parameter("publish_visualization", publish_visualization_);
    this->get_parameter("visualization_period_ms", visualization_period_ms_);

    if (!publish_visualization_ || !marker_pub_) {
      if (visualization_was_enabled_ && marker_pub_) {
        publishClearVisualization();
      }
      visualization_was_enabled_ = false;
      return false;
    }
    visualization_was_enabled_ = true;

    if (visualization_period_ms_ <= 0) {
      return true;
    }

    const int64_t now_ns = this->now().nanoseconds();
    if (last_visualization_time_ns_ == 0 || now_ns - last_visualization_time_ns_ >=
                                              static_cast<int64_t>(visualization_period_ms_) * 1000000LL) {
      last_visualization_time_ns_ = now_ns;
      return true;
    }
    return false;
  }

  void publishClearVisualization()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    marker_pub_->publish(markers);
  }

  void publishVisualization(const rclcpp::Time & stamp,
    const std::string & crop_frame,
    const std::string & cloud_frame,
    const std::vector<Eigen::Vector3f> & centers,
    const std::vector<Eigen::Vector3f> & sizes,
    const bool odom_ready,
    const float min_z)
  {
    double z_plane_size = z_plane_size_;
    double z_plane_thickness = z_plane_thickness_;
    this->get_parameter("z_plane_size", z_plane_size);
    this->get_parameter("z_plane_thickness", z_plane_thickness);

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (size_t i = 0; i < centers.size(); ++i) {
      visualization_msgs::msg::Marker box;
      box.header.frame_id = crop_frame;
      box.header.stamp = stamp;
      box.ns = "crop_boxes";
      box.id = static_cast<int>(i);
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = centers[i].x();
      box.pose.position.y = centers[i].y();
      box.pose.position.z = centers[i].z();
      box.pose.orientation.w = 1.0;
      box.scale.x = sizes[i].x();
      box.scale.y = sizes[i].y();
      box.scale.z = sizes[i].z();
      box.color.r = remove_inside_ ? 1.0f : 0.1f;
      box.color.g = remove_inside_ ? 0.15f : 0.8f;
      box.color.b = 0.05f;
      box.color.a = 0.22f;
      markers.markers.push_back(box);
    }

    if (odom_ready) {
      float odom_x = 0.0f;
      float odom_y = 0.0f;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x = odom_x_;
        odom_y = odom_y_;
      }

      visualization_msgs::msg::Marker z_plane;
      z_plane.header.frame_id = cloud_frame;
      z_plane.header.stamp = stamp;
      z_plane.ns = "z_pass_plane";
      z_plane.id = 1000;
      z_plane.type = visualization_msgs::msg::Marker::CUBE;
      z_plane.action = visualization_msgs::msg::Marker::ADD;
      z_plane.pose.position.x = odom_x;
      z_plane.pose.position.y = odom_y;
      z_plane.pose.position.z = min_z;
      z_plane.pose.orientation.w = 1.0;
      z_plane.scale.x = z_plane_size;
      z_plane.scale.y = z_plane_size;
      z_plane.scale.z = z_plane_thickness;
      z_plane.color.r = 0.1f;
      z_plane.color.g = 0.45f;
      z_plane.color.b = 1.0f;
      z_plane.color.a = 0.18f;
      markers.markers.push_back(z_plane);
    }

    marker_pub_->publish(markers);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  float center_x_{0.0f};
  float center_y_{0.0f};
  float center_z_{0.0f};
  std::string position_frame_;
  std::string filter_mode_;
  bool remove_inside_{true};
  bool log_stats_{true};
  int stats_log_period_ms_{1000};
  int64_t last_stats_log_time_ns_{0};
  bool publish_visualization_{false};
  bool visualization_was_enabled_{false};
  std::string visualization_topic_;
  int visualization_period_ms_{200};
  int64_t last_visualization_time_ns_{0};
  float z_plane_size_{12.0f};
  float z_plane_thickness_{0.03f};
  float box_padding_{0.0f};
  float size_x_{12.0f};
  float size_y_{6.0f};
  float size_z_{3.5f};
  std::vector<Eigen::Vector3f> centers_;
  std::vector<Eigen::Vector3f> sizes_;
  float z_offset_{0.0f};
  float odom_x_{0.0f};
  float odom_y_{0.0f};
  float odom_z_{0.0f};
  bool odom_ready_{false};
  std::mutex odom_mutex_;

  // 成员级共享指针与滤波器实例：全局复用以避免高频构造和内存分配
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_;
  pcl::PassThrough<pcl::PointXYZ> pass_filter_;

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
