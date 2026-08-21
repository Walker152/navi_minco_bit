#include <rog_map_ros/cloud_registered_crop_filter.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <pcl/common/point_tests.h>
#include <tf2/exceptions.h>

namespace rog_map {

CloudRegisteredCropFilter::CloudRegisteredCropFilter(const Config & config,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Logger logger,
  MarkerPublisher::SharedPtr marker_publisher)
: tf_buffer_(std::move(tf_buffer)), clock_(std::move(clock)), logger_(std::move(logger)),
  marker_publisher_(std::move(marker_publisher)), position_frame_(config.cloud_filter_position_frame),
  filter_mode_(config.cloud_filter_mode), remove_inside_(config.cloud_filter_remove_inside),
  log_stats_(config.cloud_filter_log_stats), stats_log_period_ms_(config.cloud_filter_stats_log_period_ms),
  publish_visualization_(config.cloud_filter_publish_visualization),
  visualization_period_ms_(config.cloud_filter_visualization_period_ms),
  z_plane_size_(static_cast<float>(config.cloud_filter_z_plane_size)),
  z_plane_thickness_(static_cast<float>(config.cloud_filter_z_plane_thickness)),
  z_offset_(static_cast<float>(config.cloud_filter_z_offset))
{
  const Eigen::Vector3f default_center(static_cast<float>(config.cloud_filter_center_x),
    static_cast<float>(config.cloud_filter_center_y),
    static_cast<float>(config.cloud_filter_center_z));
  const Eigen::Vector3f default_size(std::abs(static_cast<float>(config.cloud_filter_size_x)),
    std::abs(static_cast<float>(config.cloud_filter_size_y)),
    std::abs(static_cast<float>(config.cloud_filter_size_z)));
  const float padding = std::max(0.0F, static_cast<float>(config.cloud_filter_box_padding));

  const auto & centers_x = config.cloud_filter_centers_x;
  const auto & centers_y = config.cloud_filter_centers_y;
  const auto & centers_z = config.cloud_filter_centers_z;
  const bool has_center_arrays = !centers_x.empty() || !centers_y.empty() || !centers_z.empty();
  const bool valid_center_arrays =
    !centers_x.empty() && centers_x.size() == centers_y.size() && centers_x.size() == centers_z.size();

  if (has_center_arrays && !valid_center_arrays) {
    RCLCPP_WARN(
      logger_, "[ROGMap CloudFilter] cloud_filter.positions.* size mismatch, using the single position.");
  }

  if (valid_center_arrays) {
    boxes_.reserve(centers_x.size());
    for (size_t i = 0; i < centers_x.size(); ++i) {
      Box box;
      box.center = Eigen::Vector3f(static_cast<float>(centers_x[i]),
        static_cast<float>(centers_y[i]),
        static_cast<float>(centers_z[i]));
      box.size = default_size;
      boxes_.push_back(box);
    }
  } else {
    boxes_.push_back(Box{default_center, default_size});
  }

  const auto & sizes_x = config.cloud_filter_sizes_x;
  const auto & sizes_y = config.cloud_filter_sizes_y;
  const auto & sizes_z = config.cloud_filter_sizes_z;
  const bool has_size_arrays = !sizes_x.empty() || !sizes_y.empty() || !sizes_z.empty();
  const bool valid_size_arrays = valid_center_arrays && sizes_x.size() == boxes_.size() &&
                                 sizes_x.size() == sizes_y.size() && sizes_x.size() == sizes_z.size();
  if (has_size_arrays && !valid_size_arrays) {
    RCLCPP_WARN(logger_,
      "[ROGMap CloudFilter] cloud_filter.box_sizes.* size mismatch, using cloud_filter.box_size for all "
      "boxes.");
  }
  if (valid_size_arrays) {
    for (size_t i = 0; i < boxes_.size(); ++i) {
      boxes_[i].size = Eigen::Vector3f(std::abs(static_cast<float>(sizes_x[i])),
        std::abs(static_cast<float>(sizes_y[i])),
        std::abs(static_cast<float>(sizes_z[i])));
    }
  }
  for (auto & box : boxes_) {
    box.size.array() += 2.0F * padding;
  }

  RCLCPP_INFO(logger_,
    "[ROGMap CloudFilter] enabled frame=%s mode=%s remove_inside=%s boxes=%zu visualization=%s",
    position_frame_.c_str(),
    filter_mode_.c_str(),
    remove_inside_ ? "true" : "false",
    boxes_.size(),
    publish_visualization_ ? "true" : "false");
}

Eigen::Matrix4f CloudRegisteredCropFilter::transformToMatrix(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  const Eigen::Quaternionf quaternion(static_cast<float>(rotation.w),
    static_cast<float>(rotation.x),
    static_cast<float>(rotation.y),
    static_cast<float>(rotation.z));
  matrix.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  matrix(0, 3) = static_cast<float>(translation.x);
  matrix(1, 3) = static_cast<float>(translation.y);
  matrix(2, 3) = static_cast<float>(translation.z);
  return matrix;
}

Eigen::Vector3f CloudRegisteredCropFilter::transformPoint(
  const Eigen::Matrix4f & transform, const Eigen::Vector3f & point)
{
  return (transform * Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0F)).head<3>();
}

bool CloudRegisteredCropFilter::lookupTransform(const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  Eigen::Matrix4f & transform) const
{
  if (!tf_buffer_) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[ROGMap CloudFilter] TF buffer is unavailable for %s <- %s",
      target_frame.c_str(),
      source_frame.c_str());
    return false;
  }
  try {
    transform = transformToMatrix(
      tf_buffer_->lookupTransform(target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.05)));
    return true;
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[ROGMap CloudFilter] TF lookup failed for %s <- %s: %s",
      target_frame.c_str(),
      source_frame.c_str(),
      error.what());
    return false;
  }
}

void CloudRegisteredCropFilter::prepareBoxesForCloudFrame(
  const Eigen::Matrix4f & transform, bool transform_centers)
{
  working_boxes_ = boxes_;
  if (transform_centers) {
    for (auto & box : working_boxes_) {
      box.center = transformPoint(transform, box.center);
    }
  }
}

bool CloudRegisteredCropFilter::filter(
  PointCloud & cloud, const std_msgs::msg::Header & header, const Vec3f & robot_position)
{
  const std::string cloud_frame = header.frame_id.empty() ? position_frame_ : header.frame_id;
  const std::string filter_frame = position_frame_.empty() ? cloud_frame : position_frame_;
  const bool need_transform = filter_frame != cloud_frame;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  if (need_transform) {
    const bool transform_centers = filter_mode_ == "transform_center";
    if (!lookupTransform(transform_centers ? cloud_frame : filter_frame,
          transform_centers ? filter_frame : cloud_frame,
          rclcpp::Time(header.stamp),
          transform)) {
      return false;
    }
  }

  const bool transform_centers = need_transform && filter_mode_ == "transform_center";
  prepareBoxesForCloudFrame(transform, transform_centers);
  const std::string crop_frame = filter_mode_ == "transform_cloud" ? filter_frame : cloud_frame;
  const float min_z = static_cast<float>(robot_position.z()) + z_offset_;
  const size_t raw_points = cloud.points.size();
  size_t after_z_points = 0;
  box_hit_counts_.assign(working_boxes_.size(), 0U);

  filtered_cloud_.clear();
  filtered_cloud_.points.reserve(cloud.points.size());
  for (const auto & point : cloud.points) {
    if (!pcl::isFinite(point) || point.z < min_z) {
      continue;
    }
    ++after_z_points;

    Eigen::Vector3f test_point(point.x, point.y, point.z);
    if (need_transform && filter_mode_ == "transform_cloud") {
      test_point = transformPoint(transform, test_point);
    }

    bool inside_any = false;
    for (size_t i = 0; i < working_boxes_.size(); ++i) {
      const Eigen::Vector3f half_size = 0.5F * working_boxes_[i].size;
      const bool inside = (test_point.array() >= (working_boxes_[i].center - half_size).array()).all() &&
                          (test_point.array() <= (working_boxes_[i].center + half_size).array()).all();
      if (inside) {
        inside_any = true;
        ++box_hit_counts_[i];
      }
    }
    if (remove_inside_ ? !inside_any : inside_any) {
      filtered_cloud_.points.push_back(point);
    }
  }

  cloud.points.swap(filtered_cloud_.points);
  cloud.width = static_cast<uint32_t>(cloud.points.size());
  cloud.height = 1U;
  cloud.is_dense = true;

  if (shouldPublishVisualization()) {
    publishVisualization(header, crop_frame, cloud_frame, working_boxes_, robot_position, min_z);
  }
  if (shouldLogStats()) {
    logStats(raw_points,
      after_z_points,
      cloud.points.size(),
      cloud_frame,
      filter_frame,
      need_transform,
      min_z,
      working_boxes_,
      box_hit_counts_);
  }
  return true;
}

bool CloudRegisteredCropFilter::shouldPublishVisualization()
{
  if (!publish_visualization_ || !marker_publisher_) {
    return false;
  }
  if (visualization_period_ms_ <= 0) {
    return true;
  }
  const int64_t now_ns = clock_->now().nanoseconds();
  if (last_visualization_time_ns_ == 0 ||
      now_ns - last_visualization_time_ns_ >= static_cast<int64_t>(visualization_period_ms_) * 1000000LL) {
    last_visualization_time_ns_ = now_ns;
    return true;
  }
  return false;
}

bool CloudRegisteredCropFilter::shouldLogStats()
{
  if (!log_stats_) {
    return false;
  }
  if (stats_log_period_ms_ <= 0) {
    return true;
  }
  const int64_t now_ns = clock_->now().nanoseconds();
  if (last_stats_log_time_ns_ == 0 ||
      now_ns - last_stats_log_time_ns_ >= static_cast<int64_t>(stats_log_period_ms_) * 1000000LL) {
    last_stats_log_time_ns_ = now_ns;
    return true;
  }
  return false;
}

void CloudRegisteredCropFilter::publishVisualization(const std_msgs::msg::Header & header,
  const std::string & crop_frame,
  const std::string & cloud_frame,
  const std::vector<Box> & boxes,
  const Vec3f & robot_position,
  float min_z)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (size_t i = 0; i < boxes.size(); ++i) {
    visualization_msgs::msg::Marker box;
    box.header.frame_id = crop_frame;
    box.header.stamp = header.stamp;
    box.ns = "crop_boxes";
    box.id = static_cast<int>(i);
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = boxes[i].center.x();
    box.pose.position.y = boxes[i].center.y();
    box.pose.position.z = boxes[i].center.z();
    box.pose.orientation.w = 1.0;
    box.scale.x = boxes[i].size.x();
    box.scale.y = boxes[i].size.y();
    box.scale.z = boxes[i].size.z();
    box.color.r = remove_inside_ ? 1.0F : 0.1F;
    box.color.g = remove_inside_ ? 0.15F : 0.8F;
    box.color.b = 0.05F;
    box.color.a = 0.22F;
    markers.markers.push_back(box);
  }

  visualization_msgs::msg::Marker z_plane;
  z_plane.header.frame_id = cloud_frame;
  z_plane.header.stamp = header.stamp;
  z_plane.ns = "z_pass_plane";
  z_plane.id = 1000;
  z_plane.type = visualization_msgs::msg::Marker::CUBE;
  z_plane.action = visualization_msgs::msg::Marker::ADD;
  z_plane.pose.position.x = robot_position.x();
  z_plane.pose.position.y = robot_position.y();
  z_plane.pose.position.z = min_z;
  z_plane.pose.orientation.w = 1.0;
  z_plane.scale.x = z_plane_size_;
  z_plane.scale.y = z_plane_size_;
  z_plane.scale.z = z_plane_thickness_;
  z_plane.color.r = 0.1F;
  z_plane.color.g = 0.45F;
  z_plane.color.b = 1.0F;
  z_plane.color.a = 0.18F;
  markers.markers.push_back(z_plane);
  marker_publisher_->publish(markers);
}

void CloudRegisteredCropFilter::logStats(size_t raw_points,
  size_t after_z_points,
  size_t output_points,
  const std::string & cloud_frame,
  const std::string & filter_frame,
  bool need_transform,
  float min_z,
  const std::vector<Box> & boxes,
  const std::vector<size_t> & box_hit_counts)
{
  const size_t changed_points = remove_inside_
                                  ? (after_z_points >= output_points ? after_z_points - output_points : 0U)
                                  : output_points;
  std::ostringstream stream;
  stream << "[ROGMap CloudFilter] raw=" << raw_points << " after_z=" << after_z_points
         << " output=" << output_points << " changed=" << changed_points << " mode=" << filter_mode_
         << " cloud_frame=" << cloud_frame << " filter_frame=" << filter_frame
         << " need_tf=" << (need_transform ? "true" : "false") << " min_z=" << min_z;
  for (size_t i = 0; i < boxes.size(); ++i) {
    const Eigen::Vector3f half_size = 0.5F * boxes[i].size;
    stream << " | box" << i << " center=(" << boxes[i].center.x() << "," << boxes[i].center.y() << ","
           << boxes[i].center.z() << ") size=(" << boxes[i].size.x() << "," << boxes[i].size.y() << ","
           << boxes[i].size.z() << ") min=(" << boxes[i].center.x() - half_size.x() << ","
           << boxes[i].center.y() - half_size.y() << "," << boxes[i].center.z() - half_size.z() << ") max=("
           << boxes[i].center.x() + half_size.x() << "," << boxes[i].center.y() + half_size.y() << ","
           << boxes[i].center.z() + half_size.z() << ") hits=" << box_hit_counts[i];
  }
  RCLCPP_INFO(logger_, "%s", stream.str().c_str());
}

}  // namespace rog_map
