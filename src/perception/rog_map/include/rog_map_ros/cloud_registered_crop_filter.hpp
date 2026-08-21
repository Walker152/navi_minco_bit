#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map_core/config.hpp>

namespace rog_map {

class CloudRegisteredCropFilter
{
public:
  using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;

  CloudRegisteredCropFilter(const Config & config,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger,
    MarkerPublisher::SharedPtr marker_publisher);

  bool filter(PointCloud & cloud, const std_msgs::msg::Header & header, const Vec3f & robot_position);

private:
  struct Box
  {
    Eigen::Vector3f center{Eigen::Vector3f::Zero()};
    Eigen::Vector3f size{Eigen::Vector3f::Zero()};
  };

  static Eigen::Matrix4f transformToMatrix(const geometry_msgs::msg::TransformStamped & transform);
  static Eigen::Vector3f transformPoint(const Eigen::Matrix4f & transform, const Eigen::Vector3f & point);
  bool lookupTransform(const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    Eigen::Matrix4f & transform) const;
  void prepareBoxesForCloudFrame(const Eigen::Matrix4f & transform, bool transform_centers);
  bool shouldPublishVisualization();
  bool shouldLogStats();
  void publishVisualization(const std_msgs::msg::Header & header,
    const std::string & crop_frame,
    const std::string & cloud_frame,
    const std::vector<Box> & boxes,
    const Vec3f & robot_position,
    float min_z);
  void logStats(size_t raw_points,
    size_t after_z_points,
    size_t output_points,
    const std::string & cloud_frame,
    const std::string & filter_frame,
    bool need_transform,
    float min_z,
    const std::vector<Box> & boxes,
    const std::vector<size_t> & box_hit_counts);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  MarkerPublisher::SharedPtr marker_publisher_;
  std::vector<Box> boxes_;
  std::vector<Box> working_boxes_;
  std::vector<size_t> box_hit_counts_;
  PointCloud filtered_cloud_;
  std::string position_frame_;
  std::string filter_mode_;
  bool remove_inside_{true};
  bool log_stats_{false};
  int stats_log_period_ms_{1000};
  bool publish_visualization_{false};
  int visualization_period_ms_{200};
  float z_plane_size_{12.0F};
  float z_plane_thickness_{0.03F};
  float z_offset_{0.0F};
  int64_t last_stats_log_time_ns_{0};
  int64_t last_visualization_time_ns_{0};
};

}  // namespace rog_map
