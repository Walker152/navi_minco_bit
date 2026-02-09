#include "minco_core/visualizer.hpp"

// C++ standard library
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>

// ROS 2
#include "sensor_msgs/msg/point_field.hpp"

namespace minco_planner {

void Visualizer::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & global_frame,
  const small_rog_map::HybridESDFMap::Ptr & esdf_map,
  bool enable_esdf_timer)
{
  node_ = parent;
  global_frame_ = global_frame;
  esdf_map_ = esdf_map;

  auto node = parent.lock();
  if (!node) {
    return;
  }

  // --- Visualization publishers ---------------------------------------------

  // Backup
  backup_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/backup_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Optimized
  opt_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/opt_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  // A* guide path
  astar_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/astar_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Markers
  control_points_vis_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "/minco_control_points_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // ESDF cloud
  esdf_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/esdf_cloud", 10);

  // 15Hz visualization timer
  visual_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(66),
    std::bind(&Visualizer::visualTimerCallback, this));

  // 1Hz ESDF timer (only if enabled)
  if (enable_esdf_timer) {
    esdf_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {
        auto node_ptr = node_.lock();
        if (!node_ptr) {
          return;
        }
        if (esdf_cloud_pub_ && esdf_cloud_pub_->get_subscription_count() > 0) {
          std_msgs::msg::Header header;
          header.stamp = node_ptr->now();
          header.frame_id = global_frame_;
          publishEsdfCloud(header);
        }
      });
  }
}

void Visualizer::cleanup()
{
  visual_timer_.reset();
  esdf_timer_.reset();

  backup_path_vis_pub_.reset();
  opt_path_vis_pub_.reset();
  astar_path_vis_pub_.reset();
  control_points_vis_pub_.reset();
  esdf_cloud_pub_.reset();

  esdf_map_.reset();

  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_control_points_.clear();
  vis_astar_path_ = nav_msgs::msg::Path();
  vis_opt_time_ = -1.0;
  has_vis_opt_traj_ = false;
  has_vis_backup_traj_ = false;
}

void Visualizer::update(
  const std::vector<Eigen::Vector3d> & control_points,
  const traj_opt::Trajectory & backup_traj,
  const traj_opt::Trajectory & opt_traj,
  double opt_time_seconds,
  const nav_msgs::msg::Path & astar_path)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_control_points_ = control_points;

  vis_backup_traj_ = backup_traj;
  has_vis_backup_traj_ = (backup_traj.getTotalDuration() > 1e-3);

  vis_opt_traj_ = opt_traj;
  vis_opt_time_ = opt_time_seconds;
  has_vis_opt_traj_ = (opt_traj.getTotalDuration() > 1e-3);

  vis_astar_path_ = astar_path;
}

void Visualizer::visualTimerCallback()
{
  std_msgs::msg::Header header;
  auto node = node_.lock();
  if (node) {
    header.stamp = node->now();
    header.frame_id = global_frame_;
  } else {
    return;
  }

  std::lock_guard<std::mutex> lock(vis_mutex_);

  // 1. A* Path
  if (astar_path_vis_pub_ && !vis_astar_path_.poses.empty()) {
    vis_astar_path_.header = header;
    for (auto & p : vis_astar_path_.poses) {
      p.header = header;
    }
    astar_path_vis_pub_->publish(vis_astar_path_);
  }

  // 2. Control Points
  if (control_points_vis_pub_ && !vis_control_points_.empty()) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "minco_control_points";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.25;
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;
    mk.color.r = 1.0f;
    mk.color.g = 0.55f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    mk.points.reserve(vis_control_points_.size());
    for (const auto & p : vis_control_points_) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = 0.05;
      mk.points.push_back(pt);
    }
    control_points_vis_pub_->publish(mk);
  }

  // 3. Backup Path
  if (backup_path_vis_pub_ && has_vis_backup_traj_ && vis_backup_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_backup_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_backup_traj_, header, steps, t_step);
    backup_path_vis_pub_->publish(path_msg);
  }

  // 4. Optimized Path
  if (opt_path_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_opt_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_opt_traj_, header, steps, t_step);
    opt_path_vis_pub_->publish(path_msg);
  }

  // 5. Opt time text
  if (control_points_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3 && vis_opt_time_ > 0.0) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "opt_time";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mk.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d start_pos = vis_opt_traj_.getPos(0.0);
    mk.pose.position.x = start_pos(0);
    mk.pose.position.y = start_pos(1);
    mk.pose.position.z = 1.0;
    mk.pose.orientation.w = 1.0;
    mk.scale.z = 0.3;
    mk.color.r = 1.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << (vis_opt_time_ * 1000.0) << " ms";
    mk.text = ss.str();
    control_points_vis_pub_->publish(mk);
  }
}

nav_msgs::msg::Path Visualizer::convertTrajectoryToPath(
  const traj_opt::Trajectory & traj,
  const std_msgs::msg::Header & header,
  int steps,
  double t_step) const
{
  nav_msgs::msg::Path path_msg;
  path_msg.header = header;

  if (steps <= 0 || t_step <= 0.0) {
    return path_msg;
  }

  path_msg.poses.resize(static_cast<size_t>(steps));

  const double total_duration = traj.getTotalDuration();
  for (int i = 0; i < steps; ++i) {
    double t = i * t_step;
    if (t > total_duration) {
      t = total_duration;
    }

    Eigen::Vector3d pos = traj.getPos(t);
    Eigen::Vector3d vel = traj.getVel(t);

    pos.z() = 0.0;
    vel.z() = 0.0;

    double yaw = 0.0;
    if (vel.head<2>().norm() > 1e-4) {
      yaw = std::atan2(vel(1), vel(0));
    } else if (i > 0) {
      const auto & last_q = path_msg.poses[static_cast<size_t>(i - 1)].pose.orientation;
      yaw = 2.0 * std::atan2(last_q.z, last_q.w);
    }

    auto & pose = path_msg.poses[static_cast<size_t>(i)];
    pose.header = header;
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
  }

  return path_msg;
}

void Visualizer::publishEsdfCloud(const std_msgs::msg::Header & header)
{
  if (!esdf_cloud_pub_ || !esdf_map_) {
    return;
  }

  // Visualize fused ESDF (static + dynamic). We sample on the dynamic layer grid if available,
  // otherwise fall back to the static layer grid.
  int width = 0;
  int height = 0;
  double res = 0.0;
  Eigen::Vector2d origin(0.0, 0.0);

  const auto dynamic_layer = esdf_map_->dynamicLayer();
  if (dynamic_layer && dynamic_layer->isValid()) {
    width = dynamic_layer->width();
    height = dynamic_layer->height();
    res = dynamic_layer->resolution();
    origin = dynamic_layer->origin();
  } else {
    const auto static_layer = esdf_map_->staticLayer();
    if (!static_layer || !static_layer->isValid()) {
      return;
    }
    width = static_layer->width();
    height = static_layer->height();
    res = static_layer->resolution();
    origin = static_layer->origin();
  }

  if (width <= 0 || height <= 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(static_cast<size_t>(width) * static_cast<size_t>(height));
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  cloud.fields.resize(4);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;

  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height);

  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      const float x = static_cast<float>(origin.x() + ix * res);
      const float y = static_cast<float>(origin.y() + iy * res);
      const float z = 0.0f;
      double dist = 0.0;
      Eigen::Vector3d grad;
      esdf_map_->evaluate(
        Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), 0.0), dist, grad);
      if (!std::isfinite(dist)) {
        dist = 0.0;
      }
      const float intensity = static_cast<float>(dist);

      const size_t point_index =
        static_cast<size_t>(iy) * static_cast<size_t>(width) + static_cast<size_t>(ix);
      const size_t base = point_index * static_cast<size_t>(cloud.point_step);
      std::memcpy(&cloud.data[base + 0], &x, sizeof(float));
      std::memcpy(&cloud.data[base + 4], &y, sizeof(float));
      std::memcpy(&cloud.data[base + 8], &z, sizeof(float));
      std::memcpy(&cloud.data[base + 12], &intensity, sizeof(float));
    }
  }

  esdf_cloud_pub_->publish(cloud);
}

}  // namespace minco_planner
