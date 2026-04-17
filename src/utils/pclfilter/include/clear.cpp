#include "../include/clear.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace pclfilter {
using Polygon2D = std::vector<Point2D>;

clear::clear() : Node("clear_node")
{
  if_clear_ = false;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// main
void clear::init_basemap()
{
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 10, std::bind(&clear::basemapCloudCB, this, std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 10);

  if (!loadcubeParam("cube")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load cube yaml");
  }
}

void clear::basemapCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
{
  std::string source_frame = input_msg->header.frame_id;
  RCLCPP_INFO(this->get_logger(), "sourceID:%s", source_frame.c_str());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *cloud_in);

  // 安全检查
  if (cloud_in->empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty cloud after filtering!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Input cloud size: %zu points", cloud_in->size());

  // 直接在map坐标系中进行过滤（不需要转换到camera_init）
  for (size_t i = 0; i < min_pts_.size(); ++i) {
    size_t before_size = cloud_in->size();
    once_filter(cloud_in, min_pts_[i], max_pts_[i]);
    size_t after_size = cloud_in->size();
    RCLCPP_INFO(this->get_logger(),
      "Filter %zu: from %zu to %zu points (removed %zu)",
      i,
      before_size,
      after_size,
      before_size - after_size);
  }

  RCLCPP_INFO(this->get_logger(), "Output cloud size: %zu points", cloud_in->size());

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*cloud_in, filtered_msg);
  filtered_msg.header.frame_id = source_frame;  // 保持原frame_id（map）
  filtered_msg.header.stamp = input_msg->header.stamp;

  // 直接发布，无需转换
  cloud_pub_->publish(filtered_msg);
}

inline void printZCoordinates(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in)
{
  for (size_t i = 0; i < cloud_in->points.size(); ++i) {
    float z = cloud_in->points[i].z;
    std::cout << "Point " << i << " z: " << z << std::endl;
  }
}

void clear::once_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
  const Eigen::Vector4f & min_pt,
  const Eigen::Vector4f & max_pt)
{
  if (cloud_in->empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty cloud passed to filter");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_in);
  crop_box_filter.setMin(min_pt);
  printZCoordinates(cloud_in);
  crop_box_filter.setMax(max_pt);
  crop_box_filter.setNegative(true);  // Remove points inside the box

  crop_box_filter.filter(*cloud_filtered);
  *cloud_in = *cloud_filtered;
}

void clear::init_odom()
{
  basemap_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 10, std::bind(&clear::cloudCB, this, std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 10);

  if (!loadparam("polygons")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load polygons yaml");
  }
}

void clear::init_mapwithodom()
{
  if_clear_ = false;
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 10, std::bind(&clear::combineCloudCB, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&clear::odomCB, this, std::placeholders::_1));
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 10);

  if (!loadparam("polygons")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load polygons yaml");
  }
}

void clear::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  x_ = odom_msg->pose.pose.position.x;
  y_ = odom_msg->pose.pose.position.y;
  if_clear_ = is_non_area(x_, y_);
}

void clear::cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
{
  std::string source_frame = input_msg->header.frame_id;
  std::string target_frame = "base_link";

  if (if_clear_) {
    // 构造空点云
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header = input_msg->header;
    empty_cloud.header.frame_id = target_frame;
    empty_cloud.height = 0;
    empty_cloud.width = 0;
    empty_cloud.is_dense = false;
    empty_cloud.is_bigendian = false;
    empty_cloud.fields.clear();
    empty_cloud.data.clear();

    cloud_pub_->publish(empty_cloud);
    RCLCPP_INFO(this->get_logger(), "Clear!");
  } else {
    try {
      if (!tf_buffer_->canTransform(
            target_frame, source_frame, tf2::TimePointZero, std::chrono::seconds(1))) {
        RCLCPP_WARN(
          this->get_logger(), "Cannot transform from %s to %s", source_frame.c_str(), target_frame.c_str());
        return;
      }

      sensor_msgs::msg::PointCloud2 cloud_in_base_link;
      tf2::doTransform(*input_msg,
        cloud_in_base_link,
        tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero));
      cloud_pub_->publish(cloud_in_base_link);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform exception: %s", ex.what());
      return;
    }
  }
}

void clear::once_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
  const Eigen::Vector4f & min_pt,
  const Eigen::Vector4f & max_pt,
  double cubeyaw)
{
  pcl::CropBox<pcl::PointXYZ> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_in);
  crop_box_filter.setMin(min_pt);
  crop_box_filter.setMax(max_pt);
  crop_box_filter.setNegative(true);
  crop_box_filter.setRotation(Eigen::Vector3f(0, 0, cubeyaw));
  crop_box_filter.filter(*cloud_in);
}

bool clear::is_non_area(double x, double y)
{
  for (const auto & poly : polygons_) {
    int cnt = 0;
    size_t n = poly.size();
    for (size_t i = 0; i < n; i++) {
      const Point2D & p1 = poly[i];
      const Point2D & p2 = poly[(i + 1) % n];
      if (((p1.y > y) != (p2.y > y)) && (x < (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y) + p1.x)) {
        cnt++;
      }
    }
    if (cnt % 2 == 1) {
      return true;
    }
  }
  return false;
}

void clear::combineCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
{
  // TODO:结合里程计进行多边形判断
  cloudCB(input_msg);
}

bool clear::loadparam(const std::string & param_name)
{
  // 声明参数
  this->declare_parameter<std::string>(param_name + "_file", "");
  std::string yaml_file;
  this->get_parameter(param_name + "_file", yaml_file);

  if (yaml_file.empty()) {
    // 尝试从包路径加载默认文件
    yaml_file = "/home/rm/sentinel-up-gimbal/src/pclfilter/config/basemap.yaml";
  }

  try {
    YAML::Node config = YAML::LoadFile(yaml_file);
    if (!config["polygons"]) {
      RCLCPP_ERROR(this->get_logger(), "No 'polygons' key in yaml file");
      return false;
    }

    polygons_.clear();
    for (const auto & polygon_node : config["polygons"]) {
      Polygon2D polygon;
      for (const auto & point_node : polygon_node) {
        Point2D pt;
        pt.x = point_node["x"].as<double>();
        pt.y = point_node["y"].as<double>();
        polygon.push_back(pt);
      }
      if (!polygon.empty()) {
        polygons_.push_back(polygon);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu polygons from %s", polygons_.size(), yaml_file.c_str());
    return true;
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load yaml file: %s", e.what());
    return false;
  }
}

bool clear::loadcubeParam(const std::string & param_name)
{
  this->declare_parameter<std::string>(param_name + "_file", "");
  std::string yaml_file;
  this->get_parameter(param_name + "_file", yaml_file);

  if (yaml_file.empty()) {
    yaml_file = "/home/rm/sentinel-up-gimbal/src/pclfilter/config/cube.yaml";
  }

  try {
    YAML::Node config = YAML::LoadFile(yaml_file);
    if (!config["cube"]) {
      RCLCPP_ERROR(this->get_logger(), "No 'cube' key in yaml file");
      return false;
    }

    min_pts_.clear();
    max_pts_.clear();

    for (const auto & cuboid : config["cube"]) {
      if (!cuboid["min"] || !cuboid["max"]) {
        RCLCPP_ERROR(this->get_logger(), "Cuboid missing min/max");
        continue;
      }

      Eigen::Vector4f min_pt, max_pt;
      auto min_vec = cuboid["min"].as<std::vector<float>>();
      auto max_vec = cuboid["max"].as<std::vector<float>>();

      if (min_vec.size() != 4 || max_vec.size() != 4) {
        RCLCPP_ERROR(this->get_logger(), "Cuboid min/max not 4 elements");
        continue;
      }

      for (int j = 0; j < 4; ++j) {
        min_pt[j] = min_vec[j];
        max_pt[j] = max_vec[j];
      }

      min_pts_.push_back(min_pt);
      max_pts_.push_back(max_pt);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu cuboids from %s", min_pts_.size(), yaml_file.c_str());
    return true;
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load yaml file: %s", e.what());
    return false;
  }
}
}  // namespace pclfilter
