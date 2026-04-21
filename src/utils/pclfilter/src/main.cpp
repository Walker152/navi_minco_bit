#include "../include/depth_cluster.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class DepthClusterNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unique_ptr<pclfilter::DepthCluster> depthCluster_;

public:
  DepthClusterNode() : rclcpp::Node("depth_cluster")
  {
    // 声明参数
    this->declare_parameter<std::string>("topics.input_cloud_topic", "/gicp_map");
    this->declare_parameter<std::string>("topics.output_ground_topic", "/ground_points_map");
    this->declare_parameter<std::string>("topics.output_obstacles_topic", "/obstacle_clusters_map");

    this->declare_parameter<double>("clustering.vertical_resolution", 1.0);
    this->declare_parameter<double>("clustering.horizontal_resolution", 0.2);
    this->declare_parameter<int>("clustering.lidar_lines", 32);
    this->declare_parameter<int>("clustering.min_cluster_size", 3);
    this->declare_parameter<double>("clustering.ground.max_slope_angle", 30.0);
    this->declare_parameter<int>("clustering.normal_estimation_k", 15);
    this->declare_parameter<double>("clustering.depth_threshold", 0.5);
    this->declare_parameter<bool>("clustering.use_euclidean", true);
    this->declare_parameter<double>("clustering.euclidean.cluster_tolerance", 0.8);
    this->declare_parameter<int>("clustering.euclidean.min_cluster_size", 3);
    this->declare_parameter<int>("clustering.euclidean.max_cluster_size", 25000);
    this->declare_parameter<double>("clustering.normal_curvature_threshold", 0.05);
    this->declare_parameter<bool>("clustering.auto_estimate_reference_normal", true);
    this->declare_parameter<double>("clustering.sensor_tilt_angle_x", 30.0);
    this->declare_parameter<double>("clustering.sensor_tilt_angle_y", 0.0);
    this->declare_parameter<bool>("clustering.adaptive_radius", false);
    this->declare_parameter<bool>("clustering.temporal_filter", false);
    this->declare_parameter<int>("clustering.temporal_window_size", 5);

    // 获取参数
    std::string input_topic = this->get_parameter("topics.input_cloud_topic").as_string();
    std::string output_ground = this->get_parameter("topics.output_ground_topic").as_string();
    std::string output_obstacles = this->get_parameter("topics.output_obstacles_topic").as_string();

    double vert_res = this->get_parameter("clustering.vertical_resolution").as_double();
    double horiz_res = this->get_parameter("clustering.horizontal_resolution").as_double();
    int lidar_lines = this->get_parameter("clustering.lidar_lines").as_int();
    int min_cluster_size = this->get_parameter("clustering.min_cluster_size").as_int();
    double ground_max_slope = this->get_parameter("clustering.ground.max_slope_angle").as_double();
    int normal_k = this->get_parameter("clustering.normal_estimation_k").as_int();
    double depth_thresh = this->get_parameter("clustering.depth_threshold").as_double();
    bool use_euclidean = this->get_parameter("clustering.use_euclidean").as_bool();
    double euclidean_tolerance = this->get_parameter("clustering.euclidean.cluster_tolerance").as_double();
    int euclidean_min_size = this->get_parameter("clustering.euclidean.min_cluster_size").as_int();
    int euclidean_max_size = this->get_parameter("clustering.euclidean.max_cluster_size").as_int();
    double normal_curvature = this->get_parameter("clustering.normal_curvature_threshold").as_double();
    bool auto_estimate = this->get_parameter("clustering.auto_estimate_reference_normal").as_bool();
    double tilt_x = this->get_parameter("clustering.sensor_tilt_angle_x").as_double();
    double tilt_y = this->get_parameter("clustering.sensor_tilt_angle_y").as_double();
    bool adaptive_radius = this->get_parameter("clustering.adaptive_radius").as_bool();
    bool temporal_filter = this->get_parameter("clustering.temporal_filter").as_bool();
    int temporal_window = this->get_parameter("clustering.temporal_window_size").as_int();

    // 打印配置
    RCLCPP_INFO(this->get_logger(), "=== DepthCluster Parameters ===");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Output ground: %s", output_ground.c_str());
    RCLCPP_INFO(this->get_logger(), "Output obstacles: %s", output_obstacles.c_str());
    RCLCPP_INFO(this->get_logger(), "Max slope angle: %.1f°", ground_max_slope);
    RCLCPP_INFO(this->get_logger(), "Normal estimation K: %d", normal_k);
    RCLCPP_INFO(this->get_logger(), "Curvature threshold: %.3f", normal_curvature);
    RCLCPP_INFO(this->get_logger(), "Auto estimate reference normal: %s", auto_estimate ? "yes" : "no");
    if (!auto_estimate)
      RCLCPP_INFO(this->get_logger(), "Sensor tilt X: %.1f°, Y: %.1f°", tilt_x, tilt_y);
    RCLCPP_INFO(this->get_logger(), "Use Euclidean: %s", use_euclidean ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Euclidean tolerance: %.2f m", euclidean_tolerance);
    RCLCPP_INFO(this->get_logger(), "Adaptive radius: %s", adaptive_radius ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Temporal filter: %s", temporal_filter ? "true" : "false");
    if (temporal_filter)
      RCLCPP_INFO(this->get_logger(), "Temporal window size: %d", temporal_window);

    // 创建算法实例
    depthCluster_.reset(new pclfilter::DepthCluster(static_cast<float>(vert_res),
      static_cast<float>(horiz_res),
      lidar_lines,
      min_cluster_size,
      static_cast<float>(ground_max_slope),
      normal_k,
      static_cast<float>(depth_thresh),
      use_euclidean,
      static_cast<float>(euclidean_tolerance),
      euclidean_min_size,
      euclidean_max_size,
      static_cast<float>(normal_curvature),
      auto_estimate,
      static_cast<float>(tilt_x),
      static_cast<float>(tilt_y),
      adaptive_radius,
      temporal_filter,
      temporal_window));

    // 创建发布者和订阅者
    point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_obstacles, 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_ground, 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10, std::bind(&DepthClusterNode::CloudCB, this, std::placeholders::_1));
  }

  void CloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);
    RCLCPP_INFO(this->get_logger(), "Received %zu points", laserCloudIn->size());

    depthCluster_->setInputCloud(laserCloudIn);

    const auto & clustersIndex = depthCluster_->getClustersIndex();
    const auto & ground_index = depthCluster_->getGroundCloudIndices();

    RCLCPP_INFO(this->get_logger(), "Clusters: %zu", clustersIndex.size());
    for (size_t i = 0; i < clustersIndex.size(); ++i)
      RCLCPP_INFO(this->get_logger(), "  Cluster %zu: %zu points", i, clustersIndex[i].size());

    // 构建彩色点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    colorCloud->resize(laserCloudIn->size());
    for (size_t i = 0; i < laserCloudIn->size(); ++i) {
      colorCloud->points[i].x = laserCloudIn->points[i].x;
      colorCloud->points[i].y = laserCloudIn->points[i].y;
      colorCloud->points[i].z = laserCloudIn->points[i].z;
      colorCloud->points[i].r = 128;
      colorCloud->points[i].g = 128;
      colorCloud->points[i].b = 128;
    }

    // 地面绿色
    for (int idx : ground_index) {
      if (idx >= 0 && idx < static_cast<int>(colorCloud->size())) {
        colorCloud->points[idx].r = 0;
        colorCloud->points[idx].g = 255;
        colorCloud->points[idx].b = 0;
      }
    }

    // 聚类随机颜色
    srand(12345);
    for (const auto & cluster : clustersIndex) {
      uint8_t r = rand() % 256;
      uint8_t g = rand() % 256;
      uint8_t b = rand() % 256;
      for (int idx : cluster) {
        if (idx >= 0 && idx < static_cast<int>(colorCloud->size())) {
          colorCloud->points[idx].r = r;
          colorCloud->points[idx].g = g;
          colorCloud->points[idx].b = b;
        }
      }
    }

    // 发布地面点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*colorCloud, ground_index, *groundOut);
    sensor_msgs::msg::PointCloud2 groundMsg;
    pcl::toROSMsg(*groundOut, groundMsg);
    groundMsg.header = cloud_ptr->header;
    ground_pub_->publish(groundMsg);
    RCLCPP_INFO(this->get_logger(), "Published ground points: %zu", groundOut->size());

    // 发布障碍物点云
    auto merged = depthCluster_->getMergedClustersIndex();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obsOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*colorCloud, merged, *obsOut);
    sensor_msgs::msg::PointCloud2 obsMsg;
    pcl::toROSMsg(*obsOut, obsMsg);
    obsMsg.header = cloud_ptr->header;
    point_pub_->publish(obsMsg);
    RCLCPP_INFO(this->get_logger(), "Published obstacle points: %zu", obsOut->size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthClusterNode>());
  rclcpp::shutdown();
  return 0;
}