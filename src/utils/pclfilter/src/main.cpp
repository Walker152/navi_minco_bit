#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "../include/depth_cluster.hpp"
#include <iostream>
#include <memory>

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
    // Declare parameters with defaults (match config/depth_cluster.yaml structure)
    this->declare_parameter<std::string>("topics.input_cloud_topic", "/gicp_map");
    this->declare_parameter<std::string>("topics.output_obstacles_topic", "/obstacle_clusters_map");
    this->declare_parameter<std::string>("topics.output_ground_topic", "/ground_points_map");

    this->declare_parameter<double>("clustering.vertical_resolution", 1.0);
    this->declare_parameter<double>("clustering.horizontal_resolution", 0.2);
    this->declare_parameter<int>("clustering.lidar_lines", 32);
    this->declare_parameter<int>("clustering.min_cluster_size", 20);
    this->declare_parameter<double>("clustering.ground.distance_threshold", 0.3);
    this->declare_parameter<double>("clustering.ground.height_threshold", 0.5);
    this->declare_parameter<double>("clustering.ground.max_slope_angle", 30.0);

    // Filtering
    this->declare_parameter<double>("filtering.range.min_range", 0.1);
    this->declare_parameter<double>("filtering.range.max_range", 100.0);
    this->declare_parameter<double>("filtering.z_range.min_height", -2.0);
    this->declare_parameter<double>("filtering.z_range.max_height", 5.0);

    // Performance
    this->declare_parameter<double>("performance.publish_rate", 0.0);
    this->declare_parameter<int>("performance.queue_size", 10);
    this->declare_parameter<bool>("performance.use_multi_threading", false);

    // Debug
    this->declare_parameter<bool>("debug.verbose", false);
    this->declare_parameter<bool>("debug.save_results", false);
    this->declare_parameter<std::string>("debug.save_path", "/tmp/depth_cluster_debug");

    // Frames
    this->declare_parameter<std::string>("frames.input_frame", "map");
    this->declare_parameter<std::string>("frames.output_frame", "map");
    this->declare_parameter<std::string>("frames.base_frame", "base_link");
    this->declare_parameter<double>("frames.tf_timeout", 1.0);

    // Load parameters
    std::string input_topic = this->get_parameter("topics.input_cloud_topic").as_string();
    std::string output_obstacles = this->get_parameter("topics.output_obstacles_topic").as_string();
    std::string output_ground = this->get_parameter("topics.output_ground_topic").as_string();

    double vert_res = this->get_parameter("clustering.vertical_resolution").as_double();
    double horiz_res = this->get_parameter("clustering.horizontal_resolution").as_double();
    int lidar_lines = this->get_parameter("clustering.lidar_lines").as_int();
    int min_cluster_size = this->get_parameter("clustering.min_cluster_size").as_int();
    double ground_dist = this->get_parameter("clustering.ground.distance_threshold").as_double();
    double ground_height = this->get_parameter("clustering.ground.height_threshold").as_double();
    double ground_max_slope = this->get_parameter("clustering.ground.max_slope_angle").as_double();

    // Filtering
    double min_range = this->get_parameter("filtering.range.min_range").as_double();
    double max_range = this->get_parameter("filtering.range.max_range").as_double();
    double min_height = this->get_parameter("filtering.z_range.min_height").as_double();
    double max_height = this->get_parameter("filtering.z_range.max_height").as_double();

    // Performance
    double publish_rate = this->get_parameter("performance.publish_rate").as_double();
    int queue_size = this->get_parameter("performance.queue_size").as_int();
    bool use_multi_threading = this->get_parameter("performance.use_multi_threading").as_bool();

    // Debug
    bool verbose = this->get_parameter("debug.verbose").as_bool();
    bool save_results = this->get_parameter("debug.save_results").as_bool();
    std::string save_path = this->get_parameter("debug.save_path").as_string();

    // Frames
    std::string input_frame = this->get_parameter("frames.input_frame").as_string();
    std::string output_frame = this->get_parameter("frames.output_frame").as_string();
    std::string base_frame = this->get_parameter("frames.base_frame").as_string();
    double tf_timeout = this->get_parameter("frames.tf_timeout").as_double();

    // Create DepthCluster with configured params
    depthCluster_.reset(new pclfilter::DepthCluster(static_cast<float>(vert_res), static_cast<float>(horiz_res), lidar_lines, min_cluster_size));
    // 将地面相关阈值传递给算法
    depthCluster_->setGroundDistanceThreshold(static_cast<float>(ground_dist));
    depthCluster_->setGroundHeightThreshold(static_cast<float>(ground_height));
    depthCluster_->setGroundMaxSlopeAngle(static_cast<float>(ground_max_slope));

    // Create publishers/subscriber using configured topics
    point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_obstacles, 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_ground, 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&DepthClusterNode::CloudCB, this, std::placeholders::_1));
  }
  // 给定时间偏移的雷达（如某些Velodyne型号），基于恒定角速度模型：
  // 该注释说明对旋转式 LiDAR 的时间/运动补偿考虑（deskew）。

  void CloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr)
  {
    // 使用PointXYZ接收原始数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);

    // 步骤1：深度聚类处理
    depthCluster_->setInputCloud(laserCloudIn);

    // 步骤2：获取处理结果
    const auto& clustersIndex = depthCluster_->getClustersIndex();
    const auto& ground_index = depthCluster_->getGroundCloudIndices();
    
    // 预分配彩色点云内存
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor(new pcl::PointCloud<pcl::PointXYZRGB>());
    laserCloudColor->resize(laserCloudIn->size());
    pcl_conversions::toPCL(cloud_ptr->header, laserCloudColor->header);
    
    // 单次遍历初始化点云（避免多次遍历）
    const uint8_t DEFAULT_R = 128, DEFAULT_G = 128, DEFAULT_B = 128;
    for (size_t i = 0; i < laserCloudIn->size(); ++i)
    {
      const auto& pt = laserCloudIn->points[i];
      auto& col_pt = laserCloudColor->points[i];
      col_pt.x = pt.x;
      col_pt.y = pt.y;
      col_pt.z = pt.z;
      col_pt.r = DEFAULT_R;
      col_pt.g = DEFAULT_G;
      col_pt.b = DEFAULT_B;
    }
    
    // 为地面点设置绿色（优先于聚类颜色）
    for (int idx : ground_index)
    {
      if (idx >= 0 && idx < static_cast<int>(laserCloudColor->size()))
      {
        laserCloudColor->points[idx].r = 0;
        laserCloudColor->points[idx].g = 255;
        laserCloudColor->points[idx].b = 0;
      }
    }
    
    // 为每个聚类分配不同的随机颜色
    srand(12345);
    for (const auto& cluster_vec : clustersIndex)
    {
      uint8_t r = rand() % 256;
      uint8_t g = rand() % 256;
      uint8_t b = rand() % 256;
      
      for (int idx : cluster_vec)
      {
        if (idx >= 0 && idx < static_cast<int>(laserCloudColor->size()))
        {
          laserCloudColor->points[idx].r = r;
          laserCloudColor->points[idx].g = g;
          laserCloudColor->points[idx].b = b;
        }
      }
    }

    // 发布地面点云（绿色）
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZRGB>());
      ground_points->reserve(ground_index.size());
      for (int idx : ground_index)
      {
        if (idx >= 0 && idx < static_cast<int>(laserCloudColor->size()))
        {
          ground_points->push_back(laserCloudColor->points[idx]);
        }
      }
      sensor_msgs::msg::PointCloud2 ground_msg;
      pcl::toROSMsg(*ground_points, ground_msg);
      ground_msg.header = cloud_ptr->header;
      ground_pub_->publish(ground_msg);
    }

    // 发布障碍物聚类结果（彩色）
    {
      const auto& cluster_indices = depthCluster_->getMergedClustersIndex();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZRGB>());
      cluster_points->reserve(cluster_indices.size());
      for (int idx : cluster_indices)
      {
        if (idx >= 0 && idx < static_cast<int>(laserCloudColor->size()))
        {
          cluster_points->push_back(laserCloudColor->points[idx]);
        }
      }
      sensor_msgs::msg::PointCloud2 cluster_msg;
      pcl::toROSMsg(*cluster_points, cluster_msg);
      cluster_msg.header = cloud_ptr->header;
      point_pub_->publish(cluster_msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthClusterNode>());
  rclcpp::shutdown();
  return 0;
}

// TODO:
// 点云坐标系转换
// 参数调整