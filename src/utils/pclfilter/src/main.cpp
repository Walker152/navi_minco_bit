#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "../include/depth_cluster.hpp"
#include <iostream>

class DepthClusterNode : public rclcpp::Node
{
private:      
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  pclfilter::DepthCluster depthCluster_;

public:
  DepthClusterNode() : rclcpp::Node("depth_cluster"), depthCluster_(1, 0.2, 32, 20)
  {
    point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 10);
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_baselink", 10);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10,
      std::bind(&DepthClusterNode::CloudCB, this, std::placeholders::_1));
  }

  void CloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr)
  {
    // 使用PointXYZ接收原始数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);

    // 步骤2：深度聚类处理
    depthCluster_.setInputCloud(laserCloudIn); // 输入点云到算法

    // 步骤3：获取聚类结果
    vector<vector<int>> clustersIndex = depthCluster_.getClustersIndex();
    
    // 创建彩色点云用于可视化
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudColor(new pcl::PointCloud<pcl::PointXYZRGB>());
    laserCloudColor->resize(laserCloudIn->size());
    
    // 复制XYZ坐标
    for (size_t i = 0; i < laserCloudIn->size(); ++i)
    {
      laserCloudColor->points[i].x = laserCloudIn->points[i].x;
      laserCloudColor->points[i].y = laserCloudIn->points[i].y;
      laserCloudColor->points[i].z = laserCloudIn->points[i].z;
      // 默认颜色（灰色）
      laserCloudColor->points[i].r = 128;
      laserCloudColor->points[i].g = 128;
      laserCloudColor->points[i].b = 128;
    }
    
    // 为每个聚类分配不同的随机颜色
    srand(12345); // 固定随机种子，保证颜色一致性
    for (auto cluster_vec : clustersIndex)
    {
      uint8_t r = rand() % 256;
      uint8_t g = rand() % 256;
      uint8_t b = rand() % 256;
      
      for (int idx : cluster_vec)
      {
        laserCloudColor->points[idx].r = r;
        laserCloudColor->points[idx].g = g;
        laserCloudColor->points[idx].b = b;
      }
    }

    // 步骤4：处理地面点云
    auto ground_index = depthCluster_.getGroundCloudIndices();
    // 地面点设置为绿色
    for (int idx : ground_index)
    {
      laserCloudColor->points[idx].r = 0;
      laserCloudColor->points[idx].g = 255;
      laserCloudColor->points[idx].b = 0;
    }

    // 步骤5：发布地面点云（绿色）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*laserCloudColor, ground_index, *ground_points);
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_points, ground_msg);
    ground_msg.header = cloud_ptr->header;
    ground_pub_->publish(ground_msg);

    // 步骤6：发布障碍物聚类结果（彩色）
    auto cluster_indices = depthCluster_.getMergedClustersIndex();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*laserCloudColor, cluster_indices, *cluster_points);
    sensor_msgs::msg::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cluster_points, cluster_msg);
    cluster_msg.header = cloud_ptr->header;
    point_pub_->publish(cluster_msg);
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