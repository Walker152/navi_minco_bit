#pragma once
#include <unordered_map>
#include <queue>
#include <ctime>
#include <set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
using namespace std;
using namespace pcl;
// 实现点云的地面提取与基于深度的快速聚类

namespace pclfilter
{
    // 点云索引与深度信息
    struct PointInfo
    {
    public:
        int index_;
        float depth_;
        PointInfo(int index, float depth) : index_(index), depth_(depth) {};
    };

    class DepthCluster
    {
    public:
        DepthCluster(float vertcal_resolution,
                     float horizontal_resolution,
                     int lidar_lines,
                     int cluster_size);
        void initParams();
        // 点云输入
        void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

        // 输入点云指针，返回拟合平面的参数
        Eigen::Vector4f estimatePlaneParams(pcl::PointCloud<pcl::PointXYZ>::Ptr &initial_ground_points);
        /**
         * 地面点提取主函数
         * @param msg 输入点云
         * @param label_image 标签图像（输出地面标记）
         * @return 地面点索引集合
         */
        vector<int> exactGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg,
                                      vector<vector<int>> &label_image);

        /**
         * 根据平面参数标记地面点
         * @param msg 输入点云
         * @param label_image 标签图像
         * @param ground_params 平面方程参数
         * @return 地面点索引集合
         */
        vector<int> exactGroundCloudIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg,
                                            vector<vector<int>> &label_image,
                                            Eigen::Vector4f &ground_params);

        /**
         * 提取初始地面点（低高度点）
         * @param cloud 输入点云（已排序）
         * @param initial_ground_points 输出初始地面点
         */
        void extractInitialGroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &initial_ground_points);

        /**
         * 区域增长聚类核心算法 ？？？？？？？？？？
         * @param depth_image 深度图像
         * @param label_image 标签图像（输入地面标记，输出聚类标记）
         * @param cloud_msg 原始点云（用于索引映射）
         */
        void labelComponents(const vector<vector<PointInfo>> &depth_image,
                             vector<vector<int>> &label_image,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg);

        // 传入深度点云，返回索引与对应深度
        vector<vector<PointInfo>> generateDepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused_ptr);

        // 根据索引和深度信息判断，是否是同一聚类
        bool judgmentCondition(const vector<vector<PointInfo>> &depth_image,
                               const pair<int, int> &target_point,
                               const pair<int, int> &neigh_point);

        // 3D点转极坐标索引
        bool calculateCoordinate(const pcl::PointXYZ &point,
                                 int &row,
                                 int &col);

        // 列索引环状修正
        bool warpPoint(pair<int, int> &pt);
        // 结果获取接口
        vector<vector<int>> getClustersIndex(); // 获取各簇索引集合
        vector<int> getMergedClustersIndex();   // 获取合并后的所有簇索引
        vector<int> getGroundCloudIndices();    // 获取地面点索引
        void paramsReset();                     // 清空参数

    private:
        // 地面提取相关
        pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_Pointcloud_; // 排序后的点云？？？？？？？？？？/
        vector<int> ground_points_indices_;                      // indices释义:索引
        float sensor_height_;
        // 聚类参数
        float vertcal_resolution_;
        float horizontal_resolution_;
        int lidar_lines_;
        int cluster_size_;
        // 图像参数
        int image_rows_;
        int image_cols_;
        int vertical_angle_min_;
        int vertical_angle_max_;
        // 聚类结果存储
        vector<vector<int>> clusters_indices_vec_; // 各簇索引集合
    };
}