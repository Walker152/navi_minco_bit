#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Eigen>
#include <nlohmann/json.hpp>

namespace pclfilter
{
    struct Point2D{
        double x;
        double y;
    };
    using Polygon2D = std::vector<Point2D>; 

    class ClearNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr basemap_cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        std::vector<Polygon2D> polygons_;
        // cube
        std::vector<Eigen::Vector4f> min_pts_;
        std::vector<Eigen::Vector4f> max_pts_;
        //
        double CTE; // 点云膨胀系数
        double x_;
        double y_;
        double yaw_;
        double pitch_;
        double roll_;
        bool if_clear_;
        int way_select;

    public:
        bool loadparam(const std::string &param_name);
        bool is_non_area(double x, double y);
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
        void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg);
        void basemapCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg);
        void combineCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg);
        bool loadcubeParam(const std::string &param_name);
        void init_basemap();
        void init_odom();
        void init_mapwithodom();
        void once_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
            const Eigen::Vector4f &min_pt,
            const Eigen::Vector4f &max_pt,
            double cubeyaw);
        void once_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
            const Eigen::Vector4f &min_pt,
            const Eigen::Vector4f &max_pt);
        
        
        explicit ClearNode(const std::string& node_name)
        : Node(node_name)
        {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        };
    };                                                                          //显示构造函数
} // namespace pclfilter