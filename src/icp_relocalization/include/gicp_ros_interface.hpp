#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <deque>

#include "color_text.hpp"
#include "gicp_filter.hpp"
#include "log.hpp"

#define LOG_DEBUG(prefix, ...) icp_log::log_info((prefix), __VA_ARGS__)
#define LOG_DEBUG_BLOCK(prefix, ...) icp_log::log_block((prefix), __VA_ARGS__)

namespace icp_relocalization
{

  // 负责ROS接口、数据融合和漂移检测
  class GicpRosInterface : public rclcpp::Node
  {
  public:
    explicit GicpRosInterface(const rclcpp::NodeOptions& options);

  private:
    enum class State
    {
      UNINITIALIZED,
      INITIALIZING,
      LOCALIZED
    };

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void setupGicp(const std::string& target_pcd_file);

    void checkDriftAndCorrect(const Eigen::Matrix4f& icp_pose);

    void publishPose(const Eigen::Matrix4f& pose, const rclcpp::Time& stamp);

    // ROS 接口
    std::unique_ptr<GicpFilter> gicp_filter_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr callback_group_lidar_;
    rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    void fsmTimerCallback();
    void runFSM();

    // 状态与缓存
    State state_ = State::UNINITIALIZED;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    PointCloud::Ptr latest_cloud_;
    std::string latest_cloud_frame_;
    rclcpp::Time latest_cloud_stamp_;
    bool has_new_cloud_ = false;
    bool gicp_initialized_ = false;
    rclcpp::Time last_icp_time_;

    // 变换与位姿
    Eigen::Matrix4f map_to_camera_init_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_icp_pose_ = Eigen::Matrix4f::Identity();  // base_link 在 map 下的最新融合位姿

    // 默认参数
    std::string base_frame_;
    std::string map_frame_;
    double drift_threshold_m_;    // 位置漂移阈值
    double drift_threshold_rad_;  // 姿态漂移阈值
    double alignment_frequency_;  // 地图对齐(GICP)低频执行频率
    bool publish_pose_on_odom_;   // 是否在每次里程计回调发布融合位姿
    bool use_initial_alignment_;  // 是否启用 SAC-IA 初始定位
    GicpFilter::Options gicp_options_;
  };

}  // namespace icp_relocalization
