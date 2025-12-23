#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <deque>
#include <mutex>

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
      CONVERGING,
      LOCALIZED
    };

    enum class Mode
    {
      SAC_IA,
      INITIAL_GUESS
    };

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void setupGicp(const std::string& target_pcd_file);

    void publishStaticTf(const rclcpp::Time& stamp);
    void publishVisualization(const PointCloud::Ptr& cloud, const rclcpp::Time& stamp);
    void printEvaluation(const Eigen::Matrix4f& initial_guess, const Eigen::Matrix4f& final_transformation, double fitness_score, double time_ms);

    // ROS 接口
    std::unique_ptr<GicpFilter> gicp_filter_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr callback_group_lidar_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    void fsmTimerCallback();
    void runFSM();

    // 状态与缓存
    State state_ = State::UNINITIALIZED;

    // 点云累积
    PointCloud::Ptr accumulated_cloud_;
    int current_accumulated_frames_ = 0;
    rclcpp::Time last_cloud_stamp_;
    std::string cloud_frame_id_;

    bool gicp_initialized_ = false;
    rclcpp::Time last_icp_time_;

    // 变换与位姿
    Eigen::Matrix4f map_to_camera_init_ = Eigen::Matrix4f::Identity();

    // 默认参数
    std::string map_frame_;
    double alignment_frequency_;      // 地图对齐(GICP)低频执行频率
    Mode mode_ = Mode::SAC_IA;        // 重定位模式
    int accumulate_frames_;           // 参与配准的累积帧数
    double fitness_score_threshold_;  // 配准得分阈值
    int converged_count_threshold_;   // 收敛次数阈值
    int converged_count_ = 0;         // 当前收敛次数
    std::vector<double> initial_pose_; // 初始位姿猜测 [x, y, z, roll, pitch, yaw]

    GicpFilter::Options gicp_options_;
  };

}  // namespace icp_relocalization
