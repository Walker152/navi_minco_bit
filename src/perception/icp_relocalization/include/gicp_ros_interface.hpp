#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <mutex>

#include "color_text.hpp"
#include "gicp_filter.hpp"
#include "log.hpp"

#define LOG_DEBUG(prefix, ...) icp_log::log_info((prefix), __VA_ARGS__)
#define LOG_DEBUG_BLOCK(prefix, ...) icp_log::log_block((prefix), __VA_ARGS__)

namespace icp_relocalization {

// 负责ROS接口、数据融合和漂移检测
class GicpRosInterface : public rclcpp::Node
{
public:
  explicit GicpRosInterface(const rclcpp::NodeOptions & options);

private:
  //超时处理
  std::chrono::steady_clock::time_point reloc_start_time_;  //重定位开始时间
  double timeout_seconds_;                                  // 超时阈值
  std::vector<double> default_pose_on_timeout_;             // 超时后发布的默认位姿

  // 默认位姿发布函数
  void publishDefaultPose();

  enum class State
  {
    UNINITIALIZED,
    INITIALIZING,
    CONVERGING,
    LOCALIZED
  };

  enum class Mode
  {
    MULTI_GUESS,
    INITIAL_GUESS
  };

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void setupGicp(const std::string & target_pcd_file);
  void activateLidarSubscription();
  void initializeDebugPublishers();
  void startVisualizationTimer();
  void startTfPublishTimer();
  void tfPublishTimerCallback();

  // ROS 接口
  std::unique_ptr<GicpFilter> gicp_filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_cropped_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_aligned_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_raw_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_cropped_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_data_;
  rclcpp::CallbackGroup::SharedPtr callback_group_compute_;
  rclcpp::CallbackGroup::SharedPtr callback_group_utility_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr tf_publish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;

  void fsmTimerCallback();
  void visualizationTimerCallback();
  void runFSM();
  void startFsmTimer();
  void relocalizeServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 状态与缓存
  State state_ = State::UNINITIALIZED;

  // 点云累积
  PointCloud::Ptr accumulated_cloud_;
  PointCloud::Ptr current_source_cloud_;
  int current_accumulated_frames_ = 0;
  rclcpp::Time last_cloud_stamp_;
  std::string cloud_frame_id_;
  std::mutex cloud_mtx_;

  bool gicp_initialized_ = false;
  rclcpp::Time last_icp_time_;

  bool enable_continuous_relocalization_;  // 低频重定位功能开关
  int tracking_lost_count_ = 0;            // 连续追踪丢失计数
  int max_tracking_lost_count_;            // 最大允许跟丢次数
  double max_translation_jump_ = 1.0;      // 连续更新允许的最大平移跳变(m)
  double max_yaw_jump_ = 0.35;             // 连续更新允许的最大航向跳变(rad)
  bool has_last_successful_pose_ = false;  // 是否已有成功发布并确认的位姿
  Eigen::Matrix4f last_successful_pose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f latest_odom_pose_ = Eigen::Matrix4f::Identity();
  std::mutex odom_mtx_;

  // 变换与位姿
  Eigen::Matrix4f map_to_camera_init_ = Eigen::Matrix4f::Identity();
  std::mutex pose_mtx_;

  // 默认参数
  std::string map_frame_;
  bool visualization_en_ = true;      // 是否启用可视化
  std::string source_cloud_topic_;    // 激光雷达点云话题
  double alignment_frequency_;        // 地图对齐(GICP)低频执行频率
  Mode mode_ = Mode::MULTI_GUESS;     // 重定位模式
  int accumulate_frames_;             // 参与配准的累积帧数
  double score_threshold_ = 20;       // small_gicp score阈值（越小越好）
  int converged_count_threshold_;     // 收敛次数阈值
  int converged_count_ = 0;           // 当前收敛次数
  bool has_localized_once_ = false;   // 是否曾经成功进入LOCALIZED
  std::vector<double> initial_pose_;  // 初始位姿猜测 [x, y, z, roll, pitch, yaw]

  std::chrono::duration<double> fsm_period_{1.0};
  double tf_publish_frequency_ = 10.0;
  std::chrono::duration<double> tf_publish_period_{0.1};

  GicpFilter::Options gicp_options_;
};

}  // namespace icp_relocalization
