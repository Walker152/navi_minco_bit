#include "gicp_ros_interface.hpp"
#include "gicp_utils.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace icp_relocalization {
using namespace color_text;
GicpRosInterface::GicpRosInterface(const rclcpp::NodeOptions & options)
: Node("gicp_relocalization_node", options)
{
  // Default parameters
  std::string mode_str = this->declare_parameter<std::string>("mode", "multi_guess");
  if (mode_str == "multi_guess")
    mode_ = Mode::MULTI_GUESS;
  else if (mode_str == "initial_guess")
    mode_ = Mode::INITIAL_GUESS;
  else {
    std::cout << color_text::YELLOW << "[GICP] Invalid mode: " << mode_str << ". Defaulting to multi_guess."
              << color_text::RESET << std::endl;
    mode_ = Mode::MULTI_GUESS;
  }

  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  visualization_en_ = this->declare_parameter<bool>("visualization_en", true);
  source_cloud_topic_ = this->declare_parameter<std::string>("source_cloud_topic", "/livox/stdpc");
  alignment_frequency_ = this->declare_parameter<double>("alignment_frequency", 1.0);
  tf_publish_frequency_ = this->declare_parameter<double>("tf_publish_frequency", 10.0);
  accumulate_frames_ = this->declare_parameter<int>("accumulate_frames", 5);
  score_threshold_ = this->declare_parameter<double>("gicp.score_threshold", -1.0);
  normalized_score_threshold_ = this->declare_parameter<double>("gicp.normalized_score_threshold", 0.1);
  min_inlier_ratio_ = this->declare_parameter<double>("gicp.min_inlier_ratio", 0.3);
  min_overlap_ratio_ = this->declare_parameter<double>("gicp.min_overlap_ratio", 0.3);
  fine_alignment_enabled_ = this->declare_parameter<bool>("gicp.fine_alignment.enable", true);
  fine_max_correspondence_distance_ =
    this->declare_parameter<double>("gicp.fine_alignment.max_correspondence_distance", 0.2);
  planar_observability_check_enabled_ =
    this->declare_parameter<bool>("gicp.planar_observability.enable", false);
  min_planar_eigen_ratio_ =
    this->declare_parameter<double>("gicp.planar_observability.min_eigen_ratio", 1.0e-3);
  converged_count_threshold_ = this->declare_parameter<int>("converged_count_threshold", 3);
  max_stability_xy_spread_ = this->declare_parameter<double>("convergence.max_xy_spread", 0.05);
  max_stability_yaw_spread_ = this->declare_parameter<double>("convergence.max_yaw_spread", 0.01);
  initial_pose_guard_enabled_ = this->declare_parameter<bool>("initial_pose_guard.enable", true);
  max_initial_translation_correction_ =
    this->declare_parameter<double>("initial_pose_guard.max_translation_correction", 0.5);
  max_initial_yaw_correction_ =
    this->declare_parameter<double>("initial_pose_guard.max_yaw_correction", 0.15);
  results_recording_enabled_ = this->declare_parameter<bool>("results.enable", false);
  results_path_ =
    this->declare_parameter<std::string>("results.path", "/tmp/gicp_relocalization_results.csv");
  enable_continuous_relocalization_ =
    this->declare_parameter<bool>("enable_continuous_relocalization", false);
  max_tracking_lost_count_ = this->declare_parameter<int>("max_tracking_lost_count", 3);
  max_translation_jump_ =
    this->declare_parameter<double>("continuous_relocalization.max_translation_jump", 1.0);
  max_yaw_jump_ = this->declare_parameter<double>("continuous_relocalization.max_yaw_jump", 0.35);

  if (!(std::isfinite(max_translation_jump_) && max_translation_jump_ > 0.0)) {
    max_translation_jump_ = 1.0;
  }
  if (!(std::isfinite(max_yaw_jump_) && max_yaw_jump_ > 0.0)) {
    max_yaw_jump_ = 0.35;
  }
  if (!std::isfinite(score_threshold_)) {
    score_threshold_ = -1.0;
  }
  if (!(std::isfinite(normalized_score_threshold_) && normalized_score_threshold_ > 0.0)) {
    normalized_score_threshold_ = 0.1;
  }
  if (!(std::isfinite(min_inlier_ratio_) && min_inlier_ratio_ >= 0.0 && min_inlier_ratio_ <= 1.0)) {
    min_inlier_ratio_ = 0.3;
  }
  if (!(std::isfinite(min_overlap_ratio_) && min_overlap_ratio_ >= 0.0 && min_overlap_ratio_ <= 1.0)) {
    min_overlap_ratio_ = 0.3;
  }
  if (!(std::isfinite(fine_max_correspondence_distance_) && fine_max_correspondence_distance_ > 0.0)) {
    fine_max_correspondence_distance_ = 0.2;
  }
  if (!(std::isfinite(min_planar_eigen_ratio_) && min_planar_eigen_ratio_ >= 0.0 &&
        min_planar_eigen_ratio_ <= 1.0)) {
    min_planar_eigen_ratio_ = 1.0e-3;
  }
  if (converged_count_threshold_ < 3) {
    converged_count_threshold_ = 3;
  }
  if (!(std::isfinite(max_stability_xy_spread_) && max_stability_xy_spread_ > 0.0)) {
    max_stability_xy_spread_ = 0.05;
  }
  if (!(std::isfinite(max_stability_yaw_spread_) && max_stability_yaw_spread_ > 0.0 &&
        max_stability_yaw_spread_ <= M_PI)) {
    max_stability_yaw_spread_ = 0.01;
  }
  if (!(std::isfinite(max_initial_translation_correction_) && max_initial_translation_correction_ > 0.0)) {
    max_initial_translation_correction_ = 0.5;
  }
  if (!(std::isfinite(max_initial_yaw_correction_) && max_initial_yaw_correction_ > 0.0 &&
        max_initial_yaw_correction_ <= M_PI)) {
    max_initial_yaw_correction_ = 0.15;
  }
  if (results_path_.empty()) {
    results_recording_enabled_ = false;
  }

  std::string target_pcd_file = this->declare_parameter<std::string>("target_pcd_file", "map.pcd");

  // Map Offset Parameters
  initial_pose_ =
    this->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // GICP Parameters
  gicp_options_.target_voxel_leaf_size =
    this->declare_parameter<double>("gicp.target_voxel_leaf_size", 0.1);
  gicp_options_.source_voxel_leaf_size =
    this->declare_parameter<double>("gicp.source_voxel_leaf_size", 0.1);
  gicp_options_.max_correspondence_distance =
    this->declare_parameter<double>("gicp.max_correspondence_distance", 1.5);
  gicp_options_.max_iterations = this->declare_parameter<int>("gicp.max_iterations", 100);
  gicp_options_.transformation_epsilon =
    this->declare_parameter<double>("gicp.transformation_epsilon", 1e-4);
  gicp_options_.euclidean_fitness_epsilon =
    this->declare_parameter<double>("gicp.euclidean_fitness_epsilon", 1e-4);

  // Height Filter Parameters (optional performance optimization)
  gicp_options_.height_filter_enabled = this->declare_parameter<bool>("height_filter.enable", false);
  gicp_options_.height_filter_min_z = this->declare_parameter<double>("height_filter.min_z", -1000.0);
  gicp_options_.height_filter_max_z = this->declare_parameter<double>("height_filter.max_z", 1000.0);

  // Source Crop Parameters
  gicp_options_.source_crop_enabled = this->declare_parameter<bool>("source_crop.enable", true);
  gicp_options_.source_crop_min_x = this->declare_parameter<double>("source_crop.min_x", -20.0);
  gicp_options_.source_crop_max_x = this->declare_parameter<double>("source_crop.max_x", 20.0);
  gicp_options_.source_crop_min_y = this->declare_parameter<double>("source_crop.min_y", -20.0);
  gicp_options_.source_crop_max_y = this->declare_parameter<double>("source_crop.max_y", 20.0);
  gicp_options_.source_crop_min_z = this->declare_parameter<double>("source_crop.min_z", -2.5);
  gicp_options_.source_crop_max_z = this->declare_parameter<double>("source_crop.max_z", 2.5);

  // Multi-Guess Parameters
  const auto rects =
    this->declare_parameter<std::vector<double>>("multi_guess.search_rectangles", std::vector<double>{});
  gicp_options_.z_candidates =
    this->declare_parameter<std::vector<double>>("multi_guess.z_candidates", std::vector<double>{0.0});
  gicp_options_.step_x = this->declare_parameter<double>("multi_guess.step_x", 1.0);
  gicp_options_.step_y = this->declare_parameter<double>("multi_guess.step_y", 1.0);
  gicp_options_.step_yaw = this->declare_parameter<double>("multi_guess.step_yaw", 0.785);

  if (rects.size() % 4 != 0) {
    std::cout << color_text::YELLOW << "[GICP] multi_guess.search_rectangles size (" << rects.size()
              << ") is not divisible by 4. Ignoring tail values." << color_text::RESET << std::endl;
  }
  gicp_options_.search_areas.clear();
  for (size_t i = 0; i + 3 < rects.size(); i += 4) {
    GicpFilter::SearchArea area;
    area.min_x = rects[i];
    area.max_x = rects[i + 1];
    area.min_y = rects[i + 2];
    area.max_y = rects[i + 3];
    gicp_options_.search_areas.push_back(area);
  }

  // 超时参数
  timeout_seconds_ = this->declare_parameter<double>("timeout_seconds", 30.0);
  reloc_start_time_ = std::chrono::steady_clock::now();  // 记录开始时间

  // 默认位姿参数 [x, y, z, roll, pitch, yaw]
  default_pose_on_timeout_ = this->declare_parameter<std::vector<double>>(
    "default_pose_on_timeout", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  if (default_pose_on_timeout_.size() != 6) {
    std::cout << color_text::YELLOW << "[GICP] default_pose_on_timeout size invalid ("
              << default_pose_on_timeout_.size() << "), fallback to [0,0,0,0,0,0]" << color_text::RESET
              << std::endl;
    default_pose_on_timeout_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  reloc_start_time_ = std::chrono::steady_clock::now();  // 记录开始时间
  std::cout << color_text::BLUE << "[GICP] Relocalization initialized: mode=" << mode_str
            << ", map_frame=" << map_frame_ << ", source_topic=" << source_cloud_topic_
            << ", alignment_hz=" << alignment_frequency_ << ", tf_hz=" << tf_publish_frequency_
            << ", accumulate_frames=" << accumulate_frames_ << ", raw_score_limit=" << score_threshold_
            << ", normalized_score_limit=" << normalized_score_threshold_
            << ", min_inlier_ratio=" << min_inlier_ratio_ << ", min_overlap_ratio=" << min_overlap_ratio_
            << ", target_pcd=" << target_pcd_file << color_text::RESET << std::endl;
  std::cout << color_text::BLUE
            << "[GICP] XY/Yaw validation: fine_alignment=" << (fine_alignment_enabled_ ? "true" : "false")
            << ", fine_max_corr_dist=" << fine_max_correspondence_distance_
            << ", planar_check=" << (planar_observability_check_enabled_ ? "true" : "false")
            << ", min_planar_ratio=" << min_planar_eigen_ratio_
            << ", stability_xy=" << max_stability_xy_spread_
            << ", stability_yaw=" << max_stability_yaw_spread_
            << ", initial_guard=" << (initial_pose_guard_enabled_ ? "true" : "false")
            << ", results=" << (results_recording_enabled_ ? results_path_ : "disabled")
            << color_text::RESET << std::endl;
  std::cout << color_text::BLUE << "[GICP] Timeout config: " << timeout_seconds_ << " s, default_pose=["
            << default_pose_on_timeout_[0] << ", " << default_pose_on_timeout_[1] << ", "
            << default_pose_on_timeout_[2] << ", " << default_pose_on_timeout_[3] << ", "
            << default_pose_on_timeout_[4] << ", " << default_pose_on_timeout_[5] << "]"
            << color_text::RESET << std::endl;
  std::cout << color_text::BLUE << "[GICP] Options: target_leaf=" << gicp_options_.target_voxel_leaf_size
            << ", source_leaf=" << gicp_options_.source_voxel_leaf_size
            << ", max_corr_dist=" << gicp_options_.max_correspondence_distance
            << ", max_iter=" << gicp_options_.max_iterations
            << ", trans_eps=" << gicp_options_.transformation_epsilon
            << ", fit_eps=" << gicp_options_.euclidean_fitness_epsilon << color_text::RESET << std::endl;
  std::cout << color_text::BLUE << "[GICP] Multi-guess: step_x=" << gicp_options_.step_x
            << ", step_y=" << gicp_options_.step_y << ", step_yaw=" << gicp_options_.step_yaw
            << ", z_candidates=" << gicp_options_.z_candidates.size()
            << ", areas=" << gicp_options_.search_areas.size() << color_text::RESET << std::endl;

  if (mode_ == Mode::MULTI_GUESS) {
    std::cout << color_text::BLUE << "[GICP] Mode: MULTI_GUESS. Waiting for lidar scan to initialize."
              << color_text::RESET << std::endl;
  } else {
    std::cout << color_text::BLUE << "[GICP] Mode: INITIAL_GUESS. Waiting for initial pose."
              << color_text::RESET << std::endl;
    // 如果提供了初始位姿参数，则直接使用
    if (initial_pose_.size() == 6) {
      std::cout << color_text::BLUE << "[GICP] Using initial pose from parameters." << color_text::RESET
                << std::endl;
    }
  }

  // ROS接口初始化
  callback_group_data_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_compute_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_utility_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  activateLidarSubscription();
  rclcpp::SubscriptionOptions odom_sub_options;
  odom_sub_options.callback_group = callback_group_data_;
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
    rclcpp::SensorDataQoS(),
    std::bind(&GicpRosInterface::odomCallback, this, std::placeholders::_1),
    odom_sub_options);

  initializeDebugPublishers();

  setupGicp(target_pcd_file);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  accumulated_cloud_ = std::make_shared<PointCloud>();
  current_source_cloud_ = std::make_shared<PointCloud>();
  initializeResultsRecorder();

  // 初始化FSM定时器
  if (alignment_frequency_ <= 0.0) {
    alignment_frequency_ = 1.0;
  }
  if (tf_publish_frequency_ <= 0.0) {
    tf_publish_frequency_ = 10.0;
  }
  fsm_period_ = std::chrono::duration<double>(1.0 / alignment_frequency_);
  tf_publish_period_ = std::chrono::duration<double>(1.0 / tf_publish_frequency_);
  startFsmTimer();
  startVisualizationTimer();
  startTfPublishTimer();

  // 初始化重定位服务
  relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>("/gicp_recall",
    std::bind(
      &GicpRosInterface::relocalizeServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_utility_);
}

void GicpRosInterface::activateLidarSubscription()
{
  rclcpp::SubscriptionOptions lidar_sub_options;
  lidar_sub_options.callback_group = callback_group_data_;
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(source_cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GicpRosInterface::lidarCallback, this, std::placeholders::_1),
    lidar_sub_options);
}

void GicpRosInterface::initializeDebugPublishers()
{
  if (!visualization_en_) {
    return;
  }

  pub_source_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/source_raw", 10);
  pub_source_cropped_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/source_cropped", 10);
  pub_source_aligned_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/source_aligned", 10);
  pub_target_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/target_raw", 10);
  pub_target_cropped_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/target_cropped", 10);
}

void GicpRosInterface::startFsmTimer()
{
  if (fsm_timer_) {
    fsm_timer_->cancel();
    fsm_timer_.reset();
  }

  fsm_timer_ = this->create_wall_timer(
    fsm_period_, std::bind(&GicpRosInterface::fsmTimerCallback, this), callback_group_compute_);
}

void GicpRosInterface::startVisualizationTimer()
{
  if (!visualization_en_) {
    return;
  }

  if (visualization_timer_) {
    visualization_timer_->cancel();
    visualization_timer_.reset();
  }

  visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
    std::bind(&GicpRosInterface::visualizationTimerCallback, this),
    callback_group_utility_);
}

void GicpRosInterface::startTfPublishTimer()
{
  if (tf_publish_timer_) {
    tf_publish_timer_->cancel();
    tf_publish_timer_.reset();
  }

  tf_publish_timer_ = this->create_wall_timer(tf_publish_period_,
    std::bind(&GicpRosInterface::tfPublishTimerCallback, this),
    callback_group_utility_);
}

void GicpRosInterface::tfPublishTimerCallback()
{
  Eigen::Matrix4f pose_snapshot = Eigen::Matrix4f::Identity();
  {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    if (!gicp_initialized_) {
      return;
    }
    pose_snapshot = map_to_camera_init_;
  }
  gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, pose_snapshot, this->now());
}

void GicpRosInterface::relocalizeServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::cout << color_text::BLUE << "[GICP] Received request to re-trigger relocalization."
            << color_text::RESET << std::endl;

  // 1. Reset State
  mode_ = Mode::MULTI_GUESS;
  state_ = State::UNINITIALIZED;
  converged_count_ = 0;
  has_localized_once_ = false;
  has_last_successful_pose_ = false;
  has_initial_reference_pose_ = false;
  convergence_poses_.clear();
  {
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    current_accumulated_frames_ = 0;
    if (accumulated_cloud_)
      accumulated_cloud_->clear();
    if (current_source_cloud_)
      current_source_cloud_->clear();
  }
  gicp_initialized_ = false;
  last_cloud_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  //超时计时器重置
  reloc_start_time_ = std::chrono::steady_clock::now();

  // 2. Reactivate Lidar Subscription if needed
  if (!lidar_sub_) {
    activateLidarSubscription();
  }

  // 3. Restart FSM Timer
  startFsmTimer();

  response->success = true;
  response->message = "Relocalization process restarted.";
}

void GicpRosInterface::setupGicp(const std::string & target_pcd_file)
{
  try {
    // 1. 加载点云
    PointCloud::Ptr target_cloud(new PointCloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_file, *target_cloud) == -1) {
      throw std::runtime_error("Couldn't read file " + target_pcd_file);
    }

    // 2. 初始化 GICP Filter
    gicp_filter_ = std::make_unique<GicpFilter>(target_cloud, gicp_options_);
  } catch (const std::runtime_error & e) {
    std::cerr << color_text::RED << "[GICP] Failed to setup GICP: " << e.what() << color_text::RESET
              << std::endl;
    rclcpp::shutdown();
  }
}

void GicpRosInterface::publishDefaultPose()
{
  std::cout << color_text::YELLOW << "[GICP] Publishing timeout default pose: ["
            << default_pose_on_timeout_[0] << ", " << default_pose_on_timeout_[1] << ", "
            << default_pose_on_timeout_[2] << ", " << default_pose_on_timeout_[3] << ", "
            << default_pose_on_timeout_[4] << ", " << default_pose_on_timeout_[5] << "]"
            << color_text::RESET << std::endl;

  Eigen::Matrix4f default_transform = Eigen::Matrix4f::Identity();

  default_transform(0, 3) = default_pose_on_timeout_[0];  // x
  default_transform(1, 3) = default_pose_on_timeout_[1];  // y
  default_transform(2, 3) = default_pose_on_timeout_[2];  // z

  tf2::Quaternion q;
  q.setRPY(default_pose_on_timeout_[3],  // roll
    default_pose_on_timeout_[4],         // pitch
    default_pose_on_timeout_[5]);        // yaw
  Eigen::Quaternionf eigen_q(q.w(), q.x(), q.y(), q.z());
  default_transform.block<3, 3>(0, 0) = eigen_q.toRotationMatrix();

  Eigen::Matrix4f pose_snapshot = Eigen::Matrix4f::Identity();
  {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    map_to_camera_init_ = default_transform;
    gicp_initialized_ = true;
    pose_snapshot = map_to_camera_init_;
  }

  gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, pose_snapshot, this->now());
  std::cout << color_text::GREEN << "[GICP] Default pose published" << color_text::RESET << std::endl;
}

void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloud::Ptr temp_cloud(new PointCloud());
  pcl::fromROSMsg(*msg, *temp_cloud);

  {
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    if (current_accumulated_frames_ >= accumulate_frames_) {
      return;
    }

    *accumulated_cloud_ += *temp_cloud;
    current_accumulated_frames_++;
    last_cloud_stamp_ = msg->header.stamp;
    cloud_frame_id_ = msg->header.frame_id;

    // Full cloud 输入可能超过 10 万点：用已有 source leaf 压缩，不再清空整个累积窗口。
    if (accumulated_cloud_->size() > 100000 && std::isfinite(gicp_options_.source_voxel_leaf_size) &&
        gicp_options_.source_voxel_leaf_size > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      const float leaf = static_cast<float>(gicp_options_.source_voxel_leaf_size);
      voxel_filter.setLeafSize(leaf, leaf, leaf);
      voxel_filter.setInputCloud(accumulated_cloud_);
      PointCloud::Ptr compacted_cloud(new PointCloud());
      voxel_filter.filter(*compacted_cloud);
      accumulated_cloud_.swap(compacted_cloud);
    }
  }
}

void GicpRosInterface::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & pose = msg->pose.pose;

  Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
  odom_pose(0, 3) = static_cast<float>(pose.position.x);
  odom_pose(1, 3) = static_cast<float>(pose.position.y);
  odom_pose(2, 3) = static_cast<float>(pose.position.z);

  Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));
  odom_pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

  std::lock_guard<std::mutex> lock(odom_mtx_);
  latest_odom_pose_ = odom_pose;
}

void GicpRosInterface::fsmTimerCallback()
{
  runFSM();
}

void GicpRosInterface::visualizationTimerCallback()
{
  if (!visualization_en_) {
    return;
  }

  rclcpp::Time now_stamp = this->now();
  Eigen::Matrix4f pose_snapshot = Eigen::Matrix4f::Identity();
  {
    std::lock_guard<std::mutex> lock(pose_mtx_);
    pose_snapshot = map_to_camera_init_;
  }

  if (pub_source_raw_ && current_source_cloud_ && !current_source_cloud_->empty() &&
      pub_source_raw_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 source_raw_msg;
    pcl::toROSMsg(*current_source_cloud_, source_raw_msg);
    source_raw_msg.header.frame_id = cloud_frame_id_.empty() ? map_frame_ : cloud_frame_id_;
    source_raw_msg.header.stamp = (last_cloud_stamp_.nanoseconds() == 0) ? now_stamp : last_cloud_stamp_;
    pub_source_raw_->publish(source_raw_msg);
  }

  if (pub_target_raw_ && gicp_filter_ && pub_target_raw_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 target_raw_msg;
    pcl::toROSMsg(*gicp_filter_->getTargetCloud(), target_raw_msg);
    target_raw_msg.header.frame_id = map_frame_;
    target_raw_msg.header.stamp = now_stamp;
    pub_target_raw_->publish(target_raw_msg);
  }

  if (!current_source_cloud_ || current_source_cloud_->empty()) {
    return;
  }

  rclcpp::Time cloud_stamp = (last_cloud_stamp_.nanoseconds() == 0) ? now_stamp : last_cloud_stamp_;

  gicp_utils::publishSourceCroppedDebug(visualization_en_,
    gicp_options_,
    map_frame_,
    current_source_cloud_,
    pose_snapshot,
    cloud_stamp,
    pub_source_cropped_);

  gicp_utils::publishTargetCroppedDebug(
    visualization_en_, map_frame_, cloud_stamp, gicp_filter_.get(), pub_target_cropped_);

  gicp_utils::publishVisualization(current_source_cloud_,
    cloud_stamp,
    visualization_en_,
    gicp_initialized_,
    map_frame_,
    pose_snapshot,
    pub_source_aligned_);
}

double GicpRosInterface::poseYaw(const Eigen::Matrix4f & pose)
{
  return std::atan2(static_cast<double>(pose(1, 0)), static_cast<double>(pose(0, 0)));
}

double GicpRosInterface::wrappedYawDifference(const double lhs, const double rhs)
{
  return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

void GicpRosInterface::initializeResultsRecorder()
{
  if (!results_recording_enabled_) {
    return;
  }

  const std::filesystem::path results_path(results_path_);
  std::error_code error;
  if (results_path.has_parent_path()) {
    std::filesystem::create_directories(results_path.parent_path(), error);
  }
  if (error) {
    std::cerr << color_text::YELLOW << "[GICP] Failed to create results directory: " << error.message()
              << color_text::RESET << std::endl;
    results_recording_enabled_ = false;
    return;
  }

  const bool write_header = !std::filesystem::exists(results_path, error) ||
                            (!error && std::filesystem::file_size(results_path, error) == 0);
  if (error) {
    std::cerr << color_text::YELLOW << "[GICP] Failed to inspect results file: " << error.message()
              << color_text::RESET << std::endl;
    results_recording_enabled_ = false;
    return;
  }

  results_stream_.open(results_path_, std::ios::out | std::ios::app);
  if (!results_stream_.is_open()) {
    std::cerr << color_text::YELLOW << "[GICP] Failed to open results file: " << results_path_
              << color_text::RESET << std::endl;
    results_recording_enabled_ = false;
    return;
  }

  if (write_header) {
    results_stream_ << "ros_time,cloud_time,state,stage,converged,quality_accepted,initial_guard_accepted,"
                       "stability_ready,stability_accepted,localized,time_ms,raw_score,normalized_score,"
                       "num_inliers,source_points,inlier_ratio,overlap_ratio,planar_min_eigenvalue,"
                       "planar_eigen_ratio,planar_yaw_scale,x,y,z,yaw,initial_dxy,initial_dyaw,"
                       "stability_xy,stability_yaw\n";
    results_stream_.flush();
  }

  std::cout << color_text::BLUE << "[GICP] Results recording enabled: " << results_path_
            << color_text::RESET << std::endl;
}

void GicpRosInterface::recordAlignmentResult(const std::string & stage,
  const GicpFilter::Result & result,
  const bool quality_accepted,
  const bool initial_guard_accepted,
  const bool stability_ready,
  const bool stability_accepted,
  const bool localized,
  const double time_ms,
  const double initial_dxy,
  const double initial_dyaw,
  const double stability_xy,
  const double stability_yaw)
{
  if (!results_recording_enabled_ || !results_stream_.is_open()) {
    return;
  }

  const char * state_name = "unknown";
  switch (state_) {
  case State::UNINITIALIZED:
    state_name = "uninitialized";
    break;
  case State::INITIALIZING:
    state_name = "initializing";
    break;
  case State::CONVERGING:
    state_name = "converging";
    break;
  case State::LOCALIZED:
    state_name = "localized";
    break;
  }

  const Eigen::Matrix4f & pose = result.final_transformation;
  results_stream_ << std::fixed << std::setprecision(9) << this->now().seconds() << ','
                  << last_cloud_stamp_.seconds() << ',' << state_name << ',' << stage << ','
                  << (result.converged ? 1 : 0) << ',' << (quality_accepted ? 1 : 0) << ','
                  << (initial_guard_accepted ? 1 : 0) << ',' << (stability_ready ? 1 : 0) << ','
                  << (stability_accepted ? 1 : 0) << ',' << (localized ? 1 : 0) << ',' << time_ms << ','
                  << result.score << ',' << result.normalized_score << ',' << result.num_inliers << ','
                  << result.source_points << ',' << result.inlier_ratio << ',' << result.overlap_ratio
                  << ',' << result.planar_min_eigenvalue << ',' << result.planar_eigen_ratio << ','
                  << result.planar_yaw_scale << ',' << pose(0, 3) << ',' << pose(1, 3) << ',' << pose(2, 3)
                  << ',' << poseYaw(pose) << ',' << initial_dxy << ',' << initial_dyaw << ','
                  << stability_xy << ',' << stability_yaw << '\n';
  results_stream_.flush();
}

GicpFilter::Result GicpRosInterface::runTwoStageAlignment(const PointCloud::Ptr & source_cloud,
  const Eigen::Matrix4f & initial_guess,
  double & time_ms,
  std::string & final_stage)
{
  const auto start_time = std::chrono::high_resolution_clock::now();
  GicpFilter::Result result = gicp_filter_->align(source_cloud, initial_guess);
  final_stage = "coarse";

  if (fine_alignment_enabled_ && result.converged && result.final_transformation.allFinite()) {
    result =
      gicp_filter_->align(source_cloud, result.final_transformation, fine_max_correspondence_distance_);
    final_stage = "fine";
  }

  const auto end_time = std::chrono::high_resolution_clock::now();
  time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  return result;
}

bool GicpRosInterface::isInitialCorrectionAccepted(
  const Eigen::Matrix4f & candidate, double & translation_correction, double & yaw_correction) const
{
  translation_correction = std::numeric_limits<double>::quiet_NaN();
  yaw_correction = std::numeric_limits<double>::quiet_NaN();
  if (!initial_pose_guard_enabled_ || !has_initial_reference_pose_ || has_localized_once_) {
    return true;
  }

  const double dx = static_cast<double>(candidate(0, 3) - initial_reference_pose_(0, 3));
  const double dy = static_cast<double>(candidate(1, 3) - initial_reference_pose_(1, 3));
  translation_correction = std::hypot(dx, dy);
  yaw_correction = std::abs(wrappedYawDifference(poseYaw(candidate), poseYaw(initial_reference_pose_)));
  return translation_correction <= max_initial_translation_correction_ &&
         yaw_correction <= max_initial_yaw_correction_;
}

bool GicpRosInterface::evaluatePoseStability(
  Eigen::Matrix4f & representative, double & max_xy_spread, double & max_yaw_spread) const
{
  max_xy_spread = 0.0;
  max_yaw_spread = 0.0;
  if (convergence_poses_.size() < static_cast<std::size_t>(converged_count_threshold_)) {
    return false;
  }

  for (std::size_t i = 0; i < convergence_poses_.size(); ++i) {
    for (std::size_t j = i + 1; j < convergence_poses_.size(); ++j) {
      const double dx = static_cast<double>(convergence_poses_[i](0, 3) - convergence_poses_[j](0, 3));
      const double dy = static_cast<double>(convergence_poses_[i](1, 3) - convergence_poses_[j](1, 3));
      max_xy_spread = std::max(max_xy_spread, std::hypot(dx, dy));
      max_yaw_spread = std::max(max_yaw_spread,
        std::abs(wrappedYawDifference(poseYaw(convergence_poses_[i]), poseYaw(convergence_poses_[j]))));
    }
  }

  std::size_t medoid_index = 0;
  double best_distance_sum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < convergence_poses_.size(); ++i) {
    double distance_sum = 0.0;
    for (std::size_t j = 0; j < convergence_poses_.size(); ++j) {
      const double dx = static_cast<double>(convergence_poses_[i](0, 3) - convergence_poses_[j](0, 3));
      const double dy = static_cast<double>(convergence_poses_[i](1, 3) - convergence_poses_[j](1, 3));
      const double xy_distance = std::hypot(dx, dy) / max_stability_xy_spread_;
      const double yaw_distance =
        std::abs(wrappedYawDifference(poseYaw(convergence_poses_[i]), poseYaw(convergence_poses_[j]))) /
        max_stability_yaw_spread_;
      distance_sum += xy_distance + yaw_distance;
    }
    if (distance_sum < best_distance_sum) {
      best_distance_sum = distance_sum;
      medoid_index = i;
    }
  }
  representative = convergence_poses_[medoid_index];

  return max_xy_spread <= max_stability_xy_spread_ && max_yaw_spread <= max_stability_yaw_spread_;
}

bool GicpRosInterface::isAlignmentAccepted(const GicpFilter::Result & result) const
{
  const bool raw_score_ok = score_threshold_ <= 0.0 || result.score <= score_threshold_;
  const bool planar_observability_ok =
    !planar_observability_check_enabled_ ||
    (std::isfinite(result.planar_eigen_ratio) && result.planar_eigen_ratio >= min_planar_eigen_ratio_);
  return result.converged && result.final_transformation.allFinite() && result.num_inliers > 0 &&
         result.source_points > 0 && std::isfinite(result.score) &&
         std::isfinite(result.normalized_score) && std::isfinite(result.inlier_ratio) &&
         std::isfinite(result.overlap_ratio) && raw_score_ok &&
         result.normalized_score <= normalized_score_threshold_ &&
         result.inlier_ratio >= min_inlier_ratio_ && result.overlap_ratio >= min_overlap_ratio_ &&
         planar_observability_ok;
}

void GicpRosInterface::runFSM()
{
  // 检查当前状态是否正在进行重定位
  if ((state_ == State::INITIALIZING || state_ == State::CONVERGING) && !has_localized_once_) {
    auto now = std::chrono::steady_clock::now();  // 获取当前时间
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - reloc_start_time_)
                     .count();  // 计算时间差（秒）

    // 如果超过设定的超时时间
    if (elapsed > timeout_seconds_) {
      std::cout << color_text::YELLOW << "[GICP] Relocalization timeout after " << elapsed
                << " s (threshold: " << timeout_seconds_ << " s), fallback to default pose"
                << color_text::RESET << std::endl;

      publishDefaultPose();
      convergence_poses_.clear();

      // 进入定位完成状态
      state_ = State::LOCALIZED;

      // 停止计算以节省资源
      if (fsm_timer_) {
        fsm_timer_->cancel();
      }
      if (lidar_sub_) {
        lidar_sub_.reset();
      }

      // 清空累积的点云
      {
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        accumulated_cloud_->clear();
        current_accumulated_frames_ = 0;
      }

      return;  // 直接返回，不再执行后续的重定位逻辑
    }
  }

  PointCloud::Ptr source_cloud(new PointCloud());
  {
    std::lock_guard<std::mutex> lock(cloud_mtx_);

    // 检查是否有足够的点云帧进行累积
    if (current_accumulated_frames_ < accumulate_frames_) {
      return;
    }

    *source_cloud = *accumulated_cloud_;
    *current_source_cloud_ = *accumulated_cloud_;

    // 提取后立即清空，避免锁覆盖后续耗时配准过程
    accumulated_cloud_->clear();
    current_accumulated_frames_ = 0;
  }

  if (source_cloud->empty() || source_cloud->size() < 100) {
    std::cout << color_text::YELLOW << "[GICP] Accumulated cloud is empty or too small ("
              << source_cloud->size() << " points)." << color_text::RESET << std::endl;
    return;
  }

  // 状态机逻辑
  switch (state_) {
  case State::UNINITIALIZED: {
    if (mode_ == Mode::MULTI_GUESS) {
      has_initial_reference_pose_ = false;
      convergence_poses_.clear();
      converged_count_ = 0;
      state_ = State::INITIALIZING;
    } else if (mode_ == Mode::INITIAL_GUESS) {
      std::cout << color_text::BLUE << "[GICP] Initializing from parameter pose..." << color_text::RESET
                << std::endl;

      // Initial pose from param (map -> camera_init)
      Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();
      initial_pose.block<3, 1>(0, 3) =
        Eigen::Vector3f(initial_pose_[0], initial_pose_[1], initial_pose_[2]);

      tf2::Quaternion q;
      q.setRPY(initial_pose_[3], initial_pose_[4], initial_pose_[5]);
      Eigen::Quaternionf q_eigen(q.w(), q.x(), q.y(), q.z());
      initial_pose.block<3, 3>(0, 0) = q_eigen.toRotationMatrix();

      {
        std::lock_guard<std::mutex> lock(pose_mtx_);
        map_to_camera_init_ = initial_pose;
      }

      initial_reference_pose_ = initial_pose;
      has_initial_reference_pose_ = true;
      convergence_poses_.clear();
      gicp_initialized_ = true;
      converged_count_ = 0;
      state_ = State::CONVERGING;
    }
    break;
  }

  case State::INITIALIZING: {
    std::cout << color_text::BLUE << "[GICP] Starting initial alignment (MULTI_GUESS)..."
              << color_text::RESET << std::endl;

    const auto start_time = std::chrono::high_resolution_clock::now();
    auto result = gicp_filter_->initialAlign(current_source_cloud_, min_inlier_ratio_);
    std::string final_stage = "multi_guess";
    if (fine_alignment_enabled_ && result.converged && result.final_transformation.allFinite()) {
      result = gicp_filter_->align(
        current_source_cloud_, result.final_transformation, fine_max_correspondence_distance_);
      final_stage = "multi_guess_fine";
    }
    const auto end_time = std::chrono::high_resolution_clock::now();
    const double time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    const bool quality_accepted = isAlignmentAccepted(result);

    if (quality_accepted) {
      std::cout << color_text::GREEN << "[GICP] Initial alignment accepted: raw_score=" << result.score
                << ", normalized_score=" << result.normalized_score << ", inliers=" << result.num_inliers
                << "/" << result.source_points << ", inlier_ratio=" << result.inlier_ratio
                << ", overlap=" << result.overlap_ratio << ", planar_ratio=" << result.planar_eigen_ratio
                << color_text::RESET << std::endl;

      // GICP 计算的是 Source(camera_init) -> Target(map) 的变换
      {
        std::lock_guard<std::mutex> lock(pose_mtx_);
        map_to_camera_init_ = result.final_transformation;
      }

      gicp_initialized_ = true;
      last_icp_time_ = this->now();
      converged_count_ = 0;
      convergence_poses_.clear();
      recordAlignmentResult(final_stage,
        result,
        true,
        true,
        false,
        false,
        false,
        time_ms,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0);
      state_ = State::CONVERGING;
    } else {
      std::cout << color_text::YELLOW
                << "[GICP] Initial alignment rejected: converged=" << (result.converged ? "true" : "false")
                << ", raw_score=" << result.score << ", normalized_score=" << result.normalized_score
                << " (required<=" << normalized_score_threshold_ << "), inliers=" << result.num_inliers
                << "/" << result.source_points << ", inlier_ratio=" << result.inlier_ratio
                << " (required>=" << min_inlier_ratio_ << "), overlap=" << result.overlap_ratio
                << " (required>=" << min_overlap_ratio_ << "), planar_ratio=" << result.planar_eigen_ratio
                << ". Retrying..." << color_text::RESET << std::endl;
      recordAlignmentResult(final_stage,
        result,
        false,
        true,
        false,
        false,
        false,
        time_ms,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.0);
      state_ = State::UNINITIALIZED;
    }
    break;
  }
  case State::CONVERGING: {
    if (!gicp_initialized_) {
      state_ = State::UNINITIALIZED;
      break;
    }

    std::cout << color_text::BLUE << "[GICP] Verifying convergence (Count: " << converged_count_ << "/"
              << converged_count_threshold_ << ")..." << color_text::RESET << std::endl;

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(pose_mtx_);
      initial_guess = map_to_camera_init_;
    }

    double time_ms = 0.0;
    std::string final_stage;
    auto result = runTwoStageAlignment(current_source_cloud_, initial_guess, time_ms, final_stage);
    const bool quality_accepted = isAlignmentAccepted(result);
    double initial_dxy = std::numeric_limits<double>::quiet_NaN();
    double initial_dyaw = std::numeric_limits<double>::quiet_NaN();
    const bool initial_guard_accepted =
      isInitialCorrectionAccepted(result.final_transformation, initial_dxy, initial_dyaw);

    if (quality_accepted && initial_guard_accepted) {
      if (has_localized_once_ && has_last_successful_pose_) {
        const Eigen::Matrix4f prev = last_successful_pose_;
        const Eigen::Matrix4f candidate = result.final_transformation;

        const double dx = static_cast<double>(candidate(0, 3) - prev(0, 3));
        const double dy = static_cast<double>(candidate(1, 3) - prev(1, 3));
        const double trans_jump = std::hypot(dx, dy);

        const double prev_yaw =
          std::atan2(static_cast<double>(prev(1, 0)), static_cast<double>(prev(0, 0)));
        const double cand_yaw =
          std::atan2(static_cast<double>(candidate(1, 0)), static_cast<double>(candidate(0, 0)));
        const double yaw_jump = std::atan2(std::sin(cand_yaw - prev_yaw), std::cos(cand_yaw - prev_yaw));

        if (trans_jump > max_translation_jump_ || std::abs(yaw_jump) > max_yaw_jump_) {
          std::cout << color_text::YELLOW
                    << "[GICP] Reject in CONVERGING due to jump from last successful pose: dxy="
                    << trans_jump << "m (limit=" << max_translation_jump_ << "), dyaw=" << yaw_jump
                    << "rad (limit=" << max_yaw_jump_ << ")" << color_text::RESET << std::endl;
          recordAlignmentResult(final_stage,
            result,
            true,
            true,
            false,
            false,
            false,
            time_ms,
            initial_dxy,
            initial_dyaw,
            0.0,
            0.0);
          converged_count_ = 0;
          convergence_poses_.clear();
          state_ = State::UNINITIALIZED;
          gicp_initialized_ = false;
          break;
        }
      }

      gicp_utils::printEvaluation(initial_guess, result.final_transformation, time_ms);

      std::cout << color_text::GREEN << "[GICP] Converged: raw_score=" << result.score
                << ", normalized_score=" << result.normalized_score << ", inliers=" << result.num_inliers
                << "/" << result.source_points << ", inlier_ratio=" << result.inlier_ratio
                << ", overlap=" << result.overlap_ratio << ", planar_ratio=" << result.planar_eigen_ratio
                << color_text::RESET << std::endl;

      convergence_poses_.push_back(result.final_transformation);
      while (convergence_poses_.size() > static_cast<std::size_t>(converged_count_threshold_)) {
        convergence_poses_.pop_front();
      }
      converged_count_ = static_cast<int>(convergence_poses_.size());

      Eigen::Matrix4f representative = result.final_transformation;
      double stability_xy = 0.0;
      double stability_yaw = 0.0;
      const bool stability_ready =
        convergence_poses_.size() >= static_cast<std::size_t>(converged_count_threshold_);
      const bool stability_accepted =
        stability_ready && evaluatePoseStability(representative, stability_xy, stability_yaw);

      if (stability_accepted) {
        {
          std::lock_guard<std::mutex> lock(pose_mtx_);
          map_to_camera_init_ = representative;
        }
        std::cout << color_text::GREEN << "[GICP] Localization confirmed from " << converged_count_
                  << " independent windows: max_dxy=" << stability_xy << "m, max_dyaw=" << stability_yaw
                  << "rad" << color_text::RESET << std::endl;
        state_ = State::LOCALIZED;
        has_localized_once_ = true;
        has_last_successful_pose_ = true;
        {
          std::lock_guard<std::mutex> lock(pose_mtx_);
          last_successful_pose_ = map_to_camera_init_;
        }

        // Publish TF immediately, then keep periodic publish by tf timer
        Eigen::Matrix4f pose_snapshot = Eigen::Matrix4f::Identity();
        {
          std::lock_guard<std::mutex> lock(pose_mtx_);
          pose_snapshot = map_to_camera_init_;
        }
        gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, pose_snapshot, this->now());
        if (!enable_continuous_relocalization_) {
          // Suspend operations to save resources
          std::cout << color_text::BLUE << "[GICP] Suspending GICP update timer and lidar subscription."
                    << color_text::RESET << std::endl;
          if (fsm_timer_) {
            fsm_timer_->cancel();
          }
          // Reset subscriber to stop receiving data completely
          if (lidar_sub_) {
            lidar_sub_.reset();
          }
        } else {
          tracking_lost_count_ = 0;
        }
      } else {
        if (stability_ready) {
          std::cout << color_text::YELLOW << "[GICP] Pose windows are not stable: max_dxy=" << stability_xy
                    << "m (limit=" << max_stability_xy_spread_ << "), max_dyaw=" << stability_yaw
                    << "rad (limit=" << max_stability_yaw_spread_ << "). Sliding the validation window."
                    << color_text::RESET << std::endl;
          convergence_poses_.pop_front();
          converged_count_ = static_cast<int>(convergence_poses_.size());
        }
        {
          std::lock_guard<std::mutex> lock(pose_mtx_);
          map_to_camera_init_ = result.final_transformation;
        }
      }
      GicpFilter::Result recorded_result = result;
      if (stability_accepted) {
        recorded_result.final_transformation = representative;
      }
      recordAlignmentResult(final_stage,
        recorded_result,
        true,
        true,
        stability_ready,
        stability_accepted,
        stability_accepted,
        time_ms,
        initial_dxy,
        initial_dyaw,
        stability_xy,
        stability_yaw);
    } else {
      if (!quality_accepted) {
        std::cout << color_text::YELLOW
                  << "[GICP] Rejected: converged=" << (result.converged ? "true" : "false")
                  << ", raw_score=" << result.score << ", normalized_score=" << result.normalized_score
                  << " (required<=" << normalized_score_threshold_ << "), inliers=" << result.num_inliers
                  << "/" << result.source_points << ", inlier_ratio=" << result.inlier_ratio
                  << " (required>=" << min_inlier_ratio_ << "), overlap=" << result.overlap_ratio
                  << " (required>=" << min_overlap_ratio_ << "), planar_ratio=" << result.planar_eigen_ratio
                  << ". Resetting validation window." << color_text::RESET << std::endl;
      } else {
        std::cout << color_text::YELLOW << "[GICP] Rejected by initial pose guard: dxy=" << initial_dxy
                  << "m (limit=" << max_initial_translation_correction_ << "), dyaw=" << initial_dyaw
                  << "rad (limit=" << max_initial_yaw_correction_ << ")" << color_text::RESET << std::endl;
      }
      recordAlignmentResult(final_stage,
        result,
        quality_accepted,
        initial_guard_accepted,
        false,
        false,
        false,
        time_ms,
        initial_dxy,
        initial_dyaw,
        0.0,
        0.0);
      converged_count_ = 0;
      convergence_poses_.clear();
      state_ = State::UNINITIALIZED;
      gicp_initialized_ = false;
    }
    break;
  }
  case State::LOCALIZED: {
    if (!enable_continuous_relocalization_) {
      break;
    }

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(pose_mtx_);
      initial_guess = map_to_camera_init_;
    }
    double time_ms = 0.0;
    std::string final_stage;
    auto result = runTwoStageAlignment(current_source_cloud_, initial_guess, time_ms, final_stage);
    const bool quality_accepted = isAlignmentAccepted(result);
    bool jump_accepted = false;

    if (quality_accepted) {
      const Eigen::Matrix4f prev = has_last_successful_pose_ ? last_successful_pose_ : map_to_camera_init_;
      const Eigen::Matrix4f candidate = result.final_transformation;

      const double dx = static_cast<double>(candidate(0, 3) - prev(0, 3));
      const double dy = static_cast<double>(candidate(1, 3) - prev(1, 3));
      const double trans_jump = std::hypot(dx, dy);

      const double prev_yaw = std::atan2(static_cast<double>(prev(1, 0)), static_cast<double>(prev(0, 0)));
      const double cand_yaw =
        std::atan2(static_cast<double>(candidate(1, 0)), static_cast<double>(candidate(0, 0)));
      const double yaw_jump = std::atan2(std::sin(cand_yaw - prev_yaw), std::cos(cand_yaw - prev_yaw));

      if (trans_jump <= max_translation_jump_ && std::abs(yaw_jump) <= max_yaw_jump_) {
        jump_accepted = true;
        {
          std::lock_guard<std::mutex> lock(pose_mtx_);
          map_to_camera_init_ = candidate;
        }
        has_last_successful_pose_ = true;
        last_successful_pose_ = candidate;
        tracking_lost_count_ = 0;
      } else {
        std::cout << color_text::YELLOW << "[GICP] Reject large jump in LOCALIZED: dxy=" << trans_jump
                  << "m (limit=" << max_translation_jump_ << "), dyaw=" << yaw_jump
                  << "rad (limit=" << max_yaw_jump_ << ")" << color_text::RESET << std::endl;
        tracking_lost_count_++;
      }
    } else {
      tracking_lost_count_++;
    }

    recordAlignmentResult(final_stage,
      result,
      quality_accepted,
      jump_accepted,
      false,
      false,
      quality_accepted && jump_accepted,
      time_ms,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      0.0);

    if (tracking_lost_count_ >= max_tracking_lost_count_) {
      std::cerr << color_text::RED << "[GICP] Tracking lost reached threshold, fallback to UNINITIALIZED."
                << color_text::RESET << std::endl;
      state_ = State::UNINITIALIZED;
      gicp_initialized_ = false;
      convergence_poses_.clear();
    }
    break;
  }

  default:
    break;
  }
}

}  // namespace icp_relocalization
