#include "gicp_ros_interface.hpp"
#include "gicp_utils.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
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
  overlap_threshold_ = this->declare_parameter<double>("overlap_threshold", 0.75);
  converged_count_threshold_ = this->declare_parameter<int>("converged_count_threshold", 5);
  enable_continuous_relocalization_ =
    this->declare_parameter<bool>("enable_continuous_relocalization", false);
  max_tracking_lost_count_ = this->declare_parameter<int>("max_tracking_lost_count", 3);
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
            << ", accumulate_frames=" << accumulate_frames_ << ", overlap_threshold=" << overlap_threshold_
            << ", target_pcd=" << target_pcd_file << color_text::RESET << std::endl;
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
  callback_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_service_ = callback_group_lidar_;

  activateLidarSubscription();
  rclcpp::SubscriptionOptions odom_sub_options;
  odom_sub_options.callback_group = callback_group_lidar_;
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
    callback_group_service_);
}

void GicpRosInterface::activateLidarSubscription()
{
  rclcpp::SubscriptionOptions lidar_sub_options;
  lidar_sub_options.callback_group = callback_group_lidar_;
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
    fsm_period_, std::bind(&GicpRosInterface::fsmTimerCallback, this), callback_group_lidar_);
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
    callback_group_lidar_);
}

void GicpRosInterface::startTfPublishTimer()
{
  if (tf_publish_timer_) {
    tf_publish_timer_->cancel();
    tf_publish_timer_.reset();
  }

  tf_publish_timer_ = this->create_wall_timer(
    tf_publish_period_, std::bind(&GicpRosInterface::tfPublishTimerCallback, this), callback_group_lidar_);
}

void GicpRosInterface::tfPublishTimerCallback()
{
  if (!gicp_initialized_) {
    return;
  }
  gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, map_to_camera_init_, this->now());
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
  current_accumulated_frames_ = 0;
  if (accumulated_cloud_)
    accumulated_cloud_->clear();
  if (current_source_cloud_)
    current_source_cloud_->clear();
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
  map_to_camera_init_ = default_transform;
  gicp_initialized_ = true;
  gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, map_to_camera_init_, this->now());
  std::cout << color_text::GREEN << "[GICP] Default pose published" << color_text::RESET << std::endl;
}

void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  PointCloud::Ptr temp_cloud(new PointCloud());
  pcl::fromROSMsg(*msg, *temp_cloud);

  if (current_accumulated_frames_ >= accumulate_frames_) {
    return;
  }

  *accumulated_cloud_ += *temp_cloud;
  current_accumulated_frames_++;
  last_cloud_stamp_ = msg->header.stamp;
  cloud_frame_id_ = msg->header.frame_id;

  // Reset if too large (safety)
  if (accumulated_cloud_->size() > 100000) {
    accumulated_cloud_->clear();
    current_accumulated_frames_ = 0;
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
    map_to_camera_init_,
    cloud_stamp,
    pub_source_cropped_);

  gicp_utils::publishTargetCroppedDebug(
    visualization_en_, map_frame_, cloud_stamp, gicp_filter_.get(), pub_target_cropped_);

  gicp_utils::publishVisualization(current_source_cloud_,
    cloud_stamp,
    visualization_en_,
    gicp_initialized_,
    map_frame_,
    map_to_camera_init_,
    pub_source_aligned_);
}

void GicpRosInterface::runFSM()
{
  // 检查当前状态是否正在进行重定位
  if (state_ == State::INITIALIZING || state_ == State::CONVERGING) {
    auto now = std::chrono::steady_clock::now();  // 获取当前时间
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - reloc_start_time_)
                     .count();  // 计算时间差（秒）

    // 如果超过设定的超时时间
    if (elapsed > timeout_seconds_) {
      std::cout << color_text::YELLOW << "[GICP] Relocalization timeout after " << elapsed
                << " s (threshold: " << timeout_seconds_ << " s), fallback to default pose"
                << color_text::RESET << std::endl;

      publishDefaultPose();

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
      accumulated_cloud_->clear();
      current_accumulated_frames_ = 0;

      return;  // 直接返回，不再执行后续的重定位逻辑
    }
  }

  // 检查是否有足够的点云帧进行累积
  if (current_accumulated_frames_ < accumulate_frames_) {
    return;
  }

  PointCloud::Ptr source_cloud(new PointCloud());
  *source_cloud = *accumulated_cloud_;
  *current_source_cloud_ = *accumulated_cloud_;

  if (source_cloud->empty() || source_cloud->size() < 100) {
    std::cout << color_text::YELLOW << "[GICP] Accumulated cloud is empty or too small ("
              << source_cloud->size() << " points)." << color_text::RESET << std::endl;
    accumulated_cloud_->clear();
    current_accumulated_frames_ = 0;
    return;
  }

  // 状态机逻辑
  switch (state_) {
  case State::UNINITIALIZED: {
    if (mode_ == Mode::MULTI_GUESS) {
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

      map_to_camera_init_ = initial_pose;

      gicp_initialized_ = true;
      converged_count_ = 0;
      state_ = State::CONVERGING;
    }
    break;
  }

  case State::INITIALIZING: {
    std::cout << color_text::BLUE << "[GICP] Starting initial alignment (MULTI_GUESS)..."
              << color_text::RESET << std::endl;

    auto result = gicp_filter_->initialAlign(current_source_cloud_);

    if (result.converged && result.overlap_ratio > overlap_threshold_) {
      std::cout << color_text::GREEN
                << "[GICP] Initial alignment accepted: overlap=" << result.overlap_ratio * 100.0
                << "% (threshold=" << overlap_threshold_ * 100.0 << "%)" << color_text::RESET << std::endl;

      // GICP 计算的是 Source(camera_init) -> Target(map) 的变换
      map_to_camera_init_ = result.final_transformation;

      gicp_initialized_ = true;
      last_icp_time_ = this->now();
      converged_count_ = 0;
      state_ = State::CONVERGING;
    } else {
      std::cout << color_text::YELLOW
                << "[GICP] Initial alignment rejected: converged=" << (result.converged ? "true" : "false")
                << ", overlap=" << result.overlap_ratio * 100.0 << "% (required>"
                << overlap_threshold_ * 100.0 << "%). Retrying..." << color_text::RESET << std::endl;
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

    Eigen::Matrix4f initial_guess = map_to_camera_init_;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto result = gicp_filter_->align(current_source_cloud_, initial_guess);
    auto end_time = std::chrono::high_resolution_clock::now();
    double time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (result.converged && result.overlap_ratio > overlap_threshold_) {
      gicp_utils::printEvaluation(initial_guess, result.final_transformation, time_ms);

      std::cout << color_text::GREEN << "[GICP] Converged: overlap=" << result.overlap_ratio * 100.0
                << "% (threshold=" << overlap_threshold_ * 100.0 << "%)" << color_text::RESET << std::endl;

      map_to_camera_init_ = result.final_transformation;

      converged_count_++;
      if (converged_count_ >= converged_count_threshold_) {
        std::cout << color_text::GREEN << "[GICP] Localization confirmed after " << converged_count_
                  << " consecutive convergences" << color_text::RESET << std::endl;
        state_ = State::LOCALIZED;

        // Publish TF immediately, then keep periodic publish by tf timer
        gicp_utils::publishCurrentTransform(tf_broadcaster_, map_frame_, map_to_camera_init_, this->now());
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
      }
    } else {
      std::cout << color_text::YELLOW
                << "[GICP] Rejected: converged=" << (result.converged ? "true" : "false")
                << ", overlap=" << result.overlap_ratio * 100.0 << "% (required>"
                << overlap_threshold_ * 100.0 << "%). Resetting count." << color_text::RESET << std::endl;
      converged_count_ = 0;
      state_ = State::UNINITIALIZED;
      gicp_initialized_ = false;
    }
    break;
  }
  case State::LOCALIZED: {
    if (!enable_continuous_relocalization_) {
      break;
    }

    Eigen::Matrix4f current_odom = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(odom_mtx_);
      current_odom = latest_odom_pose_;
    }

    Eigen::Matrix4f initial_guess = map_to_camera_init_ * current_odom;
    auto result = gicp_filter_->align(current_source_cloud_, initial_guess);

    if (result.converged && result.overlap_ratio > overlap_threshold_) {
      map_to_camera_init_ = result.final_transformation * current_odom.inverse();
      tracking_lost_count_ = 0;
    } else {
      tracking_lost_count_++;
      if (tracking_lost_count_ >= max_tracking_lost_count_) {
        std::cerr << color_text::RED << "[GICP] Tracking lost reached threshold, fallback to UNINITIALIZED."
                  << color_text::RESET << std::endl;
        state_ = State::UNINITIALIZED;
        gicp_initialized_ = false;
      }
    }
    break;
  }

  default:
    break;
  }

  // Clear accumulation for next batch after processing
  accumulated_cloud_->clear();
  current_accumulated_frames_ = 0;
}

}  // namespace icp_relocalization
