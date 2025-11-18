#include "gicp_ros_interface.hpp"

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace icp_relocalization
{

  GicpRosInterface::GicpRosInterface(const rclcpp::NodeOptions& options)
    : Node("gicp_relocalization_node", options)
  {
    // 参数获取
    this->declare_parameter<bool>("use_initial_alignment", true);
    this->declare_parameter<std::string>("target_pcd_file", "map.pcd");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("drift_threshold_m", 1.0);
    this->declare_parameter<double>("drift_threshold_rad", 0.5);
    this->declare_parameter<double>("icp_frequency", 1.0);  // 1Hz

    // GICP参数
    this->declare_parameter<double>("gicp.target_voxel_leaf_size", 2.0);
    this->declare_parameter<double>("gicp.source_voxel_leaf_size", 2.0);
    this->declare_parameter<double>("gicp.max_correspondence_distance", 10.0);
    this->declare_parameter<int>("gicp.max_iterations", 100);
    this->declare_parameter<double>("gicp.transformation_epsilon", 0.01);
    this->declare_parameter<double>("gicp.euclidean_fitness_epsilon", 0.01);

    // SAC-IA参数
    this->declare_parameter<double>("sac_ia.min_sample_distance", 0.5);
    this->declare_parameter<int>("sac_ia.correspondence_randomness", 6);
    this->declare_parameter<int>("sac_ia.num_samples", 3);
    this->declare_parameter<double>("sac_ia.max_correspondence_distance", 1.0);
    this->declare_parameter<int>("feature_k_search", 10);

    // 读取参数
    use_initial_alignment_ = this->get_parameter("use_initial_alignment").as_bool();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    drift_threshold_m_ = this->get_parameter("drift_threshold_m").as_double();
    drift_threshold_rad_ = this->get_parameter("drift_threshold_rad").as_double();
    icp_frequency_ = this->get_parameter("icp_frequency").as_double();

    gicp_options_.target_voxel_leaf_size = this->get_parameter("gicp.target_voxel_leaf_size").as_double();
    gicp_options_.source_voxel_leaf_size = this->get_parameter("gicp.source_voxel_leaf_size").as_double();
    gicp_options_.max_correspondence_distance = this->get_parameter("gicp.max_correspondence_distance").as_double();
    gicp_options_.max_iterations = this->get_parameter("gicp.max_iterations").as_int();
    gicp_options_.transformation_epsilon = this->get_parameter("gicp.transformation_epsilon").as_double();
    gicp_options_.euclidean_fitness_epsilon = this->get_parameter("gicp.euclidean_fitness_epsilon").as_double();

    gicp_options_.sac_ia_min_sample_distance = this->get_parameter("sac_ia.min_sample_distance").as_double();
    gicp_options_.sac_ia_correspondence_randomness = this->get_parameter("sac_ia.correspondence_randomness").as_int();
    gicp_options_.sac_ia_num_samples = this->get_parameter("sac_ia.num_samples").as_int();
    gicp_options_.sac_ia_max_correspondence_distance =
        this->get_parameter("sac_ia.max_correspondence_distance").as_double();
    gicp_options_.feature_k_search = this->get_parameter("feature_k_search").as_int();

    if(!use_initial_alignment_)
    {
      state_ = State::LOCALIZED;  // 如果不使用SAC-IA，则直接进入定位状态，依赖里程计初始化
      RCLCPP_INFO(this->get_logger(), "Initial alignment disabled. Relying on first odometry for initial pose.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Initial alignment enabled. Waiting for first lidar scan to initialize.");
    }

    setupGicp();

    // ROS接口初始化
    callback_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lidar_sub_options;
    lidar_sub_options.callback_group = callback_group_lidar_;

    callback_group_odom_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_sub_options;
    odom_sub_options.callback_group = callback_group_odom_;

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar",
        rclcpp::SensorDataQoS(),
        std::bind(&GicpRosInterface::lidarCallback, this, std::placeholders::_1),
        lidar_sub_options);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init",
        rclcpp::QoS(100),
        std::bind(&GicpRosInterface::odomCallback, this, std::placeholders::_1),
        odom_sub_options);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gicp_pose", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_map", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 发布一次地图
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*gicp_filter_->getTargetCloud(), map_msg);
    map_msg.header.frame_id = map_frame_;
    map_pub_->publish(map_msg);
  }

  void GicpRosInterface::setupGicp()
  {
    std::string target_pcd_file = this->get_parameter("target_pcd_file").as_string();
    try
    {
      gicp_filter_ = std::make_unique<GicpFilter>(target_pcd_file, gicp_options_);
      RCLCPP_INFO(this->get_logger(), "GICP filter initialized with map: %s", target_pcd_file.c_str());
    }
    catch(const std::runtime_error& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize GICP filter: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void GicpRosInterface::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
    // 如果不使用SAC-IA，并且尚未初始化，则使用第一帧里程计进行初始化
    if(state_ == State::LOCALIZED && !gicp_initialized_)
    {
      Eigen::Isometry3d initial_pose_iso;
      tf2::fromMsg(msg->pose.pose, initial_pose_iso);
      last_icp_pose_ = initial_pose_iso.matrix().cast<float>();
      gicp_initialized_ = true;
      last_icp_time_ = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "ICP initialized with first odometry pose.");
    }
  }

  void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloud::Ptr source_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *source_cloud);

    if(state_ == State::UNINITIALIZED)
    {
      state_ = State::INITIALIZING;
      RCLCPP_INFO(this->get_logger(), "Starting initial alignment (SAC-IA)...");

      auto result = gicp_filter_->initialAlign(source_cloud);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "Initial alignment successful! Fitness score: %f", result.fitness_score);
        last_icp_pose_ = result.final_transformation;
        gicp_initialized_ = true;
        last_icp_time_ = this->now();
        state_ = State::LOCALIZED;
        publishPose(last_icp_pose_, this->now());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Initial alignment failed. Retrying on next scan...");
        state_ = State::UNINITIALIZED;  // Reset to try again
      }
      return;
    }

    if(state_ == State::LOCALIZED)
    {
      if(!gicp_initialized_ || !last_odom_)
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial odometry or initial alignment...");
        return;
      }

      if((this->now() - last_icp_time_).seconds() < 1.0 / icp_frequency_)
      {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Processing new lidar scan for incremental alignment...");
      last_icp_time_ = this->now();

      Eigen::Isometry3d initial_guess_iso;
      tf2::fromMsg(last_odom_->pose.pose, initial_guess_iso);
      Eigen::Matrix4f initial_guess = initial_guess_iso.matrix().cast<float>();

      auto result = gicp_filter_->align(source_cloud, initial_guess);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "GICP converged with fitness score: %f", result.fitness_score);
        checkDriftAndCorrect(result.final_transformation);
        last_icp_pose_ = result.final_transformation;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "GICP failed to converge.");
        // 可以在这里添加重定位逻辑，例如切换回UNINITIALIZED状态
        // state_ = State::UNINITIALIZED;
      }
    }
  }

  void GicpRosInterface::checkDriftAndCorrect(const Eigen::Matrix4f& icp_pose)
  {
    Eigen::Vector3f icp_pos = icp_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf icp_quat(icp_pose.block<3, 3>(0, 0));

    Eigen::Vector3f odom_pos;
    odom_pos.x() = last_odom_->pose.pose.position.x;
    odom_pos.y() = last_odom_->pose.pose.position.y;
    odom_pos.z() = last_odom_->pose.pose.position.z;
    Eigen::Quaternionf odom_quat(last_odom_->pose.pose.orientation.w,
                                 last_odom_->pose.pose.orientation.x,
                                 last_odom_->pose.pose.orientation.y,
                                 last_odom_->pose.pose.orientation.z);

    float pos_diff = (icp_pos - odom_pos).norm();
    float angle_diff = icp_quat.angularDistance(odom_quat);

    RCLCPP_INFO(this->get_logger(), "ICP/Odom diff: pos=%.3fm, angle=%.3frad", pos_diff, angle_diff);

    if(pos_diff > drift_threshold_m_ || angle_diff > drift_threshold_rad_)
    {
      RCLCPP_WARN(this->get_logger(), "Significant drift detected! Resetting odometry integration base.");
      // 响应机制：这里我们选择信任ICP结果，并用它来重置我们的连续位姿估计基准
      // 在更复杂的系统中，可能需要更复杂的融合策略或状态重置
      // 简单起见，我们直接将ICP结果作为最新的有效位姿
    }

    // 无论是否漂移，都发布ICP的结果作为更正后的位姿
    publishPose(icp_pose, this->now());
  }

  void GicpRosInterface::publishPose(const Eigen::Matrix4f& pose, const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;

    Eigen::Vector3f pos = pose.block<3, 1>(0, 3);
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));

    pose_msg.pose.pose.position.x = pos.x();
    pose_msg.pose.pose.position.y = pos.y();
    pose_msg.pose.pose.position.z = pos.z();
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();

    // 暂时不填充协方差
    pose_pub_->publish(pose_msg);

    // 发布TF
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = base_frame_;
    transform_stamped.transform.translation.x = pos.x();
    transform_stamped.transform.translation.y = pos.y();
    transform_stamped.transform.translation.z = pos.z();
    transform_stamped.transform.rotation = pose_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform_stamped);
  }

}  // namespace icp_relocalization
