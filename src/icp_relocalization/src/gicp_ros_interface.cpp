#include "gicp_ros_interface.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace icp_relocalization
{
  using namespace color_text;
  GicpRosInterface::GicpRosInterface(const rclcpp::NodeOptions& options)
    : Node("gicp_relocalization_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Loading parameters from YAML file...");
    // Default parameters
    use_initial_alignment_ = this->declare_parameter<bool>("use_initial_alignment", true);
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    drift_threshold_m_ = this->declare_parameter<double>("drift_threshold_m", 1.0);
    drift_threshold_rad_ = this->declare_parameter<double>("drift_threshold_rad", 0.5);
    alignment_frequency_ = this->declare_parameter<double>("alignment_frequency", 1.0);
    publish_pose_on_odom_ = this->declare_parameter<bool>("publish_pose_on_odom", true);
    accumulate_frames_ = this->declare_parameter<int>("accumulate_frames", 5);
    std::string target_pcd_file = this->declare_parameter<std::string>("target_pcd_file", "map.pcd");

    // Map Offset Parameters
    map_offset_ = this->declare_parameter<std::vector<double>>("map_offset", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // GICP Parameters
    gicp_options_.feature_k_search = this->declare_parameter<int>("feature_k_search", 10);
    gicp_options_.target_voxel_leaf_size = this->declare_parameter<double>("gicp.target_voxel_leaf_size", 2.0);
    gicp_options_.source_voxel_leaf_size = this->declare_parameter<double>("gicp.source_voxel_leaf_size", 2.0);
    gicp_options_.max_correspondence_distance =
        this->declare_parameter<double>("gicp.max_correspondence_distance", 10.0);
    gicp_options_.max_iterations = this->declare_parameter<int>("gicp.max_iterations", 100);
    gicp_options_.transformation_epsilon = this->declare_parameter<double>("gicp.transformation_epsilon", 0.01);
    gicp_options_.euclidean_fitness_epsilon = this->declare_parameter<double>("gicp.euclidean_fitness_epsilon", 0.01);

    // SAC-IA Parameters
    gicp_options_.sac_ia_min_sample_distance = this->declare_parameter<double>("sac_ia.min_sample_distance", 0.5);
    gicp_options_.sac_ia_correspondence_randomness =
        this->declare_parameter<int>("sac_ia.correspondence_randomness", 6);
    gicp_options_.sac_ia_num_samples = this->declare_parameter<int>("sac_ia.num_samples", 3);
    gicp_options_.sac_ia_max_correspondence_distance =
        this->declare_parameter<double>("sac_ia.max_correspondence_distance", 1.0);

    std::cout << BOLDCYAN << " ========== GICP Relocalization ==========" << RESET << std::endl;
    LOG_DEBUG_BLOCK(std::string(CYAN) + "[RELOCALIZATION] ",
                    NV(use_initial_alignment_),
                    NV(base_frame_),
                    NV(map_frame_),
                    NV(drift_threshold_m_),
                    NV(drift_threshold_rad_),
                    NV(alignment_frequency_),
                    NV(publish_pose_on_odom_),
                    NV(accumulate_frames_),
                    NV(target_pcd_file));

    if(map_offset_.size() >= 6)
    {
      std::cout << BOLDCYAN << "  ---------- Map Offset ----------" << RESET << std::endl;
      LOG_DEBUG_BLOCK(std::string(CYAN) + "[OFFSET] ",
                      NV(map_offset_[0]),
                      NV(map_offset_[1]),
                      NV(map_offset_[2]),
                      NV(map_offset_[3]),
                      NV(map_offset_[4]),
                      NV(map_offset_[5]));
    }

    std::cout << BOLDCYAN << "  ---------- GICP Options ----------" << RESET << std::endl;
    LOG_DEBUG_BLOCK(std::string(CYAN) + "[GICP] ",
                    NV(gicp_options_.target_voxel_leaf_size),
                    NV(gicp_options_.source_voxel_leaf_size),
                    NV(gicp_options_.max_correspondence_distance),
                    NV(gicp_options_.max_iterations),
                    NV(gicp_options_.transformation_epsilon),
                    NV(gicp_options_.euclidean_fitness_epsilon));
    std::cout << BOLDCYAN << "  ----------SAC-IA Options----------" << RESET << std::endl;
    LOG_DEBUG_BLOCK(std::string(CYAN) + "[SAC-IA] ",
                    NV(gicp_options_.sac_ia_min_sample_distance),
                    NV(gicp_options_.sac_ia_correspondence_randomness),
                    NV(gicp_options_.sac_ia_num_samples),
                    NV(gicp_options_.sac_ia_max_correspondence_distance),
                    NV(gicp_options_.feature_k_search));
    if(!use_initial_alignment_)
    {
      state_ = State::LOCALIZED;  // 如果不使用SAC-IA，则直接进入定位状态，依赖里程计初始化
      RCLCPP_INFO(this->get_logger(), "Initial alignment disabled. Relying on first odometry for initial pose.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Initial alignment enabled. Waiting for first lidar scan to initialize.");
    }

    setupGicp(target_pcd_file);

    // ROS接口初始化
    callback_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lidar_sub_options;
    lidar_sub_options.callback_group = callback_group_lidar_;

    callback_group_odom_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_sub_options;
    odom_sub_options.callback_group = callback_group_odom_;

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered",
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
    source_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_source", 10);  // Debug: accumulated cloud
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 发布一次地图
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*gicp_filter_->getTargetCloud(), map_msg);
    map_msg.header.frame_id = map_frame_;
    map_pub_->publish(map_msg);

    // 初始化FSM定时器
    double period = 1.0 / alignment_frequency_;
    fsm_timer_ = this->create_wall_timer(std::chrono::duration<double>(period),
                                         std::bind(&GicpRosInterface::fsmTimerCallback, this),
                                         callback_group_lidar_);
  }

  void GicpRosInterface::setupGicp(const std::string& target_pcd_file)
  {
    try
    {
      // 1. 加载点云
      PointCloud::Ptr target_cloud(new PointCloud());
      if(pcl::io::loadPCDFile<pcl::PointXYZ>(target_pcd_file, *target_cloud) == -1)
      {
        throw std::runtime_error("Couldn't read file " + target_pcd_file);
      }

      // 2. 应用地图偏移
      if(map_offset_.size() == 6)
      {
        bool is_zero = true;
        for(double v : map_offset_)
          if(std::abs(v) > 1e-6)
            is_zero = false;

        if(!is_zero)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Applying map offset: xyz(%.2f, %.2f, %.2f), rpy(%.2f, %.2f, %.2f)",
                      map_offset_[0],
                      map_offset_[1],
                      map_offset_[2],
                      map_offset_[3],
                      map_offset_[4],
                      map_offset_[5]);

          Eigen::Affine3f transform = Eigen::Affine3f::Identity();
          transform.translation() << map_offset_[0], map_offset_[1], map_offset_[2];
          transform.rotate(Eigen::AngleAxisf(map_offset_[5], Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(map_offset_[4], Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(map_offset_[3], Eigen::Vector3f::UnitX()));

          pcl::transformPointCloud(*target_cloud, *target_cloud, transform);
        }
      }

      // 3. 初始化 GICP Filter
      gicp_filter_ = std::make_unique<GicpFilter>(target_cloud, gicp_options_);

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
      map_to_camera_init_ = Eigen::Matrix4f::Identity();  // 初始时刻认为 camera_init 与 map 重合
      gicp_initialized_ = true;
      last_icp_time_ = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), "ICP initialized with first odometry pose.");
    }
    // 使用最新 map_to_camera_init_ 与当前里程计计算 base_link 在 map 下的位姿
    if(publish_pose_on_odom_ && last_odom_)
    {
      Eigen::Isometry3d odom_iso;
      tf2::fromMsg(last_odom_->pose.pose, odom_iso);
      Eigen::Matrix4f current_odom = odom_iso.matrix().cast<float>();

      Eigen::Matrix4f fused_pose;
      {
        std::lock_guard<std::mutex> lock(map_to_camera_init_mutex_);
        fused_pose = map_to_camera_init_ * current_odom;
      }

      publishPose(fused_pose, last_odom_->header.stamp);
    }
  }

  void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 将新点云加入队列
    cloud_queue_.push_back(msg);

    while(cloud_queue_.size() > static_cast<size_t>(accumulate_frames_ * 2))
    {
      cloud_queue_.pop_front();
    }
  }

  void GicpRosInterface::fsmTimerCallback()
  {
    runFSM();
  }

  void GicpRosInterface::runFSM()
  {
    // 检查数据有效性
    auto odom = last_odom_;  // 本地副本，避免并发修改
    if(!odom)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry to be available...");
      return;
    }

    // 检查是否有足够的点云帧进行累积
    if(cloud_queue_.size() < static_cast<size_t>(accumulate_frames_))
    {
      return;
    }

    // 累积点云
    PointCloud::Ptr source_cloud(new PointCloud());
    std::string target_frame = odom->header.frame_id;  // 通常是 camera_init 或 odom
    rclcpp::Time cloud_stamp;

    // 取队列中最新的 accumulate_frames_ 帧
    int start_idx = cloud_queue_.size() - accumulate_frames_;
    for(int i = start_idx; i < (int)cloud_queue_.size(); ++i)
    {
      auto& msg = cloud_queue_[i];
      PointCloud::Ptr temp_cloud(new PointCloud());
      pcl::fromROSMsg(*msg, *temp_cloud);

      // 如果点云不在目标坐标系下，则进行变换 (运动补偿)
      if(msg->header.frame_id != target_frame)
      {
        try
        {
          // 查询点云时刻的 TF (msg_frame -> target_frame)
          geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
              target_frame, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

          Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf_stamped);
          pcl::transformPointCloud(*temp_cloud, *temp_cloud, tf_eigen);
        }
        catch(tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
          continue;  // 跳过这一帧
        }
      }

      *source_cloud += *temp_cloud;

      // 使用最后一帧的时间戳作为本次配准的时间戳
      if(i == (int)cloud_queue_.size() - 1)
      {
        cloud_stamp = msg->header.stamp;
      }
    }

    if(source_cloud->empty() || source_cloud->size() < 100)
    {
      RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty or too small (%zu points).", source_cloud->size());
      return;
    }

    // Debug: 发布累积后的点云
    {
      sensor_msgs::msg::PointCloud2 debug_msg;
      pcl::toROSMsg(*source_cloud, debug_msg);
      debug_msg.header.frame_id = target_frame;
      debug_msg.header.stamp = cloud_stamp;
      source_pub_->publish(debug_msg);
    }

    // 准备通用数据
    Eigen::Isometry3d odom_iso;
    tf2::fromMsg(odom->pose.pose, odom_iso);
    Eigen::Matrix4f current_odom = odom_iso.matrix().cast<float>();

    // 状态机逻辑
    switch(state_)
    {
    case State::UNINITIALIZED:
    {
      state_ = State::INITIALIZING;
      break;
    }

    case State::INITIALIZING:
    {
      RCLCPP_INFO(this->get_logger(), "Starting initial alignment (SAC-IA)...");

      auto result = gicp_filter_->initialAlign(source_cloud);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "Initial alignment successful! Fitness score: %f", result.fitness_score);

        // GICP/SAC-IA 计算的是 Source(camera_init) -> Target(map) 的变换
        {
          std::lock_guard<std::mutex> lock(map_to_camera_init_mutex_);
          map_to_camera_init_ = result.final_transformation.inverse();

          // base_link 在 map 下 = (map->camera_init) * (camera_init->base_link)
          last_icp_pose_ = map_to_camera_init_ * current_odom;
        }

        gicp_initialized_ = true;
        last_icp_time_ = this->now();
        state_ = State::LOCALIZED;
        if(publish_pose_on_odom_ && odom)
        {
          publishPose(last_icp_pose_, odom->header.stamp);
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Initial alignment failed. Retrying on next scan...");
        state_ = State::UNINITIALIZED;
      }
      break;
    }
    case State::LOCALIZED:
    {
      if(!gicp_initialized_)
      {
        state_ = State::UNINITIALIZED;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Processing new lidar scan for incremental alignment...");
      last_icp_time_ = this->now();

      // GICP 需要 Source -> Target 的初值，即 camera_init -> map
      Eigen::Matrix4f initial_guess;
      {
        std::lock_guard<std::mutex> lock(map_to_camera_init_mutex_);
        initial_guess = map_to_camera_init_.inverse();
      }

      auto result = gicp_filter_->align(source_cloud, initial_guess);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "GICP converged with fitness score: %f", result.fitness_score);

        // 更新 map -> camera_init
        {
          std::lock_guard<std::mutex> lock(map_to_camera_init_mutex_);
          map_to_camera_init_ = result.final_transformation.inverse();
          // 计算 base_link 在 map 下的位姿
          last_icp_pose_ = map_to_camera_init_ * current_odom;
        }

        checkDriftAndCorrect(last_icp_pose_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "GICP failed to converge.");
      }
      break;
    }

    default:
      break;
    }
  }

  void GicpRosInterface::checkDriftAndCorrect(const Eigen::Matrix4f& icp_pose)
  {
    Eigen::Vector3f icp_pos = icp_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf icp_quat(icp_pose.block<3, 3>(0, 0));

    // 使用当前的 map_to_camera_init_ 预测位姿 (base_link 在 map 下)
    Eigen::Isometry3d odom_iso;
    tf2::fromMsg(last_odom_->pose.pose, odom_iso);
    Eigen::Matrix4f odom_mat = odom_iso.matrix().cast<float>();
    Eigen::Matrix4f predicted_pose = map_to_camera_init_ * odom_mat;

    Eigen::Vector3f odom_pos = predicted_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf odom_quat(predicted_pose.block<3, 3>(0, 0));

    float pos_diff = (icp_pos - odom_pos).norm();
    float angle_diff = icp_quat.angularDistance(odom_quat);

    RCLCPP_INFO(this->get_logger(), "ICP/Prediction diff: pos=%.3fm, angle=%.3frad", pos_diff, angle_diff);

    if(pos_diff > drift_threshold_m_ || angle_diff > drift_threshold_rad_)
    {
      RCLCPP_WARN(this->get_logger(), "Significant drift detected! Resetting odometry integration base.");
      // TODO 加上LIO漂移替代方案，此处只是发出警告，没有实际处理逻辑
    }
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

    pose_pub_->publish(pose_msg);

    // Publish Map -> Camera Init TF
    if(last_odom_)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = map_frame_;
      t.child_frame_id = last_odom_->header.frame_id;

      Eigen::Matrix4f current_map_to_init;
      {
        std::lock_guard<std::mutex> lock(map_to_camera_init_mutex_);
        current_map_to_init = map_to_camera_init_;
      }

      Eigen::Vector3f tf_pos = current_map_to_init.block<3, 1>(0, 3);
      Eigen::Quaternionf tf_quat(current_map_to_init.block<3, 3>(0, 0));

      t.transform.translation.x = tf_pos.x();
      t.transform.translation.y = tf_pos.y();
      t.transform.translation.z = tf_pos.z();
      t.transform.rotation.x = tf_quat.x();
      t.transform.rotation.y = tf_quat.y();
      t.transform.rotation.z = tf_quat.z();
      t.transform.rotation.w = tf_quat.w();

      tf_broadcaster_->sendTransform(t);

      // 额外发布 map -> base_link 的 TF
      geometry_msgs::msg::TransformStamped tb;
      tb.header.stamp = stamp;
      tb.header.frame_id = map_frame_;
      tb.child_frame_id = base_frame_;

      Eigen::Vector3f b_pos = pose.block<3, 1>(0, 3);
      Eigen::Quaternionf b_quat(pose.block<3, 3>(0, 0));
      tb.transform.translation.x = b_pos.x();
      tb.transform.translation.y = b_pos.y();
      tb.transform.translation.z = b_pos.z();
      tb.transform.rotation.x = b_quat.x();
      tb.transform.rotation.y = b_quat.y();
      tb.transform.rotation.z = b_quat.z();
      tb.transform.rotation.w = b_quat.w();
      tf_broadcaster_->sendTransform(tb);
    }
  }

}  // namespace icp_relocalization
