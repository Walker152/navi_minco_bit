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
    std::string target_pcd_file = this->declare_parameter<std::string>("target_pcd_file", "map.pcd");

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
                    NV(target_pcd_file));
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
      Eigen::Matrix4f fused_pose = map_to_camera_init_ * current_odom;
      publishPose(fused_pose, last_odom_->header.stamp);
    }
  }

  void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if(!latest_cloud_)
    {
      latest_cloud_ = std::make_shared<PointCloud>();
    }
    pcl::fromROSMsg(*msg, *latest_cloud_);
    latest_cloud_frame_ = msg->header.frame_id;
    latest_cloud_stamp_ = msg->header.stamp;
    has_new_cloud_ = true;
  }

  void GicpRosInterface::fsmTimerCallback()
  {
    runFSM();
  }

  void GicpRosInterface::runFSM()
  {
    // 检查数据有效性
    if(!last_odom_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for odometry to be available...");
      return;
    }

    // 获取最新点云
    PointCloud::Ptr source_cloud(new PointCloud());
    std::string cloud_frame;
    rclcpp::Time cloud_stamp;
    bool has_data = false;

    if(has_new_cloud_)
    {
      *source_cloud = *latest_cloud_;
      cloud_frame = latest_cloud_frame_;
      cloud_stamp = latest_cloud_stamp_;
      has_new_cloud_ = false;
      has_data = true;
    }

    // 准备通用数据
    Eigen::Isometry3d odom_iso;
    tf2::fromMsg(last_odom_->pose.pose, odom_iso);
    Eigen::Matrix4f current_odom = odom_iso.matrix().cast<float>();

    // 状态机逻辑
    switch(state_)
    {
    case State::UNINITIALIZED:
    {
      if(!has_data)
      {
        RCLCPP_INFO(this->get_logger(), "No LIDAR data available for initialization.");
        return;
      }
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

        // 点云在 camera_init 系，SAC-IA 结果即为 map -> camera_init
        map_to_camera_init_ = result.final_transformation;
        // base_link 在 map 下 = (map->camera_init) * (camera_init->base_link)
        last_icp_pose_ = map_to_camera_init_ * current_odom;

        gicp_initialized_ = true;
        last_icp_time_ = this->now();
        state_ = State::LOCALIZED;
        if(publish_pose_on_odom_ && last_odom_)
        {
          publishPose(last_icp_pose_, last_odom_->header.stamp);
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

      if(!has_data)
        return;

      RCLCPP_INFO(this->get_logger(), "Processing new lidar scan for incremental alignment...");
      last_icp_time_ = this->now();

      // 点云在 camera_init 系，初值即为 map -> camera_init
      Eigen::Matrix4f initial_guess = map_to_camera_init_;

      auto result = gicp_filter_->align(source_cloud, initial_guess);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "GICP converged with fitness score: %f", result.fitness_score);

        // 更新 map -> camera_init
        map_to_camera_init_ = result.final_transformation;
        // 计算 base_link 在 map 下的位姿
        last_icp_pose_ = map_to_camera_init_ * current_odom;

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

      Eigen::Vector3f tf_pos = map_to_camera_init_.block<3, 1>(0, 3);
      Eigen::Quaternionf tf_quat(map_to_camera_init_.block<3, 3>(0, 0));

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
