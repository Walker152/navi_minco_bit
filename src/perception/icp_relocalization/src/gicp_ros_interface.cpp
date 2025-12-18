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
    std::string mode_str = this->declare_parameter<std::string>("mode", "sac_ia");
    if(mode_str == "sac_ia")
      mode_ = Mode::SAC_IA;
    else if(mode_str == "initial_guess")
      mode_ = Mode::INITIAL_GUESS;
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid mode: %s. Defaulting to sac_ia.", mode_str.c_str());
      mode_ = Mode::SAC_IA;
    }

    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    alignment_frequency_ = this->declare_parameter<double>("alignment_frequency", 1.0);
    accumulate_frames_ = this->declare_parameter<int>("accumulate_frames", 5);
    fitness_score_threshold_ = this->declare_parameter<double>("fitness_score_threshold", 0.5);
    converged_count_threshold_ = this->declare_parameter<int>("converged_count_threshold", 5);
    std::string target_pcd_file = this->declare_parameter<std::string>("target_pcd_file", "map.pcd");

    // Map Offset Parameters
    initial_pose_ = this->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // GICP Parameters
    gicp_options_.feature_k_search = this->declare_parameter<int>("feature_k_search", 20);
    gicp_options_.target_voxel_leaf_size = this->declare_parameter<double>("gicp.target_voxel_leaf_size", 0.1);
    gicp_options_.source_voxel_leaf_size = this->declare_parameter<double>("gicp.source_voxel_leaf_size", 0.1);
    gicp_options_.max_correspondence_distance = this->declare_parameter<double>("gicp.max_correspondence_distance", 1.5);
    gicp_options_.max_iterations = this->declare_parameter<int>("gicp.max_iterations", 100);
    gicp_options_.transformation_epsilon = this->declare_parameter<double>("gicp.transformation_epsilon", 1e-4);
    gicp_options_.euclidean_fitness_epsilon = this->declare_parameter<double>("gicp.euclidean_fitness_epsilon", 1e-4);

    // SAC-IA Parameters
    gicp_options_.sac_ia_min_sample_distance = this->declare_parameter<double>("sac_ia.min_sample_distance", 0.5);
    gicp_options_.sac_ia_correspondence_randomness =
        this->declare_parameter<int>("sac_ia.correspondence_randomness", 6);
    gicp_options_.sac_ia_num_samples = this->declare_parameter<int>("sac_ia.num_samples", 3);
    gicp_options_.sac_ia_max_correspondence_distance =
        this->declare_parameter<double>("sac_ia.max_correspondence_distance", 1.0);

    std::cout << BOLDCYAN << " ========== GICP Relocalization ==========" << RESET << std::endl;
    LOG_DEBUG_BLOCK(std::string(CYAN) + "[RELOCALIZATION] ",
                    NV(mode_str),
                    NV(map_frame_),
                    NV(alignment_frequency_),
                    NV(accumulate_frames_),
                    NV(converged_count_threshold_),
                    NV(target_pcd_file));

    if(initial_pose_.size() >= 6)
    {
      std::cout << BOLDCYAN << "  ---------- Initial Pose ----------" << RESET << std::endl;
      LOG_DEBUG_BLOCK(std::string(CYAN) + "[INIT POSE] ",
                      NV(initial_pose_[0]),
                      NV(initial_pose_[1]),
                      NV(initial_pose_[2]),
                      NV(initial_pose_[3]),
                      NV(initial_pose_[4]),
                      NV(initial_pose_[5]));
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
    
    if(mode_ == Mode::SAC_IA)
    {
      RCLCPP_INFO(this->get_logger(), "Mode: SAC-IA. Waiting for lidar scan to initialize.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Mode: Initial Guess. Waiting for initial pose.");
      // 如果提供了初始位姿参数，则直接使用
      if(initial_pose_.size() == 6)
      {
        RCLCPP_INFO(this->get_logger(), "Using initial pose from parameters.");
      }
    }

    setupGicp(target_pcd_file);

    // ROS接口初始化
    callback_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lidar_sub_options;
    lidar_sub_options.callback_group = callback_group_lidar_;

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/stdpc",
        rclcpp::SensorDataQoS(),
        std::bind(&GicpRosInterface::lidarCallback, this, std::placeholders::_1),
        lidar_sub_options);

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_map", rclcpp::QoS(1).transient_local());
    source_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_source", 10);  // Debug: accumulated cloud
    aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_aligned", 10);
    
    // Use StaticTransformBroadcaster for static map->camera_init transform
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    accumulated_cloud_ = std::make_shared<PointCloud>();

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

      // 2. 初始化 GICP Filter
      gicp_filter_ = std::make_unique<GicpFilter>(target_cloud, gicp_options_);

      RCLCPP_INFO(this->get_logger(), "GICP filter initialized with map: %s", target_pcd_file.c_str());
    }
    catch(const std::runtime_error& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize GICP filter: %s", e.what());
      rclcpp::shutdown();
    }
  }


  void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Accumulate points directly
    PointCloud::Ptr temp_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *temp_cloud);
    
    // Assuming input cloud is already in the correct frame (camera_init/odom)
    // If not, we might need to transform it, but user said it's ensured.
    *accumulated_cloud_ += *temp_cloud;
    current_accumulated_frames_++;
    last_cloud_stamp_ = msg->header.stamp;
    cloud_frame_id_ = msg->header.frame_id;

    // Reset if too large (safety)
    if(accumulated_cloud_->size() > 100000) 
    {
        // Keep last N points or just clear? 
        // For relocalization, we usually want a specific window.
        // If we just keep adding, it grows indefinitely until FSM runs.
        // FSM runs at 1Hz (default), lidar at 10Hz. 
        // accumulate_frames_ is 5.
    }
  }

  void GicpRosInterface::fsmTimerCallback()
  {
    runFSM();
  }

  void GicpRosInterface::runFSM()
  {
    // 检查是否有足够的点云帧进行累积
    if(current_accumulated_frames_ < accumulate_frames_)
    {
      return;
    }

    PointCloud::Ptr source_cloud(new PointCloud());
    *source_cloud = *accumulated_cloud_;
    
    // Clear accumulation for next batch
    accumulated_cloud_->clear();
    current_accumulated_frames_ = 0;

    if(source_cloud->empty() || source_cloud->size() < 100)
    {
      RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty or too small (%zu points).", source_cloud->size());
      return;
    }

    // Debug: 发布累积后的点云
    publishVisualization(source_cloud, last_cloud_stamp_);

    // 状态机逻辑
    switch(state_)
    {
    case State::UNINITIALIZED:
    {
      if(mode_ == Mode::SAC_IA)
      {
        state_ = State::INITIALIZING;
      }
      else if(mode_ == Mode::INITIAL_GUESS)
      {
            RCLCPP_INFO(this->get_logger(), "Initializing GICP from parameter pose...");

            // Initial pose from param (map -> camera_init)
            Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();
            initial_pose.block<3, 1>(0, 3) = Eigen::Vector3f(initial_pose_[0], initial_pose_[1], initial_pose_[2]);
            
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

    case State::INITIALIZING:
    {
      RCLCPP_INFO(this->get_logger(), "Starting initial alignment (SAC-IA)...");

      auto result = gicp_filter_->initialAlign(source_cloud);

      if(result.converged)
      {
        RCLCPP_INFO(this->get_logger(), "Initial alignment successful! Fitness score: %f", result.fitness_score);

        // GICP/SAC-IA 计算的是 Source(camera_init) -> Target(map) 的变换
        map_to_camera_init_ = result.final_transformation;

        gicp_initialized_ = true;
        last_icp_time_ = this->now();
        converged_count_ = 0;
        state_ = State::CONVERGING;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Initial alignment failed. Retrying on next scan...");
        state_ = State::UNINITIALIZED;
      }
      break;
    }
    case State::CONVERGING:
    {
      if(!gicp_initialized_)
      {
        state_ = State::UNINITIALIZED;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Verifying convergence (Count: %d/%d)...", converged_count_, converged_count_threshold_);
      
      Eigen::Matrix4f initial_guess = map_to_camera_init_;

      auto result = gicp_filter_->align(source_cloud, initial_guess);

      if(result.converged && result.fitness_score < fitness_score_threshold_)
      {
        RCLCPP_INFO(this->get_logger(), "GICP converged with score: %f", result.fitness_score);
        
        map_to_camera_init_ = result.final_transformation;
        
        converged_count_++;
        if(converged_count_ >= converged_count_threshold_)
        {
          RCLCPP_INFO(this->get_logger(), "Relocalization converged successfully! Switching to LOCALIZED state.");
          state_ = State::LOCALIZED;
          
          // Publish Static TF once converged
          publishStaticTf(this->now());
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "GICP failed to converge or score too high (%f). Resetting count.", result.fitness_score);
        converged_count_ = 0;
        // If we were in SAC_IA mode, maybe we should restart?
        // If we were in INITIAL_GUESS mode, maybe we should wait for new guess?
        // For now, let's go back to UNINITIALIZED to be safe.
        state_ = State::UNINITIALIZED;
        gicp_initialized_ = false;
      }
      break;
    }
    case State::LOCALIZED:
    {
      // In Static TF mode, we don't need to run GICP continuously.
      // We can just monitor or do nothing.
      // For now, let's just return to save CPU.
      // If re-triggering is needed, external logic should reset state to UNINITIALIZED.
      break;
    }

    default:
      break;
    }
  }

  void GicpRosInterface::publishStaticTf(const rclcpp::Time& stamp)
  {
    if(cloud_frame_id_.empty()) return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = cloud_frame_id_; // camera_init or odom

    Eigen::Matrix4f current_map_to_init = map_to_camera_init_;

    Eigen::Vector3f tf_pos = current_map_to_init.block<3, 1>(0, 3);
    Eigen::Quaternionf tf_quat(current_map_to_init.block<3, 3>(0, 0));

    t.transform.translation.x = tf_pos.x();
    t.transform.translation.y = tf_pos.y();
    t.transform.translation.z = tf_pos.z();
    t.transform.rotation.x = tf_quat.x();
    t.transform.rotation.y = tf_quat.y();
    t.transform.rotation.z = tf_quat.z();
    t.transform.rotation.w = tf_quat.w();

    static_tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Published static TF: %s -> %s", t.header.frame_id.c_str(), t.child_frame_id.c_str());
  }  
  
  void GicpRosInterface::publishVisualization(const PointCloud::Ptr& cloud, const rclcpp::Time& stamp)
  {
    // 1. 发布原始累积点云 (Source Cloud)
    if(source_pub_->get_subscription_count() > 0)
    {
      sensor_msgs::msg::PointCloud2 source_msg;
      pcl::toROSMsg(*cloud, source_msg);
      source_msg.header.frame_id = "camera_init"; // Fallback
      source_msg.header.stamp = stamp;
      source_pub_->publish(source_msg);
    }

    // 2. 发布配准后的点云 (Aligned Cloud)
    // 将点云变换到 map 坐标系下发布，用于验证配准效果
    if(aligned_cloud_pub_->get_subscription_count() > 0 && gicp_initialized_)
    {
      PointCloud::Ptr aligned_cloud(new PointCloud());
      Eigen::Matrix4f transform = map_to_camera_init_;
      pcl::transformPointCloud(*cloud, *aligned_cloud, transform);

      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned_cloud, aligned_msg);
      aligned_msg.header.frame_id = map_frame_;
      aligned_msg.header.stamp = stamp;
      aligned_cloud_pub_->publish(aligned_msg);
    }
  }

}  // namespace icp_relocalization
