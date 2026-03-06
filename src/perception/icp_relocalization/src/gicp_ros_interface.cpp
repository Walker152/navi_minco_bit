#include "gicp_ros_interface.hpp"
#include "gicp_utils.hpp"
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

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
    visualization_en_ = this->declare_parameter<bool>("visualization_en", true);
    source_cloud_topic_ = this->declare_parameter<std::string>("source_cloud_topic", "/livox_stdpc");
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
                    NV(visualization_en_),
                    NV(source_cloud_topic_),
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
                    NV(gicp_options_.euclidean_fitness_epsilon),
                    NV(gicp_options_.height_filter_enabled),
                    NV(gicp_options_.height_filter_min_z),
                    NV(gicp_options_.height_filter_max_z),
                    NV(gicp_options_.source_crop_enabled),
                    NV(gicp_options_.source_crop_min_x),
                    NV(gicp_options_.source_crop_max_x),
                    NV(gicp_options_.source_crop_min_y),
                    NV(gicp_options_.source_crop_max_y),
                    NV(gicp_options_.source_crop_min_z),
                    NV(gicp_options_.source_crop_max_z));
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

    // ROS接口初始化
    callback_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Serialize service callback with lidar/timer callbacks to avoid races under MultiThreadedExecutor.
    callback_group_service_ = callback_group_lidar_;

    activateLidarSubscription();

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_map", rclcpp::QoS(1).transient_local());
    source_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_source", 10);  // Debug: accumulated cloud
    aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gicp_aligned", 10);

    initializeDebugPublishers();

    setupGicp(target_pcd_file);
    
    // Use StaticTransformBroadcaster for static map->camera_init transform
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    accumulated_cloud_ = std::make_shared<PointCloud>();

    // 发布一次地图
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*gicp_filter_->getTargetCloud(), map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = this->now();
    map_pub_->publish(map_msg);

    // 初始化FSM定时器
    if(alignment_frequency_ <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(), "alignment_frequency <= 0 (%.3f). Forcing to 1.0 Hz.", alignment_frequency_);
      alignment_frequency_ = 1.0;
    }
    fsm_period_ = std::chrono::duration<double>(1.0 / alignment_frequency_);
    startFsmTimer();
    
    // 初始化地图发布定时器
    map_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                         std::bind(&GicpRosInterface::mapTimerCallback, this));
    // 初始化重定位服务
    relocalize_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/gicp_recall",
        std::bind(&GicpRosInterface::relocalizeServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_service_);
  }

  void GicpRosInterface::activateLidarSubscription()
  {
    rclcpp::SubscriptionOptions lidar_sub_options;
    lidar_sub_options.callback_group = callback_group_lidar_;
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        source_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GicpRosInterface::lidarCallback, this, std::placeholders::_1),
        lidar_sub_options);
  }

  void GicpRosInterface::initializeDebugPublishers()
  {
    if(!visualization_en_)
    {
      return;
    }

    pub_source_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/source_raw", 10);
    pub_source_cropped_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/source_cropped", 10);
    pub_target_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/target_raw", 10);
    pub_target_cropped_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/target_cropped", 10);
  }

  void GicpRosInterface::startFsmTimer()
  {
    if(fsm_timer_)
    {
      fsm_timer_->cancel();
      fsm_timer_.reset();
    }

    fsm_timer_ = this->create_wall_timer(
        fsm_period_,
        std::bind(&GicpRosInterface::fsmTimerCallback, this),
        callback_group_lidar_);
  }

  void GicpRosInterface::relocalizeServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(this->get_logger(),
                "%sReceived request to re-trigger relocalization.%s",
                color_text::MAGENTA.c_str(),
                color_text::RESET.c_str());

    // 1. Reset State
    mode_ = Mode::SAC_IA;
    state_ = State::UNINITIALIZED;
    converged_count_ = 0;
    current_accumulated_frames_ = 0;
    if(accumulated_cloud_) accumulated_cloud_->clear();
    gicp_initialized_ = false;
    last_cloud_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    // 2. Reactivate Lidar Subscription if needed
    if(!lidar_sub_)
    {
      activateLidarSubscription();
      RCLCPP_INFO(this->get_logger(), "Lidar subscription reactivated.");
    }

    // 3. Restart FSM Timer
    startFsmTimer();

    response->success = true;
    response->message = "Relocalization process restarted.";
  }

  void GicpRosInterface::mapTimerCallback()
  {
      if (map_pub_->get_subscription_count() > 0 && gicp_filter_) {
          sensor_msgs::msg::PointCloud2 map_msg;
          pcl::toROSMsg(*gicp_filter_->getTargetCloud(), map_msg);
          map_msg.header.frame_id = map_frame_;
          map_msg.header.stamp = this->now();
          map_pub_->publish(map_msg);
      }
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

      if(visualization_en_ && pub_target_raw_)
      {
        sensor_msgs::msg::PointCloud2 target_raw_msg;
        pcl::toROSMsg(*target_cloud, target_raw_msg);
        target_raw_msg.header.frame_id = map_frame_;
        target_raw_msg.header.stamp = this->now();
        pub_target_raw_->publish(target_raw_msg);
      }

      // 2. 初始化 GICP Filter
      gicp_filter_ = std::make_unique<GicpFilter>(target_cloud, gicp_options_);
    }
    catch(const std::runtime_error& e)
    {
      rclcpp::shutdown();
    }
  }

  void GicpRosInterface::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Accumulate points directly
    PointCloud::Ptr temp_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *temp_cloud);

    if(visualization_en_ && pub_source_raw_)
    {
      sensor_msgs::msg::PointCloud2 source_raw_msg;
      pcl::toROSMsg(*temp_cloud, source_raw_msg);
      source_raw_msg.header.frame_id = msg->header.frame_id;
      source_raw_msg.header.stamp = msg->header.stamp;
      pub_source_raw_->publish(source_raw_msg);
    }
    
    *accumulated_cloud_ += *temp_cloud;
    current_accumulated_frames_++;
    last_cloud_stamp_ = msg->header.stamp;
    cloud_frame_id_ = msg->header.frame_id;

    // Reset if too large (safety)
    if(accumulated_cloud_->size() > 100000) 
    {
      RCLCPP_WARN(this->get_logger(), "Accumulated cloud too large (%zu points). Clearing buffer.", accumulated_cloud_->size());
      accumulated_cloud_->clear();
      current_accumulated_frames_ = 0;
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
    gicp_utils::publishVisualization(source_cloud,
                     last_cloud_stamp_,
                     visualization_en_,
                     gicp_initialized_,
                     map_frame_,
                     map_to_camera_init_,
                     source_pub_,
                     aligned_cloud_pub_);

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

      gicp_utils::publishSourceCroppedDebug(visualization_en_,
                    gicp_options_,
                    map_frame_,
                    source_cloud,
                    Eigen::Matrix4f::Identity(),
                    last_cloud_stamp_,
                    pub_source_cropped_);

      auto result = gicp_filter_->initialAlign(source_cloud);

      gicp_utils::publishTargetCroppedDebug(visualization_en_,
                    map_frame_,
                    last_cloud_stamp_,
                    gicp_filter_.get(),
                    pub_target_cropped_);

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

      gicp_utils::publishSourceCroppedDebug(visualization_en_,
                    gicp_options_,
                    map_frame_,
                    source_cloud,
                    initial_guess,
                    last_cloud_stamp_,
                    pub_source_cropped_);

      auto start_time = std::chrono::high_resolution_clock::now();
      auto result = gicp_filter_->align(source_cloud, initial_guess);
      auto end_time = std::chrono::high_resolution_clock::now();
      double time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

      gicp_utils::publishTargetCroppedDebug(visualization_en_,
                    map_frame_,
                    last_cloud_stamp_,
                    gicp_filter_.get(),
                    pub_target_cropped_);

      if(result.converged && result.fitness_score < fitness_score_threshold_)
      {
        gicp_utils::printEvaluation(initial_guess, result.final_transformation, result.fitness_score, time_ms);

        std::cout << MAGENTA << "[GICP] Converged! Score: " << result.fitness_score << RESET << std::endl;

        map_to_camera_init_ = result.final_transformation;
        
        converged_count_++;
        if(converged_count_ >= converged_count_threshold_)
        {
          std::cout << GREEN << "[GICP] Localization confirmed after " << converged_count_ << " consecutive convergences!" << RESET << std::endl;
          state_ = State::LOCALIZED;
          
          // Publish Static TF once converged
          gicp_utils::publishStaticTf(static_tf_broadcaster_,
                                      map_frame_,
                                      cloud_frame_id_,
                                      map_to_camera_init_,
                                      this->now());

          // Suspend operations to save resources
          RCLCPP_INFO(this->get_logger(), "Suspending GICP update timer and lidar subscription.");
          if (fsm_timer_) {
              fsm_timer_->cancel();
          }
          // Reset subscriber to stop receiving data completely
          if (lidar_sub_) {
             lidar_sub_.reset(); 
          }
        }
      }
      else
      {
        std::cout << YELLOW << "[GICP] Failed to converge or score too high (" << result.fitness_score << "). Resetting count." << RESET << std::endl;
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

  

}  // namespace icp_relocalization
