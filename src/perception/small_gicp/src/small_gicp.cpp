#include "small_gicp.hpp"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace small_gicp_relocalization
{

  SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions& options)
    : Node("small_gicp_relocalization", options)
    , result_t_(Eigen::Isometry3d::Identity())
    , init_guess_(Eigen::Isometry3d::Identity())
    , current_frame_count_(0)
  {
    this->declare_parameter("num_threads", 8);
    this->declare_parameter("num_neighbors", 20);
    this->declare_parameter("global_leaf_size", 0.25);
    this->declare_parameter("registered_leaf_size", 0.25);
    this->declare_parameter("max_dist_sq", 1.0);
    this->declare_parameter("prior_pcd_file", "");
    this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
    this->declare_parameter("frames_to_accumulate", 10);  // 默认累积10帧
    this->declare_parameter("pass_count", 5);             // 默认累积10帧

    this->get_parameter("num_threads", num_threads_);
    this->get_parameter("num_neighbors", num_neighbors_);
    this->get_parameter("global_leaf_size", global_leaf_size_);
    this->get_parameter("registered_leaf_size", registered_leaf_size_);
    this->get_parameter("max_dist_sq", max_dist_sq_);
    this->get_parameter("prior_pcd_file", prior_pcd_file_);
    this->get_parameter("init_pose", init_pose_);
    this->get_parameter("frames_to_accumulate", frames_to_accumulate_);
    this->get_parameter("pass_count", pass_count_);

    RCLCPP_INFO(this->get_logger(), "=== Configuration ===");
    RCLCPP_INFO(this->get_logger(), "Frames to accumulate: %d", frames_to_accumulate_);
    RCLCPP_INFO(this->get_logger(), "PCD file: %s", prior_pcd_file_.c_str());

    if(!init_pose_.empty() && init_pose_.size() >= 6)
    {
      init_guess_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
      init_guess_.linear() = (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()))
                                 .toRotationMatrix();

      RCLCPP_INFO(this->get_logger(), "Initial pose guess set:");
      RCLCPP_INFO(this->get_logger(), "  Translation: [%.3f, %.3f, %.3f]", init_pose_[0], init_pose_[1], init_pose_[2]);
      RCLCPP_INFO(
          this->get_logger(), "  Rotation (RPY): [%.3f, %.3f, %.3f] rad", init_pose_[3], init_pose_[4], init_pose_[5]);
    }

    accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

    // 初始化静态TF广播器
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    loadGlobalMap(prior_pcd_file_);

    RCLCPP_INFO(this->get_logger(), "Downsampling global map...");
    target_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *global_map_, global_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "Target points after downsampling: %zu", target_->size());

    RCLCPP_INFO(this->get_logger(), "Estimating covariances...");
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

    RCLCPP_INFO(this->get_logger(), "Building KdTree...");
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/stdpc", 10, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { pointCloudCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Small GICP Relocalization Node initialized successfully!");
    RCLCPP_INFO(
        this->get_logger(), "Waiting for %d frames of point cloud data on /livox/stdpc...", frames_to_accumulate_);
  }

  void SmallGicpRelocalizationNode::loadGlobalMap(const std::string& file_name)
  {
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Loaded global map with %zu points from: %s",
                global_map_->points.size(),
                file_name.c_str());
  }
  // 累计点云（按帧）
  void SmallGicpRelocalizationNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);
    *accumulated_cloud_ += *scan;

    current_frame_count_++;

    RCLCPP_INFO(this->get_logger(),
                "Accumulated frame %d/%d (total points: %zu)",
                static_cast<int>(current_frame_count_),
                static_cast<int>(frames_to_accumulate_),
                static_cast<size_t>(accumulated_cloud_->size()));

    // 达到设定的帧数后触发配准
    if(current_frame_count_ >= frames_to_accumulate_)
    {
      RCLCPP_INFO(this->get_logger(), "Reached %d frames", frames_to_accumulate_);
      performRegistration();
      // 配准后重置计数器，继续下一轮累积
      current_frame_count_ = 0;
    }
  }
  // 执行配准
  void SmallGicpRelocalizationNode::performRegistration()
  {
    if(accumulated_cloud_->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty!");
      return;
    }

    if(global_map_->empty() || !target_ || target_->empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Target map is empty! Cannot perform registration.");
      RCLCPP_ERROR(this->get_logger(), "Please check if PCD file is loaded correctly.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "=== Starting GICP Registration ===");
    RCLCPP_INFO(
        this->get_logger(), "Accumulated %d frames with %zu points", static_cast<int>(current_frame_count_), static_cast<size_t>(accumulated_cloud_->size()));
    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *accumulated_cloud_, registered_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "Source points after downsampling: %zu", source_->size());

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        source_, small_gicp::KdTreeBuilderOMP(num_threads_));

    if(!source_ || !source_tree_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create source tree");
      return;
    }

    register_->reduction.num_threads = num_threads_;
    register_->rejector.max_dist_sq = max_dist_sq_;
    register_->optimizer.max_iterations = 20;

    RCLCPP_INFO(this->get_logger(), "Running GICP alignment...");
    small_gicp::RegistrationResult result = register_->align(*target_, *source_, *target_tree_, init_guess_);

    if(result.converged)
    {
      result_t_ = result.T_target_source;

      // RCLCPP_INFO(this->get_logger(), "=== GICP Registration CONVERGED ===");
      RCLCPP_INFO(this->get_logger(), "Iterations: %d Fitness score: %.6f", (int)result.iterations, result.error);

      printRegistrationResult();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "=== GICP Registration FAILED ===");
      // RCLCPP_WARN(this->get_logger(), "Did not converge after %d iterations", (int)result.iterations);
      // RCLCPP_WARN(this->get_logger(), "Final error: %.6f", result.error);
    }

    // 清空累积点云，准备下一轮
    accumulated_cloud_->clear();
    // RCLCPP_INFO(this->get_logger(), "Accumulated cloud cleared. Ready for next batch.\n");
  }

  void SmallGicpRelocalizationNode::printRegistrationResult()
  {
    Eigen::Matrix3d rotation = result_t_.rotation();
    Eigen::Vector3d translation = result_t_.translation();
    Eigen::Vector3d euler = rotation.eulerAngles(0, 1, 2);
    Eigen::Quaterniond quaternion(rotation);

    
    static int pass = 1;
    if(pass++ >= pass_count_)
    {
      // 发布静态变换
      // camera_init 到 map
      // 发布静态TF: /camera_init -> /map
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = "camera_init";
      transform_stamped.child_frame_id = "map";
  
      // 设置平移
      transform_stamped.transform.translation.x = translation.x();
      transform_stamped.transform.translation.y = translation.y();
      transform_stamped.transform.translation.z = translation.z();
  
      // 设置旋转（使用四元数）
      transform_stamped.transform.rotation.x = quaternion.x();
      transform_stamped.transform.rotation.y = quaternion.y();
      transform_stamped.transform.rotation.z = quaternion.z();
      transform_stamped.transform.rotation.w = quaternion.w();
  
      // 发布静态变换
      tf_static_broadcaster_->sendTransform(transform_stamped);
    }

    // RCLCPP_INFO(this->get_logger(), "\n");
    // RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════════════════════════╗");
    // RCLCPP_INFO(this->get_logger(), "║          GICP Registration Result (RT Matrix)         ║");
    // RCLCPP_INFO(this->get_logger(), "╠═══════════════════════════════════════════════════════╣");
    // RCLCPP_INFO(this->get_logger(), "║ Translation (x, y, z):                                ║");

    RCLCPP_INFO(this->get_logger(),
                "║   [%8.4f, %8.4f, %8.4f] meters          ║",
                translation.x(),
                translation.y(),
                translation.z());
    // RCLCPP_INFO(this->get_logger(), "╠═══════════════════════════════════════════════════════╣");
    // RCLCPP_INFO(this->get_logger(), "║ Rotation (Roll, Pitch, Yaw):                          ║");
    // RCLCPP_INFO(this->get_logger(), "║   [%8.4f, %8.4f, %8.4f] radians         ║", euler.x(), euler.y(), euler.z());
    RCLCPP_INFO(this->get_logger(),
                "║   [%8.4f, %8.4f, %8.4f] degrees         ║",
                euler.x() * 180.0 / M_PI,
                euler.y() * 180.0 / M_PI,
                euler.z() * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Static TF published: /map -> /camera_init");
    // RCLCPP_INFO(this->get_logger(), "╠═══════════════════════════════════════════════════════╣");
    // RCLCPP_INFO(this->get_logger(), "║ Quaternion (x, y, z, w):                              ║");
    // RCLCPP_INFO(this->get_logger(),
    //             "║   [%7.4f, %7.4f, %7.4f, %7.4f]            ║",
    //             quaternion.x(),
    //             quaternion.y(),
    //             quaternion.z(),
    //             quaternion.w());
    // RCLCPP_INFO(this->get_logger(), "╠═══════════════════════════════════════════════════════╣");
    // RCLCPP_INFO(this->get_logger(), "║ 4x4 Transformation Matrix:                            ║");
    // RCLCPP_INFO(this->get_logger(), "║                                                       ║");

    // for(int i = 0; i < 4; ++i)
    // {
    //   RCLCPP_INFO(this->get_logger(),
    //               "║  [%8.5f, %8.5f, %8.5f | %8.5f]  ║",
    //               result_t_.matrix()(i, 0),
    //               result_t_.matrix()(i, 1),
    //               result_t_.matrix()(i, 2),
    //               result_t_.matrix()(i, 3));
    // }

    // RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════════════════════════╝");
    RCLCPP_INFO(this->get_logger(), "\n");
  }

}  // namespace small_gicp_relocalization