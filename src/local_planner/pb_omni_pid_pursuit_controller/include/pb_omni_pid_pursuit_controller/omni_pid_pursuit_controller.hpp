#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"                      // Nav2控制器基类接口
#include "pb_omni_pid_pursuit_controller/pid.hpp"       // PID控制器实现
#include "visualization_msgs/msg/marker_array.hpp"      // 可视化消息类型

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class OmniPidPursuitController
 * @brief 全向轮机器人的PID Pure Pursuit路径跟踪控制器
 * 
 * @details 该控制器继承自nav2_core::Controller，专为全向轮机器人设计：
 *          - 采用Pure Pursuit算法计算目标点（Carrot Point）
 *          - 使用两个独立的PID控制器分别控制线性移动和角度转动
 *          - 支持速度缩放、曲率限制、碰撞检测等高级功能
 *          - 提供丰富的可视化和调试信息
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief 默认构造函数
   */
  OmniPidPursuitController() = default;

  /**
   * @brief 析构函数 
   */
  ~OmniPidPursuitController() override = default;

  /**
   * @brief 配置控制器状态机和参数
   * @param parent 节点的弱引用指针
   * @param name 插件名称
   * @param tf TF变换缓冲区，用于坐标系转换
   * @param costmap_ros 环境代价地图对象，用于路径规划和碰撞检测
   * 
   * @details 该函数负责：
   *         - 初始化所有控制器参数（PID增益、速度限制等）
   *         - 创建发布者（用于可视化路径、目标点等）
   *         - 初始化两个PID控制器（移动和转向）
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理控制器资源
   * @details 释放发布者和其他资源
   */
  void cleanup() override;

  /**
   * @brief 激活控制器
   * @details 启用发布者并注册动态参数回调
   */
  void activate() override;

  /**
   * @brief 停用控制器  
   * @details 禁用发布者并注销参数回调
   */
  void deactivate() override;

  /**
   * @brief 计算当前最优速度控制命令（核心函数）
   *
   * @param pose 机器人当前位姿（在全局坐标系中）
   * @param velocity 机器人当前速度
   * @param goal_checker 目标检查器指针，用于判断是否到达目标
   * @return 计算得到的速度控制命令（TwistStamped）
   * 
   * @details 该函数实现了完整的控制流程：
   *         1. 将全局路径转换到机器人本体坐标系
   *         2. 计算自适应前瞻距离
   *         3. 获取前瞻目标点（Carrot Point）
   *         4. 使用PID控制器计算线性和角速度
   *         5. 应用曲率限制和接近速度缩放
   *         6. 进行碰撞检测
   *         7. 输出全向轮控制命令（vx, vy, wz）
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置全局路径
   * @param path 从规划器获取的全局路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 设置速度限制（当前实现为占位符）
   * @param speed_limit 速度限制值
   * @param percentage 是否为百分比形式
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Gets the maximum extent of the costmap
   * @return Maximum costmap extent in meters
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Creates a Carrot Point Marker message for visualization
   * @param carrot_pose Lookahead point pose
   * @return Unique pointer to the Carrot Point Marker message
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Gets the lookahead point on the transformed plan
   * @param lookahead_dist Lookahead distance
   * @param transformed_plan Transformed local plan
   * @return Lookahead point pose
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Calculates the intersection point of a circle and a line segment
   * @param p1 Start point of the line segment
   * @param p2 End point of the line segment
   * @param r Radius of the circle
   * @return Intersection point (geometry_msgs::msg::Point)
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief Callback function for dynamic parameter updates
   * @param parameters Vector of updated parameters
   * @return Result of parameter setting
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Calculates the lookahead distance based on current velocity
   * @param speed Current robot velocity
   * @return Lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  /**
   * @brief Calculates the approach velocity scaling factor based on remaining path distance
   * @param path Transformed local path
   * @return Velocity scaling factor
   */
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Applies velocity scaling based on approach distance to the goal
   * @param path Transformed local path
   * @param linear_vel Linear velocity command (in out)
   */
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

  /**
   * @brief Checks if collision is detected along the given path
   * @param path Local path to check for collisions
   * @return True if collision detected, false otherwise
   */
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  /**
   * @brief Applies curvature based speed limitation
   * @param path Transformed local path
   * @param lookahead_pose Lookahead point pose
   * @param linear_vel Linear velocity command (in out)
   */
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  /**
   * @brief Calculates curvature using three-point circle fitting
   * @param path Transformed local path
   * @param lookahead_pose Lookahead pose (current point)
   * @param forward_dist
   * @param backward_dist
   * @return Curvature value
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  /**
   * @brief Calculates the radius of curvature using three points
   * @param near_point Pose before the current point
   * @param current_point Current pose (lookahead pose)
   * @param far_point Pose after the current point
   * @return Radius of curvature
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  /**
   * @brief Visualizes near and far points used for curvature calculation
   * @param backward_pose Near point pose
   * @param forward_pose Far point pose
   */
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  /**
   * @brief Calculates cumulative distances along the path
   * @param path The path to calculate distances for
   * @return Vector of cumulative distances
   */
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Finds a pose on the path at a given distance
   * @param path The path to search on
   * @param cumulative_distances Vector of cumulative distances along the path
   * @param target_distance The target distance to find the pose at
   * @return Pose at the target distance, or empty pose if not found
   */
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  double last_velocity_scaling_factor_;

  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;

  // Controller parameters
  double translation_kp_, translation_ki_, translation_kd_;
  bool enable_rotation_;
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double min_max_sum_error_;
  double control_duration_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_;
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_rotate_to_heading_;
  double use_rotate_to_heading_treshold_;
  double v_linear_min_;
  double v_linear_max_;
  double v_angular_min_;
  double v_angular_max_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  tf2::Duration transform_tolerance_;

  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
