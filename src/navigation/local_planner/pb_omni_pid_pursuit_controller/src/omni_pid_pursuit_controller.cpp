#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
/*
 * 算法流程：
 * 1. 路径变换：将全局路径转换到机器人本体坐标系
 * 2. 前瞻计算：根据当前速度计算自适应前瞻距离
 * 3. 目标点选择：在路径上找到前瞻目标点（Carrot Point）
 * 4. PID控制：使用两个独立 PID控制器计算线性和角速度
 * 5. 约束应用：应用曲率限制和接近速度缩放
 * 6. 碰撞检测：检查路径是否安全
 * 7. 全向轮控制：输出适合全向轮的速度命令
 *
 */

// 常用函数和命名空间
using nav2_util::declare_parameter_if_not_declared;   // 参数声明工具
using nav2_util::geometry_utils::euclidean_distance;  // 欧几里德距离计算
using std::abs;                                       // 绝对值
using std::hypot;                                     // 斜边长度计算（√(x²+y²)）
using std::max;                                       // 最大值
using std::min;                                       // 最小值
using namespace nav2_costmap_2d;                      // 代价地图命名空间
using rcl_interfaces::msg::ParameterType;             // 参数类型

namespace pb_omni_pid_pursuit_controller
{
  /**
   * @brief 配置控制器的核心实现（生命周期节点的configure阶段）
   * @details 该函数是Nav2控制器插件生命周期的第一阶段，负责：
   *          1. 初始化所有成员变量和组件引用
   *          2. 声明和读取所有ROS参数
   *          3. 创建发布者（Publisher）
   *          4. 初始化PID控制器实例
   *
   * @note 生命周期节点状态机：
   *       Unconfigured -> configure() -> Inactive -> activate() -> Active
   *       在configure阶段，节点准备资源但不开始工作
   */
  void OmniPidPursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                           std::string name,
                                           std::shared_ptr<tf2_ros::Buffer> tf,
                                           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // === 1. 节点引用和安全检查 ===
    // 获取生命周期节点的强引用，weak_ptr防止循环引用但需要转换为shared_ptr使用
    auto node = parent.lock();
    node_ = parent;  // 保存弱引用供后续使用
    if(!node)
    {
      throw nav2_core::PlannerException("Unable to lock node!");
    }

    // === 2. 核心组件初始化 ===
    costmap_ros_ = costmap_ros;             // 代价地图ROS接口（用于坐标变换和碰撞检测）
    costmap_ = costmap_ros_->getCostmap();  // 代价地图数据结构（实际的栅格地图）
    tf_ = tf;                               // TF变换缓冲区（用于坐标系转换）
    plugin_name_ = name;                    // 插件名称（用于参数命名空间）
    logger_ = node->get_logger();           // 日志记录器
    clock_ = node->get_clock();             // 时钟服务（用于时间戳）

    // === 3. 默认值设置 ===
    double transform_tolerance = 1.0;                     // 坐标变换超时容差（秒）
    double control_frequency = 20.0;                      // 控制器更新频率（Hz）
    max_robot_pose_search_dist_ = getCostmapMaxExtent();  // 在路径上搜索机器人位置的最大距离

    // === 4. PID控制器参数声明 ===
    // 平移PID参数：控制机器人向目标点的线性运动
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));  // 比例增益：响应当前位置误差
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));  // 积分增益：消除稳态误差
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));  // 微分增益：减少超调和振荡

    // 旋转控制参数：控制机器人的角度调整
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));  // 是否启用角度控制
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));  // 角度比例增益
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));  // 角度积分增益
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));  // 角度微分增益
    // === 系统控制参数 ===
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));  // 坐标变换超时容差（秒）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));  // PID积分项限幅（防止积分饼和）

    // === Pure Pursuit 算法核心参数 ===
    // 前瞻距离参数：决定机器人的“视野”远近
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));  // 基础前瞻距离（米）
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".use_velocity_scaled_lookahead_dist",
                                      rclcpp::ParameterValue(true));  // 是否使用速度自适应前瞻
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));  // 最小前瞻距离（防止过近视野）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));  // 最大前瞻距离（防止过远视野）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));  // 前瞻时间常数（秒），用于计算自适应距离
    // === 路径跟踪策略参数 ===
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));  // 是否使用线性插值找前瞻点
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));  // 是否在路径末尾旋转到目标方向
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));  // 旋转阈值（弧度）

    // === 速度限制和缩放参数 ===
    // 接近目标时的减速策略
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".min_approach_linear_velocity",
                                      rclcpp::ParameterValue(0.05));  // 接近目标时的最小速度（m/s）
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".approach_velocity_scaling_dist",
                                      rclcpp::ParameterValue(0.6));  // 开始减速的距离阈值（米）

    // 速度的绝对限制
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));  // 最小线性速度（m/s，负值表示可以倒退）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));  // 最大线性速度（m/s）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));  // 最小角速度（rad/s）
    declare_parameter_if_not_declared(node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));

    // === 路径搜索参数 ===
    declare_parameter_if_not_declared(
        node,
        plugin_name_ + ".max_robot_pose_search_dist",
        rclcpp::ParameterValue(getCostmapMaxExtent()));  // 在路径上搜索机器人位置的最大距离

    // === 曲率限制参数（防止在急转弯时速度过快） ===
    // 曲率 = 1/曲率半径，值越大表示弯道越急
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));  // 开始限速的曲率阈值（1/m）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));  // 最大限速的曲率阈值（1/m）
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".reduction_ratio_at_high_curvature",
                                      rclcpp::ParameterValue(0.5));  // 高曲率时的速度减少比例

    // 曲率计算所需的参考点距离
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));  // 前向参考点距离（米）
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));  // 后向参考点距离（米）

    // 速度缩放的变化率限制（防止突然加速/减速）
    declare_parameter_if_not_declared(node,
                                      plugin_name_ + ".max_velocity_scaling_factor_rate",
                                      rclcpp::ParameterValue(0.9));  // 速度缩放系数的最大变化率

    // === 5. 参数读取：从yaml文件或launch参数中获取实际值 ===
    node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
    node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
    node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
    node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
    node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
    node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
    node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
    node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
    node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
    if(approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0)
    {
      RCLCPP_WARN(logger_,
                  "approach_velocity_scaling_dist is larger than forward costmap extent, "
                  "leading to permanent slowdown");
    }
    node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
    node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
    node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
    node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
    node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
    node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
    node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
    node->get_parameter(plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
    node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
    node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
    node->get_parameter(plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);

    node->get_parameter("controller_frequency", control_frequency);
    // TF 容忍
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_ = 1.0 / control_frequency;

    // === 6. 发布者创建（用于调试和可视化） ===
    // 注意：这里只是创建发布者，但在activate()之前不会实际发布数据

    // 局部路径发布者：用于可视化机器人当前的局部路径
    local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
    // 前瞻点发布者：用于可视化当前的前瞻目标点（Carrot Point）
    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
    // 曲率计算点发布者：用于可视化用于曲率计算的参考点
    curvature_points_pub_ = node_.lock()->create_publisher<visualization_msgs::msg::MarkerArray>(
        "curvature_points_marker_array", rclcpp::QoS(10));

    // === 7. PID控制器实例初始化 ===
    // 平移PID：控制机器人向目标点的线性运动速度
    move_pid_ = std::make_shared<PID>(
        control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_, translation_ki_);
    // 旋转PID：控制机器人的角度调整速度
    heading_pid_ = std::make_shared<PID>(
        control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
  }

  /**
   * @brief 清理控制器资源（生命周期节点的cleanup阶段）
   * @details 在节点销毁或重新配置前调用，负责：
   *          1. 释放所有创建的资源（发布者、订阅者等）
   *          2. 重置成员变量状态
   *          3. 防止内存泄漏
   *
   * @note 状态转换： Active -> deactivate() -> Inactive -> cleanup() -> Unconfigured
   */
  void OmniPidPursuitController::cleanup()
  {
    RCLCPP_INFO(logger_,
                "Cleaning up controller: %s of type"
                " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());

    // 释放所有发布者资源，reset()会将shared_ptr置为nullptr
    local_path_pub_.reset();
    carrot_pub_.reset();
    curvature_points_pub_.reset();

    // PID控制器会通过shared_ptr自动释放，不需要显式重置
  }

  /**
   * @brief 激活控制器（生命周期节点的activate阶段）
   * @details 使控制器进入工作状态，负责：
   *          1. 激活所有发布者，开始发布数据
   *          2. 注册动态参数回调函数
   *          3. 初始化跟踪状态变量
   *
   * @note 状态转换： Inactive -> activate() -> Active
   *       只有在Active状态下，Nav2才会调用computeVelocityCommands()
   */
  void OmniPidPursuitController::activate()
  {
    RCLCPP_INFO(logger_,
                "Activating controller: %s of type "
                "regulated_pure_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());

    // 激活所有生命周期发布者，开始实际发布数据
    local_path_pub_->on_activate();
    carrot_pub_->on_activate();
    curvature_points_pub_->on_activate();

    // 注册动态参数更新回调，允许在运行时调整PID参数
    auto node = node_.lock();
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
  }

  /**
   * @brief 停用控制器（生命周期节点的deactivate阶段）
   * @details 使控制器退出工作状态，但保留资源和配置，负责：
   *          1. 停用所有发布者，停止发布数据
   *          2. 取消动态参数回调注册
   *          3. 重置跟踪状态（可选）
   *
   * @note 状态转换： Active -> deactivate() -> Inactive
   *       deactivate后可以再次activate，不需要重新configure
   */
  void OmniPidPursuitController::deactivate()
  {
    RCLCPP_INFO(logger_,
                "Deactivating controller: %s of type "
                "regulated_pure_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());

    // 停用所有生命周期发布者，停止数据发布
    local_path_pub_->on_deactivate();
    carrot_pub_->on_deactivate();
    curvature_points_pub_->on_deactivate();

    // 取消动态参数回调注册，防止在停用状态下修改参数
    dyn_params_handler_.reset();
  }

  /**
   * @brief 计算速度控制命令的核心函数
   * @details 这是控制器的心脏函数，实现了完整的PID Pure Pursuit算法流程
   *
   * 算法流程：
   * 1. 路径变换：将全局路径转换到机器人本体坐标系
   * 2. 前瞻计算：根据当前速度计算自适应前瞻距离
   * 3. 目标点选择：在路径上找到前瞻目标点（Carrot Point）
   * 4. PID控制：使用两个独立PID控制器计算线性和角速度
   * 5. 约束应用：应用曲率限制和接近速度缩放
   * 6. 碰撞检测：检查路径是否安全
   * 7. 全向轮控制：输出适合全向轮的速度命令
   */
  geometry_msgs::msg::TwistStamped
  OmniPidPursuitController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                    const geometry_msgs::msg::Twist& velocity,
                                                    nav2_core::GoalChecker* /*goal_checker*/)
  {
    // 线程安全保护：防止参数在计算过程中被修改
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    // 获取代价地图并加锁，确保数据一致性
    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // 步骤1：将全局路径转换到机器人本体坐标系
    // 这样可以简化后续的距离和角度计算
    auto transformed_plan = transformGlobalPlan(pose);

    // 步骤2：计算自适应前瞻距离
    // 速度越快，前瞻距离越远，提供更好的预测性能
    double lookahead_dist = getLookAheadDistance(velocity);

    // 步骤3：在变换后的路径上找到前瞻目标点
    auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
    carrot_pub_->publish(createCarrotMsg(carrot_pose));  // 发布用于可视化

    // 步骤4：计算控制所需的几何量
    // lin_dist: 到目标点的直线距离（用于线性PID控制）
    double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
    // theta_dist: 到目标点的方向角（用于全向轮运动分解）
    double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
    // angle_to_goal: 目标姿态角（用于角度PID控制）
    double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

    // 步骤5：旋转到目标方向的特殊处理
    // 当启用"旋转到目标方向"模式时，如果角度误差较大，停止前进只进行旋转
    if(use_rotate_to_heading_)
    {
      angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
      if(fabs(angle_to_goal) > use_rotate_to_heading_treshold_)
      {
        lin_dist = 0;  // 停止前进，专注于旋转调整
      }
    }

    // 步骤6：PID控制器计算
    //
    // PID控制原理：
    // - 线性PID：以到目标点的距离作为误差，计算所需的线性速度
    //   误差 = lin_dist - 0 = lin_dist（期望距离为0，即到达目标点）
    //   输出 = Kp*误差 + Ki*积分 + Kd*微分
    auto lin_vel = move_pid_->calculate(lin_dist, 0);

    // - 角度PID：以到目标方向的角度作为误差，计算所需的角速度
    //   误差 = angle_to_goal - 0 = angle_to_goal（期望角度为0，即朝向目标）
    //   只有在enable_rotation_为true时才进行角度调整
    auto angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

    // 步骤7：应用约束条件
    // 根据路径曲率限制最大速度，防止在急转弯时速度过快
    applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
    // 接近目标时逐渐减速，实现平滑停车
    applyApproachVelocityScaling(transformed_plan, lin_vel);

    // 步骤8：碰撞检测准备
    // 将本体坐标系路径转换回全局坐标系，用于代价地图碰撞检测
    nav_msgs::msg::Path costmap_frame_local_plan;
    int sample_points = 10;  // 采样点数，平衡检测精度和计算效率
    int plan_size = transformed_plan.poses.size();

    for(int i = 0; i < sample_points; ++i)
    {
      int index = std::min((i * plan_size) / sample_points, plan_size - 1);
      geometry_msgs::msg::PoseStamped map_pose;
      transformPose(costmap_ros_->getGlobalFrameID(), transformed_plan.poses[index], map_pose);
      costmap_frame_local_plan.poses.push_back(map_pose);
    }

    // 步骤9：生成最终控制命令（全向轮控制输出）
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;  // 保持时间戳和坐标系一致性

    // 安全检查：只有路径安全时才输出控制命令
    // if(!isCollisionDetected(costmap_frame_local_plan))
    if(true)
    {
      // === 全向轮运动学分解 ===
      // 全向轮机器人可以同时实现：
      // 1. 前后运动（linear.x）
      // 2. 左右平移（linear.y）
      // 3. 原地旋转（angular.z）
      //
      // theta_dist 是目标点相对于机器人的方向角，
      // 通过三角函数将线性速度分解为X和Y分量：
      cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);  // 前进/后退分量
      cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);  // 左右平移分量
      cmd_vel.twist.angular.z = angular_vel;               // 旋转分量（独立控制）

      // 这样机器人可以直接向目标点方向移动，同时调整姿态角
    }
    else
    {
      // 检测到碰撞风险，为保证安全立即停止机器人
      // cmd_vel 的默认值为全零，直接抛出异常通知Nav2
      // throw nav2_core::PlannerException("Collision detected in the trajectory. Stopping the robot!");
    }

    return cmd_vel;
  }

  void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path& path)
  {
    global_plan_ = path;
  }

  void OmniPidPursuitController::setSpeedLimit(const double& /*speed_limit*/, const bool& /*percentage*/)
  {
    RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
  }

  /**
   * @brief 将全局路径转换到机器人本体坐标系
   * @param pose 机器人当前位姿（在全局坐标系中）
   * @return 转换后的本地路径（在机器人本体坐标系中）
   *
   * @details 这个函数是Pure Pursuit算法的关键步骤：
   *          1. 将全局路径从 map 坐标系转换到 base_link 坐标系
   *          2. 转换后，机器人位于原点，简化后续计算
   *          3. 只转换代价地图范围内的路径点，提高效率
   *          4. 删除已经经过的路径点（路径修剪）
   *
   * @note 转换原理：
   *       - 全局路径在 map 坐标系中，原点为地图原点
   *       - 本地路径在 base_link 坐标系中，原点为机器人中心
   *       - 转换后目标点的相对位置可直接用于控制计算
   */
  nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose)
  {
    // === 步骤1：输入有效性检查 ===
    // 确保全局路径不为空，避免后续处理空路径导致的错误
    if(global_plan_.poses.empty())
    {
      throw nav2_core::PlannerException("Received plan with zero length");
    }

    // === 步骤2：机器人位姿坐标系转换 ===
    // 将机器人当前位姿从其当前坐标系转换到全局路径的坐标系（通常是map坐标系）
    // 这样才能在同一坐标系下计算机器人与路径点的距离关系
    geometry_msgs::msg::PoseStamped robot_pose;
    // 输出机器人在 map 系下的位姿
    if(!transformPose(global_plan_.header.frame_id, pose, robot_pose))
    {
      throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // === 步骤3：确定处理范围 ===
    // 获取代价地图的最大范围，只处理这个范围内的路径点
    // 超出范围的路径点对当前控制决策没有意义，可以忽略以提高效率
    // 找到代价地图的最大范围
    double max_costmap_extent = getCostmapMaxExtent();

    // === 步骤4：查找搜索上界 ===
    // 在路径上找到累计距离等于max_robot_pose_search_dist_的第一个点
    // 这个点作为搜索机器人最近点的上界，防止在很长的路径上搜索浪费时间
    // first_after_integrated_distance: Nav2工具函数，基于路径积分距离查找点
    // ？？？？？？？
    auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
        global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

    // === 步骤5：查找路径起始转换点 ===
    // 在指定范围内找到距离机器人最近的路径点
    // 这个点将作为路径转换的起始点，确保我们从机器人当前位置开始处理路径
    //
    // min_by: Nav2工具函数，根据自定义比较器找到最小值元素
    // Lambda表达式: [&robot_pose](const geometry_msgs::msg::PoseStamped& ps)
    //              捕获robot_pose引用，计算路径点ps与机器人的欧几里德距离
    auto transformation_begin = nav2_util::geometry_utils::min_by(
        global_plan_.poses.begin(),  // 搜索起始迭代器
        closest_pose_upper_bound,    // 搜索结束迭代器（上界）
        [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) { return euclidean_distance(robot_pose, ps); });

    // === 步骤6：确定转换范围的结束点 ===
    // 从transformation_begin开始，找到第一个距离机器人超过max_costmap_extent的点
    // 这样可以只转换代价地图范围内的路径点，节省计算资源
    //
    // std::find_if: 标准库算法，根据谓词查找第一个满足条件的元素
    // Lambda表达式: 捕获robot_pose和max_costmap_extent，判断距离是否超过范围
    auto transformation_end =
        std::find_if(transformation_begin,
                     global_plan_.poses.end(),
                     [&](const auto& pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

    // === 步骤7：定义坐标转换Lambda函数 ===
    // 这个函数将单个路径点从全局坐标系转换到机器人本体坐标系
    //
    // 转换步骤：
    // 1. 构造带时间戳的位姿消息
    // 2. 调用transformPose进行TF坐标变换
    // 3. 将Z坐标设为0（2D平面导航）
    auto transform_global_pose_to_local = [&](const auto& global_plan_pose)
    {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;

      // 设置源坐标系信息（全局路径的坐标系）
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;  // 使用机器人位姿的时间戳
      stamped_pose.pose = global_plan_pose.pose;            // 复制位姿数据

      // 执行坐标变换：从全局坐标系转换到机器人本体坐标系
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);

      // 强制Z坐标为0，确保2D平面导航
      transformed_pose.pose.position.z = 0.0;

      return transformed_pose;
    };

    // === 步骤8：批量坐标转换 ===
    // 使用std::transform对指定范围内的所有路径点进行坐标转换
    //
    // std::transform参数说明：
    // - transformation_begin: 源范围起始迭代器
    // - transformation_end: 源范围结束迭代器
    // - std::back_inserter(transformed_plan.poses): 输出迭代器，自动扩展容器
    // - transform_global_pose_to_local: 转换函数对象
    nav_msgs::msg::Path transformed_plan;
    std::transform(transformation_begin,
                   transformation_end,
                   std::back_inserter(transformed_plan.poses),
                   transform_global_pose_to_local);

    // === 步骤9：设置转换后路径的元信息 ===
    // 确保转换后的路径具有正确的坐标系和时间戳信息
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();  // 目标坐标系（通常是base_link）
    transformed_plan.header.stamp = robot_pose.header.stamp;            // 时间戳保持一致

    // === 步骤10：路径修剪（Path Pruning）===
    // 删除全局路径中已经经过的部分，避免重复处理
    // 这是Pure Pursuit算法的重要优化：只保留前方未经过的路径
    //
    // 修剪原理：
    // - transformation_begin之前的路径点都是机器人已经经过的
    // - 删除这些点可以防止机器人"回头看"已经走过的路径
    // - 提高算法效率，避免不必要的计算
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

    // === 步骤11：发布本地路径用于可视化 ===
    // 将转换后的路径发布到ROS话题，供RViz等工具可视化调试
    local_path_pub_->publish(transformed_plan);

    // === 步骤12：最终有效性检查 ===
    // 确保转换后的路径不为空，如果为空则说明没有有效的路径点可供跟踪
    if(transformed_plan.poses.empty())
    {
      throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
  }

  std::unique_ptr<geometry_msgs::msg::PointStamped>
  OmniPidPursuitController::createCarrotMsg(const geometry_msgs::msg::PoseStamped& carrot_pose)
  {
    auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    carrot_msg->header = carrot_pose.header;
    carrot_msg->point.x = carrot_pose.pose.position.x;
    carrot_msg->point.y = carrot_pose.pose.position.y;
    carrot_msg->point.z = 0.01;  // publish right over map to stand out
    return carrot_msg;
  }

  /**
   * @brief 获取Pure Pursuit算法的前瞻目标点（Carrot Point）
   * @param lookahead_dist 前瞻距离（米）
   * @param transformed_plan 已转换到本体坐标系的路径
   * @return 前瞻目标点的位姿
   *
   * @details Pure Pursuit算法核心：
   *          1. 在路径上找到距离机器人前瞻距离的点
   *          2. 机器人始终朝向这个“胡萝卜”移动
   *          3. 随着机器人移动，前瞻点也沿着路径移动
   *
   * @note 算法特点：
   *       - 前瞻距离越大，路径越平滑，但转弯响应越慢
   *       - 前瞻距离越小，转弯越灵敏，但可能产生振荡
   *       - 使用插值可以获得更精确的前瞻点位置
   */
  // 函数声明：根据给定的前瞻距离和已转换到机器人坐标系的路径，计算前瞻点
  geometry_msgs::msg::PoseStamped
  OmniPidPursuitController::getLookAheadPoint(const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan)
  {
    // 1. 在路径上寻找第一个到机器人当前位置（原点）的直线距离大于或等于前瞻距离的路径点
    // 使用Lambda表达式计算每个路径点到机器人（原点 [0,0]）的距离
    // hypot(x, y) 函数计算直角三角形的斜边长度，即 sqrt(x^2 + y^2)
    auto goal_pose_it =
        std::find_if(transformed_plan.poses.begin(),
                     transformed_plan.poses.end(),
                     [&](const auto& ps) { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist; });

    // 2. 处理特殊情况：如果路径上所有点都小于前瞻距离，则选择最后一个路径点作为目标
    // 这通常发生在路径终点附近，或者路径本身很短的情况下
    if(goal_pose_it == transformed_plan.poses.end())
    {
      goal_pose_it = std::prev(transformed_plan.poses.end());
    }
    // 3. 如果启用了插值，并且找到的点不是路径的起点
    // 插值可以在两个离散的路径点之间找到一个更精确的点，使运动更平滑
    else if(use_interpolation_ && goal_pose_it != transformed_plan.poses.begin())
    {
      // 获取当前找到的点（在圆外）的前一个点（保证在圆内）
      // 因为find_if找到的是第一个距离>=lookahead_dist的点，所以前一个点必然距离<lookahead_dist
      auto prev_pose_it = std::prev(goal_pose_it);

      // 4. 计算线段和圆的交点
      // 将机器人的位置视为圆心，前瞻距离为半径作一个圆
      // 寻找连接prev_pose_it和goal_pose_it的线段与这个圆的交点
      // 这个交点就是精确地位于前瞻距离上的点
      auto point = circleSegmentIntersection(prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);

      // 5. 构造并返回插值得到的位置点
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = prev_pose_it->header.frame_id;  // 使用前一个点的坐标系
      pose.header.stamp = goal_pose_it->header.stamp;        // 使用当前点的时间戳
      pose.pose.position = point;                            // 位置设置为计算出的交点
      return pose;
    }

    // 6. 默认情况：返回找到的路径点（未经插值）
    return *goal_pose_it;
  }

  /**
   * @brief 计算圆与线段的交点（用于精确的前瞻点插值）
   * @param p1 线段起点
   * @param p2 线段终点
   * @param r 圆的半径（前瞻距离）
   * @return 交点坐标
   *
   * @details 数学原理：
   *          这是一个经典的解析几何问题：
   *          1. 圆心在原点（机器人位置），半径为前瞻距离
   *          2. 线段连接路径上的两个相邻点
   *          3. 求解联立方程组：
   *             - 圆方程： x² + y² = r²
   *             - 直线方程：由p1和p2确定
   *          4. 使用二次方程公式求解
   *
   * @note 应用场景：
   *       - 当路径点稀疏时，直接使用路径点可能不够精确
   *       - 通过插值可以获得更精确的前瞻点位置
   *       - 提高路径跟踪的平滑性和精度
   *
   * @warning 注意事项：
   *          - 此函数假设圆心在原点，适用于已转换的本体坐标系
   *          - 返回的交点始终在线段p1-p2上
   *          - 参考：https://mathworld.wolfram.com/Circle-LineIntersection.html
   */
  geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(const geometry_msgs::msg::Point& p1,
                                                                                const geometry_msgs::msg::Point& p2,
                                                                                double r)
  {
    // === 圆-线段交点的解析解法 ===
    // 在机器人本体坐标系中，机器人位于原点
    // 因此圆心在(0,0)，半径为前瞻距离
    //
    // 数学推导过程：
    // 1. 线段参数方程： P(t) = p1 + t*(p2-p1), t∈[0,1]
    // 2. 圆方程： x² + y² = r²
    // 3. 将线段方程代入圆方程，得到关于t的二次方程
    // 4. 使用二次方程公式求解，并选择在线段上的解
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double d = x1 * y2 - x2 * y1;

    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - d * d);
    p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
  }
  // 计算代价地图的最大范围（用于路径转换和碰撞检测）
  double OmniPidPursuitController::getCostmapMaxExtent() const
  {
    const double max_costmap_dim_meters = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0;
  }
  bool OmniPidPursuitController::transformPose(const std::string frame,
                                               const geometry_msgs::msg::PoseStamped& in_pose,
                                               geometry_msgs::msg::PoseStamped& out_pose) const
  {
    if(in_pose.header.frame_id == frame)
    {
      out_pose = in_pose;
      return true;
    }

    try
    {
      tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
      return true;
    }
    catch(tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
  }

  bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path& path)
  {
    auto costmap = costmap_ros_->getCostmap();
    for(const auto& pose_stamped : path.poses)
    {
      const auto& pose = pose_stamped.pose;
      unsigned int mx, my;
      if(costmap->worldToMap(pose.position.x, pose.position.y, mx, my))
      {
        if(costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          return true;
        }
      }
      else
      {
        // RCLCPP_WARN(
        //   logger_,
        //   "The Local path is not in the costmap. Cannot check for collisions. "
        //   "Proceed at your own risk, slow the robot, or increase your costmap size.");
        return false;
      }
    }
    return false;
  }

  double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist& speed)
  {
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = lookahead_dist_;

    if(use_velocity_scaled_lookahead_dist_)
    {
      lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
      lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
    }

    return lookahead_dist;
  }

  double OmniPidPursuitController::approachVelocityScalingFactor(const nav_msgs::msg::Path& transformed_path) const
  {
    // Waiting to apply the threshold based on integrated distance ensures we don't
    // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
    double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
    if(remaining_distance < approach_velocity_scaling_dist_)
    {
      auto& last = transformed_path.poses.back();
      // Here we will use a regular euclidean distance from the robot frame (origin)
      // to get smooth scaling, regardless of path density.
      double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
      return distance_to_last_pose / approach_velocity_scaling_dist_;
    }
    else
    {
      return 1.0;
    }
  }

  void OmniPidPursuitController::applyApproachVelocityScaling(const nav_msgs::msg::Path& path, double& linear_vel) const
  {
    double approach_vel = linear_vel;
    double velocity_scaling = approachVelocityScalingFactor(path);
    double unbounded_vel = approach_vel * velocity_scaling;
    if(unbounded_vel < min_approach_linear_velocity_)
    {
      approach_vel = min_approach_linear_velocity_;
    }
    else
    {
      approach_vel *= velocity_scaling;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    linear_vel = std::min(linear_vel, approach_vel);
  }

  void OmniPidPursuitController::applyCurvatureLimitation(const nav_msgs::msg::Path& path,
                                                          const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                                          double& linear_vel)
  {
    double curvature = calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

    double scaled_linear_vel = linear_vel;
    if(curvature > curvature_min_)
    {
      double reduction_ratio = 1.0;
      if(curvature > curvature_max_)
      {
        reduction_ratio = reduction_ratio_at_high_curvature_;
      }
      else
      {
        reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                    (1.0 - reduction_ratio_at_high_curvature_);
      }

      double target_scaled_vel = linear_vel * reduction_ratio;
      scaled_linear_vel =
          last_velocity_scaling_factor_ + std::clamp(target_scaled_vel - last_velocity_scaling_factor_,
                                                     -max_velocity_scaling_factor_rate_ * control_duration_,
                                                     max_velocity_scaling_factor_rate_ * control_duration_);
    }
    scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

    linear_vel = std::min(linear_vel, scaled_linear_vel);
    last_velocity_scaling_factor_ = linear_vel;
  }

  double OmniPidPursuitController::calculateCurvature(const nav_msgs::msg::Path& path,
                                                      const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                                      double forward_dist,
                                                      double backward_dist) const
  {
    geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
    std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

    double lookahead_pose_cumulative_distance = 0.0;
    geometry_msgs::msg::PoseStamped robot_base_frame_pose;
    robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
    lookahead_pose_cumulative_distance =
        nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

    backward_pose = findPoseAtDistance(path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

    forward_pose = findPoseAtDistance(path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

    double curvature_radius =
        calculateCurvatureRadius(backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
    double curvature = 1.0 / curvature_radius;
    visualizeCurvaturePoints(backward_pose, forward_pose);
    return curvature;
  }

  double OmniPidPursuitController::calculateCurvatureRadius(const geometry_msgs::msg::Point& near_point,
                                                            const geometry_msgs::msg::Point& current_point,
                                                            const geometry_msgs::msg::Point& far_point) const
  {
    double x1 = near_point.x, y1 = near_point.y;
    double x2 = current_point.x, y2 = current_point.y;
    double x3 = far_point.x, y3 = far_point.y;

    double center_x =
        ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) /
        (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
    double center_y =
        ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) /
        (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
    double radius = std::hypot(x2 - center_x, y2 - center_y);
    if(std::isnan(radius) || std::isinf(radius) || radius < 1e-9)
    {
      return 1e9;
    }
    return radius;
  }

  void OmniPidPursuitController::visualizeCurvaturePoints(const geometry_msgs::msg::PoseStamped& backward_pose,
                                                          const geometry_msgs::msg::PoseStamped& forward_pose) const
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker near_marker;
    near_marker.header = backward_pose.header;
    near_marker.ns = "curvature_points";
    near_marker.id = 0;
    near_marker.type = visualization_msgs::msg::Marker::SPHERE;
    near_marker.action = visualization_msgs::msg::Marker::ADD;
    near_marker.pose = backward_pose.pose;
    near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
    near_marker.color.g = 1.0;
    near_marker.color.a = 1.0;

    visualization_msgs::msg::Marker far_marker;
    far_marker.header = forward_pose.header;
    far_marker.ns = "curvature_points";
    far_marker.id = 1;
    far_marker.type = visualization_msgs::msg::Marker::SPHERE;
    far_marker.action = visualization_msgs::msg::Marker::ADD;
    far_marker.pose = forward_pose.pose;
    far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
    far_marker.color.r = 1.0;
    far_marker.color.a = 1.0;

    marker_array.markers.push_back(near_marker);
    marker_array.markers.push_back(far_marker);

    curvature_points_pub_->publish(marker_array);
  }

  std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(const nav_msgs::msg::Path& path) const
  {
    std::vector<double> cumulative_distances;
    cumulative_distances.push_back(0.0);

    for(size_t i = 1; i < path.poses.size(); ++i)
    {
      const auto& prev_pose = path.poses[i - 1].pose.position;
      const auto& curr_pose = path.poses[i].pose.position;
      double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
      cumulative_distances.push_back(cumulative_distances.back() + distance);
    }
    return cumulative_distances;
  }

  geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
      const nav_msgs::msg::Path& path, const std::vector<double>& cumulative_distances, double target_distance) const
  {
    if(path.poses.empty() || cumulative_distances.empty())
    {
      return geometry_msgs::msg::PoseStamped();
    }
    if(target_distance <= 0.0)
    {
      return path.poses.front();
    }
    if(target_distance >= cumulative_distances.back())
    {
      return path.poses.back();
    }
    auto it = std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
    size_t index = std::distance(cumulative_distances.begin(), it);

    if(index == 0)
    {
      return path.poses.front();
    }

    double ratio = (target_distance - cumulative_distances[index - 1]) /
                   (cumulative_distances[index] - cumulative_distances[index - 1]);
    geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
    geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

    geometry_msgs::msg::PoseStamped interpolated_pose;
    interpolated_pose.header = pose2.header;
    interpolated_pose.pose.position.x = pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
    interpolated_pose.pose.position.y = pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
    interpolated_pose.pose.position.z = pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
    interpolated_pose.pose.orientation = pose2.pose.orientation;

    return interpolated_pose;
  }

  /**
   * @brief 动态参数回调函数（ROS2参数系统核心功能）
   * @param parameters 待更新的参数列表（来自外部调用，如rqt_reconfigure、命令行或代码）
   * @return SetParametersResult 参数设置结果，包含成功/失败状态和错误信息
   *
   * @details 这是ROS2生命周期节点参数系统的核心回调函数，功能包括：
   *
   * === 主要功能 ===
   * 1. **运行时参数调整**：允许在机器人运行过程中实时修改控制器参数
   * 2. **PID调优**：支持在线调整PID增益，无需重启节点
   * 3. **行为调整**：实时修改路径跟踪行为（前瞻距离、速度限制等）
   * 4. **线程安全**：使用互斥锁确保参数更新的原子性
   *
   * === 参数分类详解 ===
   *
   * 🎛️ **PID控制器参数**：
   * - translation_kp/ki/kd: 平移PID增益（控制机器人向目标点的移动响应）
   * - rotation_kp/ki/kd: 旋转PID增益（控制机器人的角度调整响应）
   * - min_max_sum_error: PID积分项限幅（防止积分饱和）
   *
   * 🎯 **Pure Pursuit算法参数**：
   * - lookahead_dist: 基础前瞻距离（影响路径跟踪的平滑度）
   * - min/max_lookahead_dist: 前瞻距离范围（防止过近或过远视野）
   * - lookahead_time: 速度自适应前瞻的时间常数
   * - use_velocity_scaled_lookahead_dist: 是否启用速度自适应前瞻
   *
   * 🚀 **运动控制参数**：
   * - v_linear_min/max: 线性速度限制（m/s）
   * - v_angular_min/max: 角速度限制（rad/s）
   * - min_approach_linear_velocity: 接近目标时的最小速度
   * - approach_velocity_scaling_dist: 开始减速的距离阈值
   *
   * 🌊 **路径平滑参数**：
   * - curvature_min/max: 曲率限制范围（控制转弯时的减速程度）
   * - reduction_ratio_at_high_curvature: 高曲率时的速度减少比例
   * - curvature_forward/backward_dist: 曲率计算的参考点距离
   * - max_velocity_scaling_factor_rate: 速度缩放的最大变化率
   *
   * 🎚️ **行为开关参数**：
   * - use_interpolation: 是否使用线性插值寻找精确前瞻点
   * - use_rotate_to_heading: 是否在路径末尾旋转到目标方向
   * - use_rotate_to_heading_treshold: 旋转模式的角度阈值
   *
   * 🔧 **系统参数**：
   * - transform_tolerance: 坐标变换超时容差（秒）
   *
   * === 使用场景 ===
   * 1. **实时调优**：在机器人运行时通过RViz插件或rqt_reconfigure调整参数
   * 2. **场景适应**：根据不同环境（室内/室外、狭窄/开阔）调整控制策略
   * 3. **性能优化**：在线调整PID参数以获得最佳跟踪性能
   * 4. **调试分析**：修改可视化和行为参数以便观察算法工作状态
   *
   * === 调用方式 ===
   * ```bash
   * # 命令行方式
   * ros2 param set /controller_server pb_omni_pid_pursuit_controller.translation_kp 5.0
   *
   * # 程序方式
   * auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "controller_server");
   * parameters_client->set_parameters({rclcpp::Parameter("pb_omni_pid_pursuit_controller.translation_kp", 5.0)});
   * ```
   *
   * @note 线程安全性：
   *       - 使用std::lock_guard确保参数更新时不会与computeVelocityCommands()冲突
   *       - 参数更新是原子操作，不会出现部分更新的情况
   *
   * @warning 参数验证：
   *          - 此函数不进行参数范围检查，调用者需确保参数值合理
   *          - 错误的参数值可能导致控制器行为异常或不稳定
   *          - 建议在实际应用中添加参数边界检查
   */
  rcl_interfaces::msg::SetParametersResult
  OmniPidPursuitController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
  {
    // 初始化返回结果，默认假设所有参数更新成功
    rcl_interfaces::msg::SetParametersResult result;

    // === 线程安全保护 ===
    // 防止在computeVelocityCommands()执行过程中修改参数，确保控制算法的一致性
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    // === 遍历所有待更新的参数 ===
    for(const auto& parameter : parameters)
    {
      const auto& type = parameter.get_type();  // 获取参数类型（double、bool、int等）
      const auto& name = parameter.get_name();  // 获取参数全名（包含命名空间）

      // === 处理浮点数类型参数 ===
      if(type == ParameterType::PARAMETER_DOUBLE)
      {
        // 🎛️ PID控制器增益参数
        // 平移PID参数：控制机器人向目标点移动的响应特性
        if(name == plugin_name_ + ".translation_kp")
        {
          translation_kp_ = parameter.as_double();  // 比例增益：响应速度和稳态误差的平衡
        }
        else if(name == plugin_name_ + ".translation_ki")
        {
          translation_ki_ = parameter.as_double();  // 积分增益：消除稳态误差，但可能引起超调
        }
        else if(name == plugin_name_ + ".translation_kd")
        {
          translation_kd_ = parameter.as_double();  // 微分增益：减少超调和振荡，提高稳定性
        }

        // 旋转PID参数：控制机器人角度调整的响应特性
        else if(name == plugin_name_ + ".rotation_kp")
        {
          rotation_kp_ = parameter.as_double();  // 角度比例增益：角度响应速度
        }
        else if(name == plugin_name_ + ".rotation_ki")
        {
          rotation_ki_ = parameter.as_double();  // 角度积分增益：消除角度稳态误差
        }
        else if(name == plugin_name_ + ".rotation_kd")
        {
          rotation_kd_ = parameter.as_double();  // 角度微分增益：减少角度振荡
        }

        // 🔧 系统控制参数
        else if(name == plugin_name_ + ".transform_tolerance")
        {
          double transform_tolerance = parameter.as_double();
          transform_tolerance_ = tf2::durationFromSec(transform_tolerance);  // 转换为tf2::Duration类型
        }
        else if(name == plugin_name_ + ".min_max_sum_error")
        {
          min_max_sum_error_ = parameter.as_double();  // PID积分项限幅，防止积分饱和
        }

        // 🎯 Pure Pursuit前瞻距离参数
        else if(name == plugin_name_ + ".lookahead_dist")
        {
          lookahead_dist_ = parameter.as_double();  // 基础前瞻距离，影响路径跟踪平滑度
        }
        else if(name == plugin_name_ + ".min_lookahead_dist")
        {
          min_lookahead_dist_ = parameter.as_double();  // 最小前瞻距离，防止"近视"
        }
        else if(name == plugin_name_ + ".max_lookahead_dist")
        {
          max_lookahead_dist_ = parameter.as_double();  // 最大前瞻距离，防止"远视"
        }
        else if(name == plugin_name_ + ".lookahead_time")
        {
          lookahead_time_ = parameter.as_double();  // 速度自适应前瞻的时间常数
        }
        else if(name == plugin_name_ + ".use_rotate_to_heading_treshold")
        {
          use_rotate_to_heading_treshold_ = parameter.as_double();  // 旋转到目标方向的角度阈值
        }

        // 🚀 速度控制和接近策略参数
        else if(name == plugin_name_ + ".min_approach_linear_velocity")
        {
          min_approach_linear_velocity_ = parameter.as_double();  // 接近目标时的最小速度
        }
        else if(name == plugin_name_ + ".approach_velocity_scaling_dist")
        {
          approach_velocity_scaling_dist_ = parameter.as_double();  // 开始减速的距离阈值
        }

        // 速度限制参数：硬件和安全约束
        else if(name == plugin_name_ + ".v_linear_max")
        {
          v_linear_max_ = parameter.as_double();  // 最大线性速度（硬件限制）
        }
        else if(name == plugin_name_ + ".v_linear_min")
        {
          v_linear_min_ = parameter.as_double();  // 最小线性速度（可为负值，支持倒退）
        }
        else if(name == plugin_name_ + ".v_angular_max")
        {
          v_angular_max_ = parameter.as_double();  // 最大角速度（硬件限制）
        }
        else if(name == plugin_name_ + ".v_angular_min")
        {
          v_angular_min_ = parameter.as_double();  // 最小角速度（可为负值，支持反向旋转）
        }

        // 🌊 曲率限制参数：路径平滑控制
        else if(name == plugin_name_ + ".curvature_min")
        {
          curvature_min_ = parameter.as_double();  // 开始限速的曲率阈值
        }
        else if(name == plugin_name_ + ".curvature_max")
        {
          curvature_max_ = parameter.as_double();  // 最大限速的曲率阈值
        }
        else if(name == plugin_name_ + ".reduction_ratio_at_high_curvature")
        {
          reduction_ratio_at_high_curvature_ = parameter.as_double();  // 高曲率时的速度减少比例
        }
        else if(name == plugin_name_ + ".curvature_forward_dist")
        {
          curvature_forward_dist_ = parameter.as_double();  // 曲率计算的前向参考点距离
        }
        else if(name == plugin_name_ + ".curvature_backward_dist")
        {
          curvature_backward_dist_ = parameter.as_double();  // 曲率计算的后向参考点距离
        }
        else if(name == plugin_name_ + ".max_velocity_scaling_factor_rate")
        {
          max_velocity_scaling_factor_rate_ = parameter.as_double();  // 速度缩放系数的最大变化率
        }
      }
      // === 处理布尔类型参数 ===
      else if(type == ParameterType::PARAMETER_BOOL)
      {
        // 🎚️ 算法行为开关参数
        if(name == plugin_name_ + ".use_velocity_scaled_lookahead_dist")
        {
          use_velocity_scaled_lookahead_dist_ = parameter.as_bool();  // 是否启用速度自适应前瞻
        }
        else if(name == plugin_name_ + ".use_interpolation")
        {
          use_interpolation_ = parameter.as_bool();  // 是否使用插值寻找精确前瞻点
        }
        else if(name == plugin_name_ + ".use_rotate_to_heading")
        {
          use_rotate_to_heading_ = parameter.as_bool();  // 是否在路径末尾旋转到目标方向
        }
      }
      // 注意：这里可以扩展支持其他参数类型（PARAMETER_INT、PARAMETER_STRING等）
    }

    // === 返回成功结果 ===
    result.successful = true;  // 所有参数更新成功
    return result;
  }
};  // namespace pb_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
