/**
 * @file parameters.cpp
 * @brief Point-LIO系统参数管理实现文件
 * @details 该文件实现了Point-LIO系统中所有参数的初始化、读取和管理功能，包括：
 *          - ROS2参数的声明和读取
 *          - 滤波器协方差矩阵的初始化
 *          - 工具函数的实现 (欧拉角转换、日志文件管理等)
 * @author Point-LIO Team
 */

#include "parameters.h"

#include <cmath>

// === 系统状态和时间管理变量定义 ===
bool is_first_frame = true;          // 标识是否为第一帧数据，用于系统初始化
double lidar_end_time = 0.0;         // 当前激光雷达帧的结束时间戳
double first_lidar_time = 0.0;       // 第一帧激光雷达的时间戳，作为时间基准
double time_con = 0.0;               // 时间连续性检查变量
double last_timestamp_lidar = -1.0;  // 上一帧激光雷达的时间戳
double last_timestamp_imu = -1.0;    // 上一帧IMU数据的时间戳
int pcd_index = 0;                   // PCD文件保存的索引编号
// === iVox增量式地图配置 ===
IVoxType::Options ivox_options_;  // iVox地图的配置选项
int ivox_nearby_type = 6;         // iVox邻近点搜索类型 (6=NEARBY6，18=NEARBY18，26=NEARBY26)

// === 传感器外参标定参数 ===
std::vector<double> extrinT(3, 0.0);  // 激光雷达到IMU的平移外参 [tx, ty, tz]
std::vector<double> extrinR(9, 0.0);  // 激光雷达到IMU的旋转外参 (3x3矩阵展开)
// === 滤波器状态向量 ===
state_input state_in;    // 输入模式状态向量 (24维) - IMU作为控制输入
state_output state_out;  // 输出模式状态向量 (30维) - IMU作为观测
// === 传感器配置参数 ===
std::string lid_topic, imu_topic;  // 激光雷达和IMU数据话题名称

// === 算法核心控制参数 ===
bool prop_at_freq_of_imu = true;  // 是否以IMU频率进行状态传播
bool check_satu = true;           // 是否检查IMU数据饱和
bool con_frame = false;           // 是否连接多帧点云
bool cut_frame = false;           // 是否切分单帧点云
bool use_imu_as_input = false;    // IMU使用模式 (false: 作为观测, true: 作为输入)
bool space_down_sample = true;    // 是否进行空间降采样
bool publish_odometry_without_downsample = false;  // 是否发布未降采样的高频里程计
bool print_cloud_input_fps = false;                // 是否打印运行频率
bool debug_pose_update_detail = false;             // 是否打印位姿更新细粒度诊断
bool blind_center_enable = false;                   // 是否将blind球心迁移到配置位置
std::vector<double> blind_center{0.0, 0.20, 0.0};  // blind球心在输入点云坐标系中的位置
// === 地图初始化参数 ===
int init_map_size = 10;  // 初始化地图所需的最少特征点数量
int con_frame_num = 1;   // 连接帧的数量

// === 几何匹配参数 ===
double match_s = 81;                // 点云匹配的搜索半径参数
float plane_thr = 0.1f;             // 平面特征提取阈值
double filter_size_surf_min = 0.5;  // 表面特征点的降采样尺寸 (米)
double filter_size_map_min = 0.5;   // 地图点的降采样尺寸 (米)
double pose_update_time_bin_ms = 0.0;  // EKF位姿更新批处理时间桶 (毫秒)
double fov_deg = 180;               // 激光雷达视场角 (度)
float DET_RANGE = 450;              // 激光雷达有效检测范围 (米)

// === IMU传感器参数 ===
double satu_acc, satu_gyro;                    // IMU加速度计和陀螺仪饱和值
double cut_frame_time_interval = 0.1;          // 帧切分时间间隔 (秒)
double cloud_input_fps_print_period = 1.0;     // 运行频率打印周期 (秒)
double debug_pose_update_detail_period = 1.0;  // 位姿更新细粒度诊断打印周期 (秒)
// === IMU传感器配置 ===
bool imu_en = true;            // IMU使能标志
double imu_time_inte = 0.005;  // IMU数据积分时间间隔 (秒)
double acc_norm;               // 加速度计单位 (1.0: g, 9.81: m/s²)

// === 噪声模型参数 ===
// 观测噪声
double laser_point_cov = 0.01;              // 激光雷达点云观测噪声协方差
double imu_meas_acc_cov, imu_meas_omg_cov;  // IMU加速度和角速度测量噪声协方差

// 过程噪声
double vel_cov;                         // 速度过程噪声协方差
double acc_cov_input, gyr_cov_input;    // 输入模式下的加速度和角速度过程噪声
double gyr_cov_output, acc_cov_output;  // 输出模式下的角速度和加速度过程噪声
double b_gyr_cov, b_acc_cov;            // 陀螺仪和加速度计偏置随机游走噪声
// === 传感器类型和文件管理 ===
int lidar_type;         // 激光雷达类型标识符
int pcd_save_interval;  // PCD文件保存间隔 (帧数)

// === 重力参数 ===
std::vector<double> gravity_init, gravity;  // 初始和当前估计的重力向量

// === 系统功能控制 ===
bool runtime_pos_log;                     // 运行时位姿日志记录使能
bool pcd_save_en;                         // PCD点云文件保存使能
double accumulated_map_publish_hz = 1.0;  // 累积地图低频发布频率
bool path_en;                             // 轨迹路径发布使能
bool extrinsic_est_en = true;             // 外参在线估计使能
bool scan_pub_en, scan_body_pub_en;       // 点云数据发布使能
bool tf_send_en;                          // TF坐标变换发布使能
// === 模块对象指针 ===
shared_ptr<Preprocess> p_pre;  // 点云预处理模块指针
shared_ptr<ImuProcess> p_imu;  // IMU数据处理模块指针

// === 滤波器时间状态变量 ===
double time_update_last = 0.0;         // 上次滤波器更新时间
double time_current = 0.0;             // 当前处理时间
double time_predict_last_const = 0.0;  // 上次预测时间常量
double t_last = 0.0;                   // 上一个时间点

// === 时间同步参数 ===
double time_diff_lidar_to_imu = 0.0;  // 激光雷达到IMU的时间偏移

// === 先验地图配置 ===
bool enable_prior_pcd;          // 先验PCD地图使能标志
string prior_pcd_map_path;      // 先验PCD地图文件路径
std::vector<double> init_pose;  // 初始位姿 [x,y,z,qx,qy,qz,qw]

// === 进阶时间控制参数 ===
double lidar_time_inte = 0.1;      // 激光雷达数据积分时间 (秒)
double first_imu_time = 0.0;       // 第一个IMU数据的时间戳
int cut_frame_num = 1;             // 帧切分数量
int orig_odom_freq = 200;          // 里程计发布频率上限 (Hz，按传感器时间)
double online_refine_time = 20.0;  // 在线优化时间 (秒)
bool cut_frame_init = false;       // 帧切分初始化标志

// === 数据结构和文件流 ===
MeasureGroup Measures;            // 传感器数据测量组
ofstream fout_out, fout_imu_pbp;  // 日志文件输出流

/**
 * @brief 从ROS2参数服务器读取所有系统参数
 * @param nh ROS2节点的共享指针
 * @details 该函数负责：
 *          1. 初始化预处理和IMU处理模块
 *          2. 声明和读取所有ROS2参数
 *          3. 配置iVox地图参数
 *          4. 设置IMU重力向量
 */
void readParameters(rclcpp::Node & nh)
{
  // === 模块初始化 ===
  p_pre.reset(new Preprocess());  // 初始化点云预处理模块
  p_imu.reset(new ImuProcess());  // 初始化IMU数据处理模块

  // === ROS2参数读取 ===
  try {
    // === 算法核心控制参数 ===
    nh.declare_parameter<bool>("prop_at_freq_of_imu", true);
    nh.get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);

    nh.declare_parameter<bool>("use_imu_as_input", false);  // 关键参数：决定滤波器架构
    nh.get_parameter("use_imu_as_input", use_imu_as_input);

    nh.declare_parameter<bool>("check_satu", true);
    nh.get_parameter("check_satu", check_satu);

    nh.declare_parameter<int>("init_map_size", 100);
    nh.get_parameter("init_map_size", init_map_size);

    nh.declare_parameter<bool>("space_down_sample", true);
    nh.get_parameter("space_down_sample", space_down_sample);

    nh.declare_parameter<double>("mapping.satu_acc", 3.0);
    nh.get_parameter("mapping.satu_acc", satu_acc);

    nh.declare_parameter<double>("mapping.satu_gyro", 35.0);
    nh.get_parameter("mapping.satu_gyro", satu_gyro);

    nh.declare_parameter<double>("mapping.acc_norm", 1.0);
    nh.get_parameter("mapping.acc_norm", acc_norm);

    nh.declare_parameter<float>("mapping.plane_thr", 0.05f);
    nh.get_parameter("mapping.plane_thr", plane_thr);

    nh.declare_parameter<double>("mapping.pose_update_time_bin_ms", 0.0);
    nh.get_parameter("mapping.pose_update_time_bin_ms", pose_update_time_bin_ms);
    if (!std::isfinite(pose_update_time_bin_ms) || pose_update_time_bin_ms < 0.0) {
      RCLCPP_WARN(
        nh.get_logger(),
        "mapping.pose_update_time_bin_ms must be finite and non-negative; falling back to 0.0");
      pose_update_time_bin_ms = 0.0;
    }

    nh.declare_parameter<int>("point_filter_num", 2);
    nh.get_parameter("point_filter_num", p_pre->point_filter_num);

    nh.declare_parameter<std::string>("common.lid_topic", ".livox.lidar");
    nh.get_parameter("common.lid_topic", lid_topic);

    nh.declare_parameter<std::string>("common.imu_topic", ".livox.imu");
    nh.get_parameter("common.imu_topic", imu_topic);

    nh.declare_parameter<bool>("common.con_frame", false);
    nh.get_parameter("common.con_frame", con_frame);

    nh.declare_parameter<int>("common.con_frame_num", 1);
    nh.get_parameter("common.con_frame_num", con_frame_num);

    nh.declare_parameter<bool>("common.cut_frame", false);
    nh.get_parameter("common.cut_frame", cut_frame);

    nh.declare_parameter<double>("common.cut_frame_time_interval", 0.1);
    nh.get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);

    nh.declare_parameter<bool>("common.print_cloud_input_fps", false);
    nh.get_parameter("common.print_cloud_input_fps", print_cloud_input_fps);

    nh.declare_parameter<double>("common.cloud_input_fps_print_period", 1.0);
    nh.get_parameter("common.cloud_input_fps_print_period", cloud_input_fps_print_period);

    nh.declare_parameter<bool>("common.debug_pose_update_detail", false);
    nh.get_parameter("common.debug_pose_update_detail", debug_pose_update_detail);

    nh.declare_parameter<double>("common.debug_pose_update_detail_period", 1.0);
    nh.get_parameter("common.debug_pose_update_detail_period", debug_pose_update_detail_period);
    if (debug_pose_update_detail_period <= 0.05) {
      debug_pose_update_detail_period = 1.0;
    }

    nh.declare_parameter<double>("common.time_diff_lidar_to_imu", 0.0);
    nh.get_parameter("common.time_diff_lidar_to_imu", time_diff_lidar_to_imu);

    nh.declare_parameter<bool>("prior_pcd.enable", false);
    nh.get_parameter("prior_pcd.enable", enable_prior_pcd);

    nh.declare_parameter<string>("prior_pcd.prior_pcd_map_path", "");
    nh.get_parameter("prior_pcd.prior_pcd_map_path", prior_pcd_map_path);

    nh.declare_parameter<std::vector<double>>("prior_pcd.init_pose", std::vector<double>());
    nh.get_parameter("prior_pcd.init_pose", init_pose);

    nh.declare_parameter<double>("filter_size_surf", 0.5);
    nh.get_parameter("filter_size_surf", filter_size_surf_min);

    nh.declare_parameter<double>("filter_size_map", 0.5);
    nh.get_parameter("filter_size_map", filter_size_map_min);

    nh.declare_parameter<float>("mapping.det_range", 300.f);
    nh.get_parameter("mapping.det_range", DET_RANGE);

    nh.declare_parameter<double>("mapping.fov_degree", 180);
    nh.get_parameter("mapping.fov_degree", fov_deg);

    nh.declare_parameter<bool>("mapping.imu_en", true);
    nh.get_parameter("mapping.imu_en", imu_en);

    nh.declare_parameter<bool>("mapping.extrinsic_est_en", true);
    nh.get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

    nh.declare_parameter<double>("mapping.imu_time_inte", 0.005);
    nh.get_parameter("mapping.imu_time_inte", imu_time_inte);

    nh.declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    nh.get_parameter("mapping.lidar_meas_cov", laser_point_cov);

    nh.declare_parameter<double>("mapping.acc_cov_input", 0.1);
    nh.get_parameter("mapping.acc_cov_input", acc_cov_input);

    nh.declare_parameter<double>("mapping.vel_cov", 20);
    nh.get_parameter("mapping.vel_cov", vel_cov);

    nh.declare_parameter<double>("mapping.gyr_cov_input", 0.1);
    nh.get_parameter("mapping.gyr_cov_input", gyr_cov_input);

    nh.declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    nh.get_parameter("mapping.gyr_cov_output", gyr_cov_output);

    nh.declare_parameter<double>("mapping.acc_cov_output", 0.1);
    nh.get_parameter("mapping.acc_cov_output", acc_cov_output);

    nh.declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    nh.get_parameter("mapping.b_gyr_cov", b_gyr_cov);

    nh.declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    nh.get_parameter("mapping.b_acc_cov", b_acc_cov);

    nh.declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    nh.get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);

    nh.declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    nh.get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);

    nh.declare_parameter<double>("preprocess.blind", 1.0);
    nh.get_parameter("preprocess.blind", p_pre->blind);

    nh.declare_parameter<bool>("preprocess.blind_center_enable", false);
    nh.get_parameter("preprocess.blind_center_enable", blind_center_enable);

    nh.declare_parameter<std::vector<double>>(
      "preprocess.blind_center", std::vector<double>{0.0, 0.20, 0.0});
    nh.get_parameter("preprocess.blind_center", blind_center);

    nh.declare_parameter<int>("preprocess.lidar_type", 1);
    nh.get_parameter("preprocess.lidar_type", lidar_type);

    nh.declare_parameter<int>("preprocess.scan_line", 16);
    nh.get_parameter("preprocess.scan_line", p_pre->N_SCANS);

    nh.declare_parameter<int>("preprocess.scan_rate", 10);
    nh.get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);

    nh.declare_parameter<int>("preprocess.timestamp_unit", 1);
    nh.get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

    nh.declare_parameter<double>("mapping.match_s", 81);
    nh.get_parameter("mapping.match_s", match_s);

    nh.declare_parameter<std::vector<double>>("mapping.gravity", std::vector<double>());
    nh.get_parameter("mapping.gravity", gravity);

    nh.declare_parameter<std::vector<double>>("mapping.gravity_init", std::vector<double>());
    nh.get_parameter("mapping.gravity_init", gravity_init);

    nh.declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    nh.get_parameter("mapping.extrinsic_T", extrinT);

    nh.declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());
    nh.get_parameter("mapping.extrinsic_R", extrinR);

    nh.declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    nh.get_parameter("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);

    nh.declare_parameter<int>("odometry.publish_frequency_hz", 200);
    nh.get_parameter("odometry.publish_frequency_hz", orig_odom_freq);
    if (orig_odom_freq <= 0) {
      RCLCPP_WARN(
        nh.get_logger(),
        "odometry.publish_frequency_hz must be positive; falling back to 200 Hz");
      orig_odom_freq = 200;
    }

    nh.declare_parameter<bool>("publish.path_en", true);
    nh.get_parameter("publish.path_en", path_en);

    nh.declare_parameter<bool>("publish.scan_publish_en", true);
    nh.get_parameter("publish.scan_publish_en", scan_pub_en);

    nh.declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    nh.get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

    nh.declare_parameter<bool>("publish.tf_send_en", true);
    nh.get_parameter("publish.tf_send_en", tf_send_en);

    nh.declare_parameter<bool>("runtime_pos_log_enable", false);
    nh.get_parameter("runtime_pos_log_enable", runtime_pos_log);

    nh.declare_parameter<bool>("pcd_save.pcd_save_en", false);
    nh.get_parameter("pcd_save.pcd_save_en", pcd_save_en);

    nh.declare_parameter<int>("pcd_save.interval", -1);
    nh.get_parameter("pcd_save.interval", pcd_save_interval);

    nh.declare_parameter<double>("pcd_save.accumulated_map_publish_hz", 1.0);
    nh.get_parameter("pcd_save.accumulated_map_publish_hz", accumulated_map_publish_hz);

    nh.declare_parameter<double>("mapping.lidar_time_inte", 0.1);
    nh.get_parameter("mapping.lidar_time_inte", lidar_time_inte);

    nh.declare_parameter<float>("mapping.ivox_grid_resolution", 0.2);
    nh.get_parameter("mapping.ivox_grid_resolution", ivox_options_.resolution_);

    nh.declare_parameter<int>("ivox_nearby_type", 18);
    nh.get_parameter("ivox_nearby_type", ivox_nearby_type);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(nh.get_logger(), "Parameter type exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(nh.get_logger(), "Exception: %s", e.what());
  }

  // === iVox地图配置 ===
  // 根据参数设置iVox邻近点搜索类型
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;  // 仅中心点，最快但精度最低
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;  // 6邻域搜索，平衡性能
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;  // 18邻域搜索，高精度
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;  // 26邻域搜索，最高精度但最慢
  } else {
    // 默认使用NEARBY18作为最佳性能平衡
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
  // === IMU重力向量设置 ===
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);  // 将重力向量设置到IMU处理模块
}

/**
 * @brief 将SO3旋转矩阵转换为欧拉角 (ZYX顺序)
 * @param rot SO3旋转矩阵
 * @return 欧拉角向量 [roll, pitch, yaw] (弧度)
 * @details 使用ZYX欧拉角约定：
 *          1. 先绕Z轴旋转 (Yaw)
 *          2. 再绕Y轴旋转 (Pitch)
 *          3. 最后绕X轴旋转 (Roll)
 *          同时处理万向锁 (Gimbal Lock) 奇异情况
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 & rot)
{
  // 计算sin(yaw)的幅度，用于判断是否接近万向锁状态
  double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));

  // 判断是否处于万向锁奇异状态 (pitch 接近 ±90度)
  bool singular = sy < 1e-6;

  double x, y, z;  // roll, pitch, yaw

  if (!singular) {
    // 正常情况：使用标准ZYX欧拉角提取公式
    x = atan2(rot(2, 1), rot(2, 2));  // Roll:  绕X轴旋转角
    y = atan2(-rot(2, 0), sy);        // Pitch: 绕Y轴旋转角
    z = atan2(rot(1, 0), rot(0, 0));  // Yaw:   绕Z轴旋转角
  } else {
    // 万向锁情况：采用替代计算方法
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;  // 将Yaw角设为0，以解决不确定性
  }

  // 返回欧拉角向量
  Eigen::Matrix<double, 3, 1> ang(x, y, z);
  return ang;
}

/**
 * @brief 打开日志文件用于调试和分析
 * @details 该函数打开两个关键的日志文件：
 *          1. mat_out.txt - 记录系统状态输出 (位姿、速度、偏置等)
 *          2. imu_pbp.txt - 记录IMU点对点处理过程
 *          这些文件对于系统调试和性能分析非常重要
 */
void open_file()
{
  // 打开状态输出日志文件 (包含位置、姿态、速度等信息)
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);

  // 打开IMU点对点处理日志文件 (记录高频处理过程)
  fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);

  // 检查文件打开状态并输出提示信息
  if (fout_out && fout_imu_pbp)
    std::cout << "~~~~ 日志文件打开成功: " << ROOT_DIR << std::endl;
  else
    std::cout << "~~~~ 错误: 日志目录不存在: " << ROOT_DIR << std::endl;
}

/**
 * @brief 重置输入模式滤波器的初始协方差矩阵 (24x24)
 * @param P_init 要重置的协方差矩阵引用
 * @details 该函数为use_imu_as_input=true模式设置初始不确定性：
 *          状态向量结构 (24维)：
 *          [0:2]   位置     - 0.1   (较大不确定性)
 *          [3:5]   姿态     - 0.1   (较大不确定性)
 *          [6:8]   外参旋转 - 0.1   (较大不确定性)
 *          [9:11]  外参平移 - 0.1   (较大不确定性)
 *          [12:14] 速度     - 0.1   (较大不确定性)
 *          [15:17] 陀螺偏置 - 0.001 (中等不确定性)
 *          [18:20] 加速偏置 - 0.001 (中等不确定性)
 *          [21:23] 重力     - 0.0001(小不确定性)
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
  // 整体初始化为 0.1 * 单位矩阵 (表示较大的初始不确定性)
  P_init = MD(24, 24)::Identity() * 0.1;

  // 重力向量的不确定性较小 (通常可以精确测量)
  P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;

  // IMU偏置的不确定性中等 (需要时间估计但相对稳定)
  P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}

/**
 * @brief 重置输出模式滤注器的初始协方差矩阵 (30x30)
 * @param P_init_output 要重置的协方差矩阵引用
 * @details 该函数为use_imu_as_input=false模式设置初始不确定性：
 *          状态向量结构 (30维)：
 *          [0:2]   位置       - 0.01   (相对小的不确定性)
 *          [3:5]   姿态       - 0.01   (相对小的不确定性)
 *          [6:8]   速度       - 0.01   (相对小的不确定性)
 *          [9:11]  陀螺偏置   - 0.01   (相对小的不确定性)
 *          [12:14] 加速偏置   - 0.01   (相对小的不确定性)
 *          [15:17] 重力       - 0.01   (相对小的不确定性)
 *          [18:20] 外参旋转   - 0.01   (相对小的不确定性)
 *          [21:23] 外参平移   - 0.0001 (极小不确定性)
 *          [24:26] 瞬时角速度 - 0.001  (中等不确定性)
 *          [27:29] 瞬时加速度 - 0.001  (中等不确定性)
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
  // 整体初始化为 0.01 * 单位矩阵 (相比输入模式更信任初值)
  P_init_output = MD(30, 30)::Identity() * 0.01;

  // 外参平移的不确定性极小 (通常可以精确测量)
  P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;

  // 瞬时角速度和加速度的不确定性中等
  // (这些状态变量在输出模式中被显式建模)
  P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}

// === 文件结尾 ===
// 该文件提供了Point-LIO系统的完整参数管理功能
// 包括参数读取、滤波器初始化和工具函数
// 详细使用方法请参考README_parameters.md
