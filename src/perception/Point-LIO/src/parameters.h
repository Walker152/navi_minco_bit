/**
 * @file parameters.h
 * @brief Point-LIO系统全局参数声明文件
 * @details 该文件声明了Point-LIO系统中所有的全局参数变量，包括：
 *          - 传感器配置参数（IMU、激光雷达）
 *          - 算法控制参数（滤波器模式、外参估计等）
 *          - 噪声模型参数（协方差矩阵）
 *          - 几何参数（外参、重力向量等）
 *          - 发布控制参数（话题使能、频率控制等）
 * @author Point-LIO Team
 */

#pragma once

// === 系统和数学库 ===
#include <Python.h>                    // Python接口支持
#include <ivox/ivox3d.h>               // iVox增量式体素地图库
#include <math.h>                      // 数学函数库
#include <omp.h>                       // OpenMP并行计算库
#include <pcl/common/transforms.h>     // PCL变换库
#include <unistd.h>                    // Unix标准函数库

// === Eigen线性代数库 ===
#include <Eigen/Core>                  // Eigen核心功能
#include <Eigen/Eigen>                 // Eigen完整功能

// === C++标准库 ===
#include <condition_variable>          // 条件变量
#include <csignal>                     // 信号处理
#include <cstring>                     // 字符串处理
#include <fstream>                     // 文件流
#include <mutex>                       // 互斥锁
#include <thread>                      // 线程库

// === ROS2消息类型 ===
#include <rclcpp/rclcpp.hpp>           // ROS2核心功能
#include <geometry_msgs/msg/vector3.hpp>       // 3D向量消息
#include <livox_ros_driver2/msg/custom_msg.hpp> // Livox自定义消息
#include <sensor_msgs/msg/imu.hpp>             // IMU消息类型
#include <sensor_msgs/msg/nav_sat_fix.hpp>     // GPS消息类型
#include <sensor_msgs/msg/point_cloud2.hpp>    // 点云消息类型

// === Point-LIO模块头文件 ===
#include "IMU_Processing.h"            // IMU数据处理模块
#include "preprocess.h"                // 点云预处理模块

// === iVox地图类型定义 ===
// 根据编译选项选择不同的iVox节点类型

// #define IVOX_NODE_TYPE_PHC  // 取消注释以使用PHC节点类型（更高性能）

#ifdef IVOX_NODE_TYPE_PHC
// PHC (Perfect Hash Clustering) 节点类型 - 适用于高性能需求
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
// 默认节点类型 - 平衡性能和内存使用
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

// === 时间管理变量 ===
extern bool is_first_frame;              // 标识是否为第一帧数据，用于系统初始化
extern double lidar_end_time;             // 当前激光雷达帧的结束时间戳
extern double first_lidar_time;           // 第一帧激光雷达的时间戳，作为时间基准
extern double time_con;                   // 时间连续性检查变量
extern double last_timestamp_lidar;       // 上一帧激光雷达的时间戳，用于时间差计算
extern double last_timestamp_imu;         // 上一帧IMU数据的时间戳，用于时间差计算

// === 文件管理变量 ===
extern int pcd_index;                     // PCD文件保存的索引编号
// === iVox地图配置变量 ===
extern IVoxType::Options ivox_options_;   // iVox增量式地图的配置选项
extern int ivox_nearby_type;              // iVox邻近点搜索类型 (0:CENTER, 6:NEARBY6, 18:NEARBY18, 26:NEARBY26)

// === 状态向量变量 ===
extern state_input state_in;              // 输入模式状态向量 (24维) - IMU作为输入时使用
extern state_output state_out;            // 输出模式状态向量 (30维) - IMU作为观测时使用

// === 传感器话题配置 ===
extern std::string lid_topic;             // 激光雷达数据话题名称 (默认: "/livox/lidar")
extern std::string imu_topic;              // IMU数据话题名称 (默认: "/livox/imu")
// === 算法核心控制参数 ===
extern bool prop_at_freq_of_imu;          // 是否以IMU频率进行状态传播 (true: 高频传播, false: 点云频率传播)
extern bool check_satu;                   // 是否检查IMU数据饱和 (防止异常数据影响估计)
extern bool con_frame;                    // 是否连接多帧点云 (用于低频率激光雷达)
extern bool cut_frame;                    // 是否切分单帧点云 (用于高频率输出)
extern bool use_imu_as_input;             // IMU数据使用模式 (true: 作为控制输入, false: 作为观测)
extern bool space_down_sample;            // 是否进行空间降采样 (减少计算量)
extern bool extrinsic_est_en;             // 是否启用外参在线估计 (自动标定激光雷达-IMU外参)
extern bool publish_odometry_without_downsample; // 是否发布未降采样的高频里程计
extern bool print_cloud_input_fps;        // 是否打印点云接收、里程计发布和位姿更新频率
extern bool debug_pose_update_detail;     // 是否打印位姿更新细粒度诊断
// === 地图初始化参数 ===
extern int init_map_size;                 // 初始化地图所需的最少特征点数量
extern int con_frame_num;                 // 连接帧的数量 (当con_frame=true时)

// === 几何匹配参数 ===
extern double match_s;                    // 点云匹配的搜索半径参数
extern float plane_thr;                   // 平面特征提取阈值 (越小要求越平)
extern double filter_size_surf_min;       // 表面特征点的降采样尺寸 (米)
extern double filter_size_map_min;        // 地图点的降采样尺寸 (米)
extern double fov_deg;                    // 激光雷达视场角 (度)
extern float DET_RANGE;                   // 激光雷达有效检测范围 (米)

// === IMU参数 ===
extern double satu_acc;                   // 加速度计饱和值 (g)
extern double satu_gyro;                  // 陀螺仪饱和值 (rad/s)
extern double cut_frame_time_interval;    // 帧切分时间间隔 (秒)
extern double cloud_input_fps_print_period; // 运行频率打印周期 (秒)
extern double debug_pose_update_detail_period; // 位姿更新细粒度诊断打印周期 (秒)
// === IMU传感器配置 ===
extern bool imu_en;                       // IMU使能标志 (false时仅使用激光雷达)
extern double imu_time_inte;              // IMU数据积分时间间隔 (秒，通常为1/IMU频率)
extern double acc_norm;                   // 加速度计单位 (1.0: g为单位, 9.81: m/s²为单位)

// === 噪声模型参数 ===
// 观测噪声
extern double laser_point_cov;            // 激光雷达点云观测噪声协方差
extern double imu_meas_acc_cov;           // IMU加速度计测量噪声协方差
extern double imu_meas_omg_cov;           // IMU陀螺仪测量噪声协方差

// 过程噪声 - 输入模式 (use_imu_as_input=true)
extern double acc_cov_input;              // 加速度过程噪声协方差 (输入模式)
extern double gyr_cov_input;              // 角速度过程噪声协方差 (输入模式)
extern double vel_cov;                    // 速度过程噪声协方差

// 过程噪声 - 输出模式 (use_imu_as_input=false)
extern double gyr_cov_output;             // 角速度过程噪声协方差 (输出模式)
extern double acc_cov_output;             // 加速度过程噪声协方差 (输出模式)

// 偏置随机游走噪声
extern double b_gyr_cov;                  // 陀螺仪偏置随机游走噪声协方差
extern double b_acc_cov;                  // 加速度计偏置随机游走噪声协方差
// === 传感器类型配置 ===
extern int lidar_type;                    // 激光雷达类型 (1:Livox, 2:Velodyne, 3:Ouster等)
extern int pcd_save_interval;             // PCD文件保存间隔 (帧数，-1表示保存所有帧)

// === 重力参数 ===
extern std::vector<double> gravity_init;  // 初始重力向量 (IMU本体坐标系下)
extern std::vector<double> gravity;       // 当前估计的重力向量 (世界坐标系下)

// === 日志和调试控制 ===
extern bool runtime_pos_log;              // 是否启用运行时位姿日志记录
extern bool pcd_save_en;                  // 是否启用PCD点云文件保存
extern double accumulated_map_publish_hz; // 累积地图发布频率，<=0 时不发布

// === 发布控制参数 ===
extern bool path_en;                      // 是否发布机器人轨迹路径
extern bool scan_pub_en;                  // 是否发布处理后的点云数据
extern bool scan_body_pub_en;             // 是否发布机体坐标系下的点云
extern bool tf_send_en;                   // 是否发布TF坐标变换
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern bool is_first_frame;

// === 外参标定参数 ===
extern std::vector<double> extrinT;       // 激光雷达到IMU的平移外参 [tx, ty, tz] (米)
extern std::vector<double> extrinR;       // 激光雷达到IMU的旋转外参 [r11,r12,r13,r21,r22,r23,r31,r32,r33]

// === 时间同步参数 ===
extern double time_diff_lidar_to_imu;     // 激光雷达到IMU的时间偏移 (秒)
extern double lidar_time_inte;            // 激光雷达数据积分时间 (秒)
extern double first_imu_time;             // 第一个IMU数据的时间戳

// === 帧处理参数 ===
extern int cut_frame_num;                 // 帧切分数量
extern int orig_odom_freq;                // 原始里程计频率 (Hz)
extern double online_refine_time;         // 在线优化时间 (秒)
extern bool cut_frame_init;               // 帧切分初始化标志

// === 滤波器时间状态变量 ===
extern double time_update_last;           // 上次滤波器更新时间
extern double time_current;               // 当前处理时间
extern double time_predict_last_const;    // 上次预测时间常量
extern double t_last;                     // 上一个时间点

// === 先验地图配置 ===
extern bool enable_prior_pcd;             // 是否启用先验PCD地图
extern string prior_pcd_map_path;         // 先验PCD地图文件路径
extern std::vector<double> init_pose;     // 初始位姿 [x, y, z, qx, qy, qz, qw]

// === 数据结构 ===
extern MeasureGroup Measures;             // 传感器数据测量组 (包含激光雷达和IMU数据)

// === 日志文件流 ===
extern ofstream fout_out;                 // 状态输出日志文件流
extern ofstream fout_imu_pbp;             // IMU点对点处理日志文件流
// === 函数声明 ===

/**
 * @brief 从ROS2参数服务器读取所有系统参数
 * @param n ROS2节点的共享指针
 * @details 该函数负责声明和读取所有ROS2参数，包括传感器配置、算法参数、噪声模型等
 */
void readParameters(rclcpp::Node & n);

/**
 * @brief 打开日志文件用于调试和分析
 * @details 打开两个日志文件：mat_out.txt(状态输出) 和 imu_pbp.txt(IMU点对点处理)
 */
void open_file();

/**
 * @brief 将SO3旋转矩阵转换为欧拉角 (ZYX顺序)
 * @param orient SO3旋转矩阵
 * @return 欧拉角向量 [roll, pitch, yaw] (弧度)
 * @details 使用ZYX欧拉角约定，并处理万向锁奇异情况
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 & orient);

/**
 * @brief 重置输入模式滤波器的初始协方差矩阵 (24x24)
 * @param P_init 要重置的协方差矩阵引用
 * @details 为use_imu_as_input=true模式设置合适的初始不确定性
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init);

/**
 * @brief 重置输出模式滤波器的初始协方差矩阵 (30x30)
 * @param P_init_output 要重置的协方差矩阵引用  
 * @details 为use_imu_as_input=false模式设置合适的初始不确定性
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);
