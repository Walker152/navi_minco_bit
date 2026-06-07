/**
 * @file li_initialization.h
 * @brief LiDAR-IMU初始化模块的头文件
 * @details 该文件定义了Point-LIO系统的数据同步和初始化功能，包括：
 *          - 多传感器数据缓冲和同步
 *          - LiDAR和IMU数据的时间对齐
 *          - 传感器数据预处理和质量检查
 *          - 系统初始化状态管理
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

#pragma once

#include <common_lib.h>        // 公共库，包含基本数据类型和工具函数
#include "Estimator.h"         // 状态估计器模块

/**
 * @def MAXN
 * @brief 最大数组大小定义
 * @details 用于性能统计数组的最大容量，支持长时间运行的性能监控
 */
#define MAXN (720000)

// ======================== 初始化状态管理变量 ========================

/**
 * @brief 数据累积和在线标定状态标志
 * @details data_accum_finished: 数据累积完成标志
 *          data_accum_start: 数据累积开始标志  
 *          online_calib_finish: 在线标定完成标志
 *          refine_print: 精化打印输出标志
 */
extern bool data_accum_finished, data_accum_start, online_calib_finish, refine_print;

/**
 * @brief 初始化帧数计数器
 * @details 记录用于初始化的累积帧数
 */
extern int frame_num_init;

/**
 * @brief 时间相关参数
 * @details time_lag_IMU_wtr_lidar: IMU相对LiDAR的时间延迟
 *          move_start_time: 运动开始时间
 *          online_calib_starts_time: 在线标定开始时间
 */
extern double time_lag_IMU_wtr_lidar, move_start_time, online_calib_starts_time;

// ======================== 传感器时间同步变量 ========================

/**
 * @brief IMU相对LiDAR的时间差
 * @details 用于传感器硬件时间同步，单位：秒
 */
extern double timediff_imu_wrt_lidar;

/**
 * @brief 时间差设置完成标志
 * @details 标记是否已完成IMU和LiDAR的时间差计算
 */
extern bool timediff_set_flg;

/**
 * @brief LIO系统估计的重力向量
 * @details 在初始化过程中估计得到的重力方向，用于坐标系对齐
 */
extern V3D gravity_lio;

// ======================== 多线程同步变量 ========================

/**
 * @brief 数据缓冲区互斥锁
 * @details 保护LiDAR和IMU数据缓冲区的线程安全访问
 */
extern mutex mtx_buffer;

/**
 * @brief 缓冲区条件变量
 * @details 用于线程间数据同步的条件变量
 */
extern condition_variable sig_buffer;

// ======================== 数据计数和状态变量 ========================

/**
 * @brief 扫描帧计数器
 * @details 累计接收到的LiDAR扫描帧数量
 */
extern int scan_count;

/**
 * @brief 帧处理计数器和等待计数器
 * @details frame_ct: 当前处理的帧计数
 *          wait_num: 等待处理的帧数量
 */
extern int frame_ct, wait_num;

// ======================== 数据缓冲区 ========================

/**
 * @brief LiDAR点云数据缓冲队列
 * @details 存储待处理的LiDAR点云数据，支持时间序列处理
 */
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;

/**
 * @brief 时间戳缓冲队列
 * @details 与LiDAR点云数据对应的时间戳序列
 */
extern std::deque<double> time_buffer;

/**
 * @brief IMU数据缓冲队列
 * @details 存储待处理的IMU测量数据
 */
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

/**
 * @brief 时间处理互斥锁
 * @details 保护时间相关操作的线程安全
 */
extern std::mutex m_time;

// ======================== 数据同步状态标志 ========================

/**
 * @brief 数据推送状态标志
 * @details lidar_pushed: LiDAR数据已推送标志
 *          imu_pushed: IMU数据已推送标志
 */
extern bool lidar_pushed, imu_pushed;

/**
 * @brief IMU首帧时间戳
 * @details 记录第一帧IMU数据的时间戳，用于时间基准
 */
extern double imu_first_time;

/**
 * @brief LiDAR数据丢失标志
 * @details 标记当前是否丢失了LiDAR数据
 */
extern bool lose_lid;

// ======================== IMU数据缓存 ========================

/**
 * @brief IMU数据缓存变量
 * @details imu_last: 上一帧IMU数据
 *          imu_next: 下一帧IMU数据
 */
extern sensor_msgs::msg::Imu imu_last, imu_next;

/**
 * @brief 连续帧点云数据指针
 * @details 用于存储多帧连续的点云数据
 */
extern PointCloudXYZI::Ptr ptr_con;

// ======================== 性能统计数组 ========================

/**
 * @brief 性能统计数组
 * @details T1: 时间统计数组1
 *          s_plot: 绘图数据数组1  
 *          s_plot2: 绘图数据数组2
 *          s_plot3: 绘图数据数组3
 *          s_plot11: 预处理时间统计数组
 */
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

// ======================== 核心函数声明 ========================

/**
 * @brief 标准点云数据回调函数
 * @param msg ROS2点云消息（PointCloud2格式）
 * @details 处理标准格式的LiDAR点云数据，包括：
 *          - 时间戳验证和回环检测
 *          - 数据预处理和格式转换
 *          - 帧切分和缓冲区管理
 *          - 连续帧合并处理（如果使能）
 *          支持的LiDAR类型：Velodyne、Ouster、HESAI等
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);
void standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr msg);

/**
 * @brief Livox点云数据回调函数  
 * @param msg Livox自定义消息格式
 * @details 专门处理Livox LiDAR的自定义数据格式，包括：
 *          - Livox特有的时间戳处理
 *          - 自定义消息格式解析
 *          - 帧切分和时间校准
 *          - 与标准点云流程的统一处理
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg);
void livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr msg);

/**
 * @brief IMU数据回调函数
 * @param msg_in ROS2 IMU消息
 * @details 处理IMU测量数据，包括：
 *          - 时间戳校正和同步处理
 *          - 数据质量检查和回环检测
 *          - IMU数据缓冲区管理
 *          - 与LiDAR的时间对齐
 */
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in);

/**
 * @brief 多传感器数据包同步函数
 * @param meas 输出的同步测量数据组
 * @return true 同步成功，false 同步失败
 * @details 核心的数据同步算法，实现：
 *          - LiDAR和IMU数据的时间对齐
 *          - 数据包的完整性检查
 *          - 缓冲区的智能管理
 *          - 数据丢失的处理策略
 *          
 *          同步策略：
 *          1. 以LiDAR扫描为时间基准
 *          2. 收集对应时间段内的所有IMU数据
 *          3. 处理传感器时间偏差和延迟
 *          4. 支持IMU禁用模式的纯LiDAR处理
 */
bool sync_packages(MeasureGroup & meas);
