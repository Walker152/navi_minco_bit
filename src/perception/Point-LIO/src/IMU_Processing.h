/**
 * @file IMU_Processing.h
 * @brief IMU数据预处理类的头文件
 * @details 该文件定义了IMU数据预处理类，负责IMU初始化、数据处理和LiDAR点云的运动补偿
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

#pragma once
#include <math.h>

#include <cmath>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>     // ROS2日志和节点功能
// #include <so3_math.h>
#include <Eigen/Eigen>           // 线性代数库
// #include "Estimator.h"
#include <common_lib.h>          // 公共库，定义了V3D、M3D等类型
#include <pcl/common/io.h>       // PCL点云输入输出
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>      // ROS2里程计消息类型
#include <sensor_msgs/msg/point_cloud2.hpp>  // ROS2点云消息类型

/// *************预配置参数

/**
 * @def MAX_INI_COUNT
 * @brief IMU初始化的最大迭代次数
 * @details 当累积的IMU帧数达到100帧时，IMU初始化完成
 */
#define MAX_INI_COUNT (100)

/**
 * @brief 点云时间排序比较函数声明
 * @param x 第一个点
 * @param y 第二个点
 * @return true if x的时间戳小于y的时间戳
 * @details 用于对点云按照时间戳进行排序，curvature字段存储时间信息
 */
const bool time_list(PointType & x, PointType & y);  // {return (x.curvature < y.curvature);};

/// *************IMU数据处理和点云去畸变
/**
 * @class ImuProcess
 * @brief IMU数据预处理类
 * @details 该类负责IMU数据的初始化、处理和LiDAR点云的运动补偿
 *          主要功能包括：
 *          1. IMU初始化（陀螺仪偏置估计、重力矢量对齐）
 *          2. IMU数据预积分和状态传播
 *          3. LiDAR点云运动补偿（去除由于运动产生的畸变）
 */
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen内存对齐宏，确保Eigen矩阵正确对齐

  /**
   * @brief 构造函数
   * @details 初始化IMU处理器的基本参数和状态
   */
  ImuProcess();
  
  /**
   * @brief 析构函数
   */
  ~ImuProcess();

  /**
   * @brief 重置IMU处理器状态
   * @details 将所有状态变量重置为初始值，通常在系统重启或失效时调用
   */
  void Reset();
  
  /**
   * @brief 主要的IMU和LiDAR数据处理函数
   * @param meas 包含IMU和LiDAR数据的测量组
   * @param pcl_un_ 输出的去畸变点云
   * @details 该函数是IMU处理的核心，负责：
   *          - 如果需要初始化，则进行IMU初始化
   *          - 对LiDAR点云进行运动补偿
   *          - 更新系统状态
   */
  void Process(const MeasureGroup & meas, PointCloudXYZI::Ptr pcl_un_);
  
  /**
   * @brief 设置陀螺仪协方差缩放因子
   * @param scaler 3D向量，表示x、y、z轴的协方差缩放因子
   */
  void set_gyr_cov(const V3D & scaler);
  
  /**
   * @brief 设置加速度计协方差缩放因子
   * @param scaler 3D向量，表示x、y、z轴的协方差缩放因子
   */
  void set_acc_cov(const V3D & scaler);
  
  /**
   * @brief 设置初始重力方向和旋转矩阵
   * @param tmp_gravity 临时重力向量
   * @param rot 输出的旋转矩阵，用于将IMU坐标系对齐到重力方向
   * @details 该函数计算IMU坐标系到世界坐标系的初始旋转，确保z轴与重力方向对齐
   */
  void Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot);

  // ===== 公共成员变量 =====
  MD(12, 12) state_cov = MD(12, 12)::Identity();  ///< 12x12状态协方差矩阵，用于卡尔曼滤波
  int lidar_type;                                  ///< LiDAR类型标识符
  V3D gravity_;                                    ///< 重力向量（世界坐标系下）
  bool imu_en;                                     ///< IMU使能标志位
  V3D mean_acc;                                    ///< 加速度计均值（用于初始化期间）
  bool imu_need_init_ = true;                      ///< IMU是否需要初始化标志位
  bool after_imu_init_ = false;                    ///< IMU初始化完成后的标志位
  bool b_first_frame_ = true;                      ///< 是否为第一帧数据的标志位
  double time_last_scan = 0.0;                     ///< 上一次扫描的时间戳
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001); ///< 陀螺仪协方差缩放因子
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001); ///< 速度协方差缩放因子

private:
  // ===== 私有成员函数 =====
  /**
   * @brief IMU初始化函数
   * @param meas 包含IMU数据的测量组
   * @param N 当前累积的IMU帧数（输入输出参数）
   * @details 该函数负责：
   *          - 累积IMU测量数据
   *          - 计算加速度计和陀螺仪的均值
   *          - 估计陀螺仪偏置
   *          - 当累积足够数据后完成初始化
   */
  void IMU_init(const MeasureGroup & meas, int & N);
  
  // ===== 私有成员变量 =====
  V3D mean_gyr;              ///< 陀螺仪均值（用于初始化期间的偏置估计）
  int init_iter_num = 1;     ///< 初始化迭代计数器，记录当前处理的IMU帧数
  rclcpp::Logger logger;     ///< ROS2日志器，用于输出调试和状态信息
};
