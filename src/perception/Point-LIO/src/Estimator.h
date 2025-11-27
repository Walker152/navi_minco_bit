/**
 * @file Estimator.h
 * @brief 状态估计器模块的头文件
 * @details 该文件定义了LIO系统的核心状态估计功能，包括：
 *          - 基于ESEKF（Error State Extended Kalman Filter）的状态估计
 *          - LiDAR点到平面的约束构建
 *          - IMU数据和LiDAR数据的融合
 *          - 系统状态的预测和更新
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

#ifndef Estimator_H
#define Estimator_H

#include "common_lib.h"              // 公共库，定义基本数据类型
#include "parameters.h"              // 参数配置文件
#include <pcl_conversions/pcl_conversions.h>  // PCL-ROS转换
#include <pcl/point_cloud.h>         // PCL点云数据结构
#include <pcl/point_types.h>         // PCL点类型定义
#include <pcl/filters/voxel_grid.h>  // PCL体素网格滤波器
// #include <ikd-Tree/ikd_Tree.h>    // ikd-Tree数据结构（已注释）
#include <pcl/io/pcd_io.h>           // PCL文件输入输出
#include <unordered_set>             // 无序集合容器

// ======================== 全局变量声明 ========================
// 用于状态估计的核心数据结构和缓存变量

/**
 * @brief 法向量点云，存储每个特征点对应的平面法向量和距离
 * @details 格式：(nx, ny, nz, d)，其中d为点到平面的距离
 */
extern PointCloudXYZI::Ptr normvec; //(new PointCloudXYZI(100000, 1));

/**
 * @brief 时间序列数组，记录每个时间步长内的点云数量
 * @details 用于分批处理不同时间戳的点云数据
 */
extern std::vector<int> time_seq;

/**
 * @brief 机体坐标系下的降采样特征点云
 * @details 用于与地图进行匹配的LiDAR特征点
 */
extern PointCloudXYZI::Ptr feats_down_body; //(new PointCloudXYZI());

/**
 * @brief 世界坐标系下的降采样特征点云
 * @details 通过状态估计变换到世界坐标系的特征点
 */
extern PointCloudXYZI::Ptr feats_down_world; //(new PointCloudXYZI());

/**
 * @brief 机体坐标系下的点坐标列表
 * @details 存储原始点云数据的3D坐标
 */
extern std::vector<V3D> pbody_list;

/**
 * @brief 最近邻点搜索结果列表
 * @details 存储每个查询点的最近邻点集合，用于平面拟合
 */
extern std::vector<PointVector> Nearest_Points; 

/**
 * @brief iVox数据结构指针，用于高效的局部地图存储和搜索
 * @details iVox是一种基于体素的快速最近邻搜索数据结构
 */
extern std::shared_ptr<IVoxType> ivox_;                    // localmap in ivox

/**
 * @brief 点搜索的平方距离数组
 * @details 存储最近邻搜索的距离结果，用于质量评估
 */
extern std::vector<float> pointSearchSqDis;

/**
 * @brief 点选择标志数组
 * @details 标记哪些点被选为有效的平面约束特征点
 */
extern bool point_selected_surf[100000]; // = {0};

/**
 * @brief 交叉乘积矩阵列表
 * @details 预计算的反对称矩阵，用于雅可比矩阵计算
 */
extern std::vector<M3D> crossmat_list;

/**
 * @brief 有效特征点数量
 * @details 当前帧中成功建立约束的特征点总数
 */
extern int effct_feat_num;

/**
 * @brief 当前处理的时间步索引
 * @details 用于分批处理点云数据的循环变量
 */
extern int k;

/**
 * @brief 当前处理的点云索引
 * @details 指向当前处理点在点云数组中的位置
 */
extern int idx;

/**
 * @brief IMU平均测量值
 * @details angvel_avr: 平均角速度, acc_avr: 平均加速度, acc_avr_norm: 归一化加速度
 */
extern V3D angvel_avr, acc_avr, acc_avr_norm;

/**
 * @brief 降采样后的特征点数量
 * @details 用于控制处理的点云规模
 */
extern int feats_down_size;

/**
 * @brief LiDAR相对于IMU的平移向量
 * @details 外参标定参数，默认为零向量
 */
extern V3D Lidar_T_wrt_IMU; //(Zero3d);

/**
 * @brief LiDAR相对于IMU的旋转矩阵
 * @details 外参标定参数，默认为单位矩阵
 */
extern M3D Lidar_R_wrt_IMU; //(Eye3d);

/**
 * @brief 重力加速度常数 (m/s²)
 * @details 用于IMU数据处理和重力补偿
 */
extern double G_m_s2;

/**
 * @brief IKFOM输入数据结构
 * @details 包含IMU测量数据的输入结构体
 */
extern input_ikfom input_in;

// ======================== 核心函数声明 ========================
// ESEKF状态估计器的核心功能函数

/**
 * @brief 构建输入状态的过程噪声协方差矩阵
 * @return 24x24的过程噪声协方差矩阵
 * @details 包含陀螺仪、加速度计及其偏置的噪声特性
 *          状态维度：位置(3) + 旋转(3) + 速度(3) + 陀螺仪偏置(3) + 加速度计偏置(3) + 重力(3) + 其他(6)
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input();

/**
 * @brief 构建输出状态的过程噪声协方差矩阵
 * @return 30x30的过程噪声协方差矩阵
 * @details 扩展状态包含额外的传感器参数和外参估计
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output();

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia LiDAR外参
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);

/**
 * @brief 输入状态的状态转移函数 f(x,u)
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 24维状态导数向量
 * @details 实现ESEKF的预测步骤，根据IMU数据更新系统状态
 *          包括位置、速度、姿态的运动学和动力学方程
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in);

/**
 * @brief 输出状态的状态转移函数 f(x,u)
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 30维状态导数向量
 * @details 扩展版本的状态转移函数，包含更多状态变量
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in);

/**
 * @brief 输入状态对状态的雅可比矩阵 ∂f/∂x
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 24x24雅可比矩阵
 * @details 用于ESEKF预测步骤的线性化，计算状态转移的一阶偏导数
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in);

// Eigen::Matrix<double, 24, 12> df_dw_input(state_input &s, const input_ikfom &in);

/**
 * @brief 输出状态对状态的雅可比矩阵 ∂f/∂x
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 30x30雅可比矩阵
 * @details 扩展版本的状态雅可比矩阵
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in);

// Eigen::Matrix<double, 30, 15> df_dw_output(state_output &s);

/**
 * @brief 输入状态的LiDAR观测模型
 * @param s 当前状态
 * @param cov_p 位置协方差
 * @param cov_R 旋转协方差
 * @param ekfom_data ESEKF数据结构，包含观测残差和雅可比矩阵
 * @details 构建点到平面的几何约束，计算观测残差h(x)和观测雅可比∂h/∂x
 *          实现LiDAR点云与局部地图的匹配约束
 */
void h_model_input(state_input &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

/**
 * @brief 输出状态的LiDAR观测模型
 * @param s 当前状态
 * @param cov_p 位置协方差
 * @param cov_R 旋转协方差
 * @param ekfom_data ESEKF数据结构
 * @details 扩展版本的LiDAR观测模型，支持外参在线估计
 */
void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

/**
 * @brief 输出状态的IMU观测模型
 * @param s 当前状态
 * @param ekfom_data ESEKF数据结构
 * @details 构建IMU测量约束，包括角速度和加速度的观测方程
 *          处理传感器饱和检测和异常值处理
 */
void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data);

/**
 * @brief 点坐标从机体坐标系转换到世界坐标系
 * @param pi 输入点（机体坐标系）
 * @param po 输出点（世界坐标系）
 * @details 利用当前状态估计进行坐标变换：
 *          p_world = R * (R_lidar * p_body + t_lidar) + t_world
 *          支持外参在线估计和固定外参两种模式
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po);

#endif