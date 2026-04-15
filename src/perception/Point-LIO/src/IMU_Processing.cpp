/**
 * @file IMU_Processing.cpp
 * @brief IMU数据预处理类的实现文件
 * @details 该文件实现了IMU数据预处理的核心功能，包括IMU初始化、数据处理和LiDAR点云运动补偿
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

#include "IMU_Processing.h"

/**
 * @brief 点云时间排序比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return true if x的时间戳小于y的时间戳
 * @details 用于对点云按照时间戳进行排序，curvature字段存储时间信息
 */
const bool time_list(PointType & x, PointType & y)
{
  return (x.curvature < y.curvature);
};

/**
 * @brief 设置陀螺仪协方差缩放因子
 * @param scaler 3D向量，表示x、y、z轴的协方差缩放因子
 * @details 该函数用于调整陀螺仪测量噪声的协方差，影响卡尔曼滤波的置信度
 */
void ImuProcess::set_gyr_cov(const V3D & scaler)
{
  cov_gyr_scale = scaler;
}

/**
 * @brief 设置加速度计协方差缩放因子
 * @param scaler 3D向量，表示x、y、z轴的协方差缩放因子
 * @details 该函数用于调整加速度计测量噪声的协方差，影响速度估计的精度
 */
void ImuProcess::set_acc_cov(const V3D & scaler)
{
  cov_vel_scale = scaler;
}

/**
 * @brief 构造函数
 * @details 初始化IMU处理器的所有成员变量
 *          - 设置初始化标志位
 *          - 创建ROS2日志器
 *          - 初始化协方差矩阵
 *          - 设置默认参数值
 */
ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;              // 启用IMU处理
  init_iter_num = 1;          // 初始化迭代计数器
  mean_acc = V3D(0, 0, 0.0);  // 加速度计均值初始化
  mean_gyr = V3D(0, 0, 0);    // 陀螺仪均值初始化
  after_imu_init_ = false;    // IMU初始化完成标志
  state_cov.setIdentity();    // 状态协方差矩阵初始化为单位矩阵
}

/**
 * @brief 析构函数
 * @details 清理资源（当前为空实现）
 */
ImuProcess::~ImuProcess()
{
}

/**
 * @brief 重置IMU处理器状态
 * @details 该函数将所有状态变量重置为初始值，通常在以下情况调用：
 *          - 系统重启时
 *          - IMU失效需要重新初始化时
 *          - 检测到大幅跳跃或异常时
 */
void ImuProcess::Reset()
{
  RCLCPP_WARN(logger, "reset ImuProcess");  // 输出重置警告信息
  mean_acc = V3D(0, 0, 0.0);                // 重置加速度计均值
  mean_gyr = V3D(0, 0, 0);                  // 重置陀螺仪均值
  imu_need_init_ = true;                    // 设置需要重新初始化
  init_iter_num = 1;                        // 重置初始化计数器
  after_imu_init_ = false;                  // 重置初始化完成标志

  time_last_scan = 0.0;  // 重置上次扫描时间
}

/**
 * @brief 设置初始重力方向和旋转矩阵
 * @param tmp_gravity 从IMU初始化得到的临时重力向量
 * @param rot 输出的旋转矩阵，用于将IMU坐标系对齐到重力方向
 * @details 该函数的核心功能：
 *          1. 计算IMU坐标系到世界坐标系的初始旋转
 *          2. 确保z轴与重力方向对齐
 *          3. 使用反对称矩阵和Rodrigues公式计算旋转
 *
 *          算法步骤：
 *          - 构造重力向量的反对称矩阵
 *          - 计算两个重力向量之间的夹角
 *          - 使用轴角表示计算旋转矩阵
 */
void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  /** 1. 初始化重力、陀螺仪偏置、加速度计和陀螺仪协方差
   ** 2. 将加速度测量值归一化为单位重力 **/

  // 构造重力向量的反对称矩阵（用于叉积运算）
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0), gravity_(1), -gravity_(0),
    0.0;

  // 计算对齐向量的模长（叉积的模）
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();

  // 计算两个重力向量的余弦值（点积）
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  // 根据对齐情况计算旋转矩阵
  if (align_norm < 1e-6) {  // 向量几乎平行或反平行
    if (align_cos > 1e-6) {
      rot = Eye3d;  // 同向：单位矩阵
    } else {
      rot = -Eye3d;  // 反向：负单位矩阵
    }
  } else {  // 一般情况：使用Rodrigues公式
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

/**
 * @brief IMU初始化函数
 * @param meas 包含IMU数据的测量组
 * @param N 当前累积的IMU帧数（输入输出参数）
 * @details 该函数是IMU初始化的核心，主要完成以下任务：
 *          1. 累积IMU测量数据（加速度计和陀螺仪）
 *          2. 使用递推平均算法计算加速度计和陀螺仪的均值
 *          3. 估计陀螺仪偏置（通过陀螺仪均值）
 *          4. 估计重力方向（通过加速度计均值）
 *
 *          初始化策略：
 *          - 假设初始化期间系统静止或匀速运动
 *          - 加速度计测量值主要反映重力
 *          - 陀螺仪测量值主要反映偏置
 *
 *          递推平均公式：mean_new = mean_old + (measurement - mean_old) / N
 */
void ImuProcess::IMU_init(const MeasureGroup & meas, int & N)
{
  /** 1. 初始化重力、陀螺仪偏置、加速度计和陀螺仪协方差
   ** 2. 将加速度测量值归一化为单位重力 **/

  // 输出初始化进度
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;  // 当前帧的加速度和角速度

  // 处理第一帧数据
  if (b_first_frame_) {
    Reset();                 // 重置所有状态
    N = 1;                   // 初始化计数器
    b_first_frame_ = false;  // 清除首帧标志

    // 提取第一帧IMU数据
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;

    // 初始化均值为第一帧数据
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  // 处理当前测量组中的所有IMU数据
  for (const auto & imu : meas.imu) {
    // 提取当前IMU数据
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // 使用递推平均算法更新均值
    // 这种方法避免了存储所有历史数据，节省内存且计算高效
    mean_acc += (cur_acc - mean_acc) / N;  // 更新加速度计均值
    mean_gyr += (cur_gyr - mean_gyr) / N;  // 更新陀螺仪均值

    N++;  // 增加样本计数
  }
}

/**
 * @brief 主要的IMU和LiDAR数据处理函数
 * @param meas 包含IMU和LiDAR数据的测量组
 * @param cur_pcl_un_ 输出的去畸变点云
 * @details 该函数是整个IMU处理流程的入口点，根据系统状态执行不同的处理逻辑：
 *
 *          处理流程：
 *          1. 如果IMU使能：
 *             a) 检查IMU数据有效性
 *             b) 如果需要初始化：执行IMU初始化流程
 *             c) 如果初始化完成：执行正常的IMU处理和点云去畸变
 *          2. 如果IMU禁用：直接输出原始点云
 *
 *          初始化阶段：
 *          - 累积IMU数据进行统计分析
 *          - 估计陀螺仪偏置和重力方向
 *          - 当样本数量达到阈值时完成初始化
 *
 *          正常处理阶段：
 *          - 使用IMU数据进行状态预测
 *          - 对LiDAR点云进行运动补偿
 *          - 输出去畸变的点云数据
 */
void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en) {  // IMU处理使能
    // 检查IMU数据有效性
    if (meas.imu.empty())
      return;

    if (imu_need_init_) {  // IMU初始化阶段
      {
        /// 处理第一帧LiDAR数据对应的IMU初始化
        IMU_init(meas, init_iter_num);

        // 注意：这里保持imu_need_init_为true，继续累积数据
        imu_need_init_ = true;

        // 检查是否达到初始化完成条件
        if (init_iter_num > MAX_INI_COUNT) {
          // 初始化完成
          RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;        // 清除初始化标志
          *cur_pcl_un_ = *(meas.lidar);  // 输出当前点云（暂未去畸变）
        }
        // 在初始化阶段，暂时输出原始点云
        // *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }

    // IMU初始化完成后的正常处理阶段
    if (!after_imu_init_)
      after_imu_init_ = true;  // 设置初始化完成标志

    // TODO: 这里应该添加IMU预积分和点云去畸变的具体实现
    // 当前版本只是简单地复制原始点云
    *cur_pcl_un_ = *(meas.lidar);
    return;
  } else {  // IMU处理禁用
    // 直接输出原始点云，不进行任何处理
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}