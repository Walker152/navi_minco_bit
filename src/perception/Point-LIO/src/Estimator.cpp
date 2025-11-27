/**
 * @file Estimator.cpp
 * @brief 状态估计器模块的实现文件
 * @details 该文件实现了LIO系统的核心状态估计功能，包括：
 *          - 基于ESEKF的非线性状态估计
 *          - LiDAR点到平面约束的构建和优化
 *          - IMU预积分和数据融合
 *          - 多传感器时空对齐和外参估计
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

// ======================== 全局变量定义 ========================
// 状态估计器使用的核心数据结构和缓存

/**
 * @brief 法向量点云存储，容量100000个点
 * @details 存储每个特征点拟合平面的法向量(nx,ny,nz)和到平面距离d
 */
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));

/**
 * @brief 时间序列数组，记录每个时间步的点数
 * @details 用于将连续扫描的点云分割成时间窗口进行处理
 */
std::vector<int> time_seq;

/**
 * @brief 机体坐标系下的特征点云，容量10000个点
 * @details 经过特征提取和降采样的LiDAR点云
 */
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));

/**
 * @brief 世界坐标系下的特征点云，容量10000个点
 * @details 通过状态估计变换后的特征点云，用于地图匹配
 */
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));

/**
 * @brief 机体坐标系点位置列表
 * @details 存储原始点云的3D坐标，用于坐标变换计算
 */
std::vector<V3D> pbody_list;

/**
 * @brief 最近邻点搜索结果
 * @details 每个查询点对应的最近邻点集合，用于局部平面拟合
 */
std::vector<PointVector> Nearest_Points;

/**
 * @brief iVox局部地图指针
 * @details 高效的体素化地图结构，支持快速最近邻搜索和动态更新
 */
std::shared_ptr<IVoxType> ivox_ = nullptr;  // localmap in ivox

/**
 * @brief 点搜索平方距离数组
 * @details 存储最近邻搜索的距离结果，用于数据关联质量评估
 */
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

/**
 * @brief 点选择标志数组，最大支持100000个点
 * @details 标记哪些点通过了几何约束检验，被选为有效特征点
 */
bool point_selected_surf[100000] = {0};

/**
 * @brief 叉积矩阵列表
 * @details 预计算的反对称矩阵，用于高效计算旋转雅可比矩阵
 */
std::vector<M3D> crossmat_list;

/**
 * @brief 当前帧有效特征点计数器
 * @details 累计成功建立几何约束的特征点数量
 */
int effct_feat_num = 0;

/**
 * @brief 时间窗口处理索引
 * @details 用于分批处理不同时间戳的点云数据
 */
int k = 0;

/**
 * @brief 点云处理的当前索引
 * @details 指向当前处理的点在点云数组中的位置
 */
int idx = -1;

/**
 * @brief 输入状态的扩展卡尔曼滤波器
 * @details 24维状态向量的ESEKF实例，用于基本状态估计
 */
esekfom::esekf<state_input, 24, input_ikfom> kf_input;

/**
 * @brief 输出状态的扩展卡尔曼滤波器
 * @details 30维状态向量的ESEKF实例，包含外参估计功能
 */
esekfom::esekf<state_output, 30, input_ikfom> kf_output;

/**
 * @brief IMU输入数据结构
 * @details 包装IMU测量数据的标准化接口
 */
input_ikfom input_in;

/**
 * @brief IMU测量数据的平均值
 * @details angvel_avr: 平均角速度, acc_avr: 平均加速度, acc_avr_norm: 归一化加速度模长
 */
V3D angvel_avr, acc_avr, acc_avr_norm;

/**
 * @brief 降采样后的特征点数量
 */
int feats_down_size = 0;

/**
 * @brief LiDAR相对IMU的平移外参
 * @details 初始化为零向量，可通过外参估计更新
 */
V3D Lidar_T_wrt_IMU(Zero3d);

/**
 * @brief LiDAR相对IMU的旋转外参
 * @details 初始化为单位矩阵，可通过外参估计更新
 */
M3D Lidar_R_wrt_IMU(Eye3d);

/**
 * @brief 标准重力加速度 (m/s²)
 * @details 用于IMU重力补偿和坐标系对齐
 */
double G_m_s2 = 9.81;

/**
 * @brief 构建输入状态的过程噪声协方差矩阵
 * @return 24x24的过程噪声协方差矩阵
 * @details 该函数构建ESEKF预测步骤使用的过程噪声协方差矩阵Q
 *          状态向量结构 (24维)：
 *          [0-2]:   位置 (x, y, z)
 *          [3-5]:   旋转 (roll, pitch, yaw) 
 *          [6-8]:   速度 (vx, vy, vz)
 *          [9-11]:  角速度偏置 (bg_x, bg_y, bg_z)
 *          [12-14]: 加速度偏置 (ba_x, ba_y, ba_z)
 *          [15-17]: 重力向量 (gx, gy, gz)
 *          [18-23]: 其他辅助状态
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
  Eigen::Matrix<double, 24, 24> cov;
  cov.setZero();  // 初始化为零矩阵
  
  // 设置陀螺仪噪声协方差 (旋转相关，索引3-5)
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  
  // 设置加速度计噪声协方差 (加速度相关，索引12-14)
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  
  // 设置陀螺仪偏置噪声协方差 (偏置随机游走，索引15-17)
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  
  // 设置加速度计偏置噪声协方差 (偏置随机游走，索引18-20)
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  
  // 以下为MTK工具包的备用实现方式（已注释）
  // MTK::setDiagonal<process_noise_input, vect3, 0>(cov, &process_noise_input::ng, gyr_cov_input);// 0.03
  // MTK::setDiagonal<process_noise_input, vect3, 3>(cov, &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
  // MTK::setDiagonal<process_noise_input, vect3, 6>(cov, &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
  // MTK::setDiagonal<process_noise_input, vect3, 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
  
  return cov;
}

/**
 * @brief 构建输出状态的过程噪声协方差矩阵
 * @return 30x30的过程噪声协方差矩阵
 * @details 该函数为扩展状态构建过程噪声协方差矩阵
 *          扩展状态向量结构 (30维)：
 *          [0-2]:   位置 (x, y, z)
 *          [3-5]:   旋转 (roll, pitch, yaw)
 *          [6-8]:   速度 (vx, vy, vz)
 *          [9-11]:  外参旋转
 *          [12-14]: 速度协方差 (新增)
 *          [15-17]: 陀螺仪噪声
 *          [18-20]: 加速度计噪声
 *          [21-23]: 重力向量
 *          [24-26]: 陀螺仪偏置
 *          [27-29]: 加速度计偏置
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();  // 初始化为零矩阵
  
  // 设置速度噪声协方差 (索引12-14)
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  
  // 设置陀螺仪输出噪声协方差 (索引15-17)
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  
  // 设置加速度计输出噪声协方差 (索引18-20)
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  
  // 设置陀螺仪偏置噪声协方差 (索引24-26)
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  
  // 设置加速度计偏置噪声协方差 (索引27-29)
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  
  return cov;
}

/**
 * @brief 输入状态的状态转移函数 f(x,u)
 * @param s 当前状态向量
 * @param in IMU输入测量数据
 * @return 24维状态导数向量 dx/dt = f(x,u)
 * @details 实现非线性状态转移方程，包括：
 *          - 位置导数 = 速度
 *          - 姿态导数 = 角速度（去偏置后）
 *          - 速度导数 = 旋转矩阵 * (加速度 - 偏置) + 重力
 *          
 *          状态向量组成：
 *          位置(0-2), 旋转(3-5), 速度(6-8), 角速度偏置(9-11), 
 *          加速度偏置(12-14), 重力(15-17), 其他辅助状态(18-23)
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  
  // 计算去偏置后的角速度
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);  // omega = gyro_meas - bias_gyro
  
  // 计算惯性坐标系下的加速度
  vect3 a_inertial = s.rot * (in.acc - s.ba);  // 旋转到惯性系并去偏置
  
  // 构建状态导数向量
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];                          // 位置导数 = 速度
    res(i + 3) = omega[i];                      // 姿态导数 = 角速度
    res(i + 12) = a_inertial[i] + s.gravity[i]; // 速度导数 = 加速度 + 重力
  }
  return res;
}

/**
 * @brief 输出状态的状态转移函数 f(x,u)
 * @param s 当前扩展状态向量
 * @param in IMU输入测量数据
 * @return 30维状态导数向量 dx/dt = f(x,u)
 * @details 扩展版本的状态转移方程，支持更多状态变量：
 *          - 包含外参估计状态
 *          - 支持在线传感器标定
 *          - 更复杂的噪声模型
 *          
 *          扩展状态向量包含30个维度，涵盖位姿、速度、偏置、
 *          外参、重力等所有需要估计的参数
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
  
  // 计算惯性坐标系下的加速度（使用状态中的加速度）
  vect3 a_inertial = s.rot * s.acc;  // 旋转到惯性坐标系
  
  // 构建扩展状态导数向量
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];                          // 位置导数 = 速度
    res(i + 3) = s.omg[i];                      // 姿态导数 = 角速度
    res(i + 12) = a_inertial[i] + s.gravity[i]; // 速度导数 = 加速度 + 重力
  }
  return res;
}

/**
 * @brief 输入状态对状态的雅可比矩阵 ∂f/∂x
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 24x24雅可比矩阵
 * @details 用于ESEKF预测步骤的线性化，计算状态转移的一阶偏导数
 *          主要的偏导数关系：
 *          - ∂位置/∂速度 = I (单位矩阵)
 *          - ∂速度/∂姿态 = -R * [a]× (加速度的反对称矩阵)
 *          - ∂速度/∂加速度偏置 = -R
 *          - ∂姿态/∂角速度偏置 = -I
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  
  // ∂位置/∂速度 = I (索引 0-2 对 12-14)
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  
  // 提取去偏置后的IMU测量值
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);    // 去偏置加速度
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);  // 去偏置角速度
  
  // ∂速度/∂姿态 = -R * [a]× (索引 12-14 对 3-5)
  // [a]× 表示加速度的反对称矩阵（用于叉积运算）
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acc_);
  
  // ∂速度/∂加速度偏置 = -R (索引 12-14 对 18-20)
  cov.template block<3, 3>(12, 18) = -s.rot;
  
  // ∂速度/∂重力 = I (索引 12-14 对 21-23)
  // 重力直接影响加速度的计算
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  
  // ∂姿态/∂角速度偏置 = -I (索引 3-5 对 15-17)
  // 角速度偏置直接影响姿态更新
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  
  return cov;
}

/**
 * @brief 输出状态对状态的雅可比矩阵 ∂f/∂x
 * @param s 当前状态
 * @param in IMU输入数据
 * @return 30x30雅可比矩阵
 * @details 扩展版本的状态雅可比矩阵，包含更多状态变量的耦合关系
 *          与输入状态的主要区别：
 *          - 支持更大的状态空间 (30维)
 *          - 包含外参估计的雅可比关系
 *          - 使用状态中存储的加速度值
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  
  // ∂位置/∂速度 = I (索引 0-2 对 12-14)
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  
  // ∂速度/∂姿态 = -R * [a]× (索引 12-14 对 3-5)
  // 使用状态中存储的加速度值 s.acc
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(s.acc);
  
  // ∂速度/∂加速度 = R (索引 12-14 对 18-20)
  // 注意：这里是正号，与输入状态不同
  cov.template block<3, 3>(12, 18) = s.rot;
  
  // ∂速度/∂重力 = I (索引 12-14 对 21-23)
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  
  // ∂姿态/∂角速度 = I (索引 3-5 对 15-17)
  // 输出状态中角速度直接作为状态变量
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  
  return cov;
}

void h_model_input(
  state_input & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  bool match_in_map = false;
  VF(4) pabcd;
  pabcd.setZero();
  normvec->resize(time_seq[k]);
  int effect_num_k = 0;
  for (int j = 0; j < time_seq[k]; j++) {
    PointType & point_body_j = feats_down_body->points[idx + j + 1];
    PointType & point_world_j = feats_down_world->points[idx + j + 1];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[idx + j + 1];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto & points_near = Nearest_Points[idx + j + 1];
      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);  //
      if ((points_near.size() <
           NUM_MATCH_POINTS))  // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) // 5)
      {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));
          // V3D norm_vec;
          // M3D Rpf, pf;
          // pf = crossmat_list[idx+j+1];
          // // pf << SKEW_SYM_MATRX(p_body);
          // Rpf = s.rot * pf;
          // norm_vec << pabcd(0), pabcd(1), pabcd(2);
          // double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
          // // if (p_norm > match_s * pd2 * pd2)
          // double epsilon = pd2 / sqrt(noise_state);
          // // std::cout << "check epsilon:" << epsilon << '\n';
          // double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
          // if (epsilon > 1.0)
          // {
          // 	weight = sqrt(2 * epsilon - 1) / epsilon;
          // 	pabcd(0) = weight * pabcd(0);
          // 	pabcd(1) = weight * pabcd(1);
          // 	pabcd(2) = weight * pabcd(2);
          // 	pabcd(3) = weight * pabcd(3);
          // }
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;

  for (int j = 0; j < time_seq[k]; j++) {
    // ekfom_data.converge = false;
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);  // conjugate().normalized()
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;

      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

void h_model_output(
  state_output & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  bool match_in_map = false;
  VF(4) pabcd;
  pabcd.setZero();
  normvec->resize(time_seq[k]);
  int effect_num_k = 0;
  for (int j = 0; j < time_seq[k]; j++) {
    PointType & point_body_j = feats_down_body->points[idx + j + 1];
    PointType & point_world_j = feats_down_world->points[idx + j + 1];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[idx + j + 1];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto & points_near = Nearest_Points[idx + j + 1];

      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);  //

      if ((points_near.size() <
           NUM_MATCH_POINTS))  // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
      {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));
          // V3D norm_vec;
          // M3D Rpf, pf;
          // pf = crossmat_list[idx+j+1];
          // // pf << SKEW_SYM_MATRX(p_body);
          // Rpf = s.rot * pf;
          // norm_vec << pabcd(0), pabcd(1), pabcd(2);
          // double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
          // // if (p_norm > match_s * pd2 * pd2)
          // double epsilon = pd2 / sqrt(noise_state);
          // double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
          // if (epsilon > 1.0)
          // {
          // 	weight = sqrt(2 * epsilon - 1) / epsilon;
          // 	pabcd(0) = weight * pabcd(0);
          // 	pabcd(1) = weight * pabcd(1);
          // 	pabcd(2) = weight * pabcd(2);
          // 	pabcd(3) = weight * pabcd(3);
          // }
          if (p_norm > match_s * pd2 * pd2) {
            // point_selected_surf[i] = true;
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;
  for (int j = 0; j < time_seq[k]; j++) {
    // ekfom_data.converge = false;
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);  // conjugate().normalized()
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;

      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

void h_model_IMU_output(state_output & s, esekfom::dyn_share_modified<double> & ekfom_data)
{
  std::memset(ekfom_data.satu_check, false, 6);
  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov,
    imu_meas_acc_cov, imu_meas_acc_cov;
  if (check_satu) {
    if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;
      ekfom_data.z_IMU(0) = 0.0;
    }

    if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      ekfom_data.z_IMU(1) = 0.0;
    }

    if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      ekfom_data.z_IMU(2) = 0.0;
    }

    if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      ekfom_data.z_IMU(3) = 0.0;
    }

    if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      ekfom_data.z_IMU(4) = 0.0;
    }

    if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      ekfom_data.z_IMU(5) = 0.0;
    }
  }
}

/**
 * @brief 点坐标从机体坐标系转换到世界坐标系
 * @param pi 输入点（机体坐标系，LiDAR坐标系）
 * @param po 输出点（世界坐标系）
 * @details 该函数实现三级坐标变换：
 *          LiDAR坐标系 → IMU坐标系 → 世界坐标系
 *          
 *          变换公式：
 *          p_world = R_world_imu * (R_imu_lidar * p_lidar + t_imu_lidar) + t_world_imu
 *          
 *          支持两种工作模式：
 *          1. 外参在线估计模式：使用状态中的外参 (offset_R_L_I, offset_T_L_I)
 *          2. 固定外参模式：使用预设的外参 (Lidar_R_wrt_IMU, Lidar_T_wrt_IMU)
 *          
 *          根据use_imu_as_input标志选择使用输入状态或输出状态的位姿
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
  // 提取LiDAR坐标系下的点坐标
  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;
  
  if (extrinsic_est_en) {  // 外参在线估计模式
    if (!use_imu_as_input) {  // 使用输出状态
      // p_world = R_world * (R_L_I * p_body + T_L_I) + t_world
      p_global =
        kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) +
        kf_output.x_.pos;
    } else {  // 使用输入状态
      p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) +
                 kf_input.x_.pos;
    }
  } else {  // 固定外参模式
    if (!use_imu_as_input) {  // 使用输出状态的位姿
      // 使用预标定的固定外参进行变换
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_output.x_.pos;
    } else {  // 使用输入状态的位姿
      p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_input.x_.pos;
    }
  }

  // 输出变换后的世界坐标
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;  // 保持强度信息不变
}