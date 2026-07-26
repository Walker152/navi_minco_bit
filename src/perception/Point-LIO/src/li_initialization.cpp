/**
 * @file li_initialization.cpp
 * @brief LiDAR-IMU初始化模块的实现文件
 * @details 该文件实现了Point-LIO系统的数据同步和初始化功能，包括：
 *          - 多传感器数据缓冲和时间同步
 *          - LiDAR点云数据的预处理和帧管理
 *          - IMU数据的缓冲和质量检查
 *          - 传感器数据包的智能同步算法
 * @author Point-LIO团队
 * @date 2025年10月1日
 */

#include "li_initialization.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "runtime_statistics.h"

// ======================== 全局变量定义 ========================
// 系统初始化和状态管理相关变量

/**
 * @brief 系统初始化状态标志
 * @details 控制数据累积、在线标定和输出显示的状态标志
 */
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false,
     refine_print = false;

/**
 * @brief 初始化帧数计数器
 * @details 记录已处理的初始化帧数量
 */
int frame_num_init = 0;

/**
 * @brief 时间管理参数
 * @details 管理传感器间的时间延迟和同步参数
 */
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0,
       online_calib_starts_time = 0.0;  // mean_acc_norm = 9.81 (已注释)

/**
 * @brief IMU首帧时间戳
 * @details 记录第一帧IMU数据的时间，用于时间基准建立
 */
double imu_first_time = 0.0;

/**
 * @brief LiDAR数据丢失标志
 * @details 标记当前处理周期是否丢失了LiDAR数据
 */
bool lose_lid = false;

/**
 * @brief 传感器时间差参数
 * @details IMU相对LiDAR的硬件时间差，用于时间同步
 */
double timediff_imu_wrt_lidar = 0.0;

/**
 * @brief 时间差设置完成标志
 * @details 标记是否已完成时间差的自动检测和设置
 */
bool timediff_set_flg = false;

/**
 * @brief LIO系统重力向量估计
 * @details 初始化为零向量，在初始化过程中会被更新
 */
V3D gravity_lio = V3D::Zero();

/**
 * @brief 数据缓冲区互斥锁
 * @details 保护多线程环境下的数据缓冲区访问
 */
mutex mtx_buffer;

/**
 * @brief IMU数据缓存
 * @details 用于数据同步过程中的IMU数据缓存
 */
sensor_msgs::msg::Imu imu_last, imu_next;

/**
 * @brief 连续帧点云数据容器
 * @details 用于存储多帧连续的点云数据进行合并处理
 */
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

// ======================== 线程同步和数据缓冲相关变量 ========================

/**
 * @brief 条件变量，用于线程间同步
 * @details 协调数据生产者和消费者线程的同步
 */
condition_variable sig_buffer;

/**
 * @brief 扫描帧计数器
 * @details 累计接收到的LiDAR扫描帧总数
 */
int scan_count = 0;

/**
 * @brief 帧处理和等待计数器
 * @details frame_ct: 连续帧处理计数, wait_num: 等待处理帧数
 */
int frame_ct = 0, wait_num = 0;

/**
 * @brief 时间处理互斥锁
 * @details 保护时间相关操作的线程安全
 */
std::mutex m_time;

/**
 * @brief 数据推送状态标志
 * @details 标记LiDAR和IMU数据是否已推送到处理队列
 */
bool lidar_pushed = false, imu_pushed = false;

/**
 * @brief 传感器数据缓冲队列
 * @details lidar_buffer: LiDAR点云数据队列
 *          time_buffer: 对应的时间戳队列
 *          imu_deque: IMU数据队列
 */
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

namespace {
std::deque<PointCloudXYZI::Ptr> pending_lidar_buffer;
std::deque<double> pending_time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> pending_imu_deque;
uint64_t imu_log_sequence = 0;
std::chrono::steady_clock::time_point last_imu_arrival_time;
bool has_last_imu_arrival_time = false;

void flush_pending_sensor_data_locked()
{
  while (!pending_lidar_buffer.empty() && !pending_time_buffer.empty()) {
    lidar_buffer.emplace_back(std::move(pending_lidar_buffer.front()));
    pending_lidar_buffer.pop_front();
    time_buffer.emplace_back(pending_time_buffer.front());
    pending_time_buffer.pop_front();
  }

  while (!pending_imu_deque.empty()) {
    imu_deque.emplace_back(std::move(pending_imu_deque.front()));
    pending_imu_deque.pop_front();
  }
}
}  // namespace

/**
 * @brief 标准点云数据回调函数
 * @param msg ROS2标准点云消息（sensor_msgs::msg::PointCloud2）
 * @details 处理标准格式LiDAR点云数据的核心函数，支持多种LiDAR类型：
 *          - Velodyne系列（VLP-16等）
 *          - Ouster系列（OS1-64等）
 *          - HESAI系列（PandarXT-32等）
 *
 *          主要处理流程：
 *          1. 时间戳验证和回环检测
 *          2. 数据预处理和格式转换
 *          3. 可选的帧切分处理
 *          4. 连续帧合并（如果使能）
 *          5. 缓冲区管理和性能统计
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);

  // 扫描帧计数递增
  scan_count++;

  const bool statistics_enabled = point_lio::RuntimeStatistics::instance().enabled();
  const double preprocess_start_time = statistics_enabled ? omp_get_wtime() : 0.0;

  // 时间戳回环检测：检查是否出现时间倒退现象
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;  // 发现时间回环，直接返回
  }

  // 更新LiDAR最新时间戳
  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  // 自动时间同步逻辑（已注释，可选功能）
  // 当IMU和LiDAR时间差过大时，自动计算硬件时间延迟
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  // 根据LiDAR类型和配置选择不同的处理模式
  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
    // ===== 帧切分模式 =====
    // 适用于高频率LiDAR，将单帧切分为多个子帧以提高时间分辨率

    deque<PointCloudXYZI::Ptr> ptr;  // 切分后的点云队列
    deque<double> timestamp_lidar;   // 对应的时间戳队列

    // 调用预处理器进行帧切分
    p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    // 将切分后的子帧逐个加入缓冲区
    while (!ptr.empty() && !timestamp_lidar.empty()) {
      pending_lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      pending_time_buffer.push_back(timestamp_lidar.front() / double(1000));  // 转换为秒单位
      timestamp_lidar.pop_front();
    }
  } else {
    // ===== 标准处理模式 =====

    // 创建点云容器，预分配20000个点的空间
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));

    // 调用预处理器进行标准点云处理
    p_pre->process(msg, ptr);

    if (con_frame) {
      // ===== 连续帧合并模式 =====
      // 将多帧点云合并为一个更大的点云，提高特征密度

      if (frame_ct == 0) {
        // 记录连续帧序列的起始时间
        time_con = last_timestamp_lidar;
      }

      if (frame_ct < 10) {
        // 累积前10帧数据
        for (int i = 0; i < ptr->size(); i++) {
          // 为每个点添加相对时间偏移（存储在curvature字段）
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        // 达到帧数上限，输出合并后的点云
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        *ptr_con_i = *ptr_con;                      // 复制合并后的点云
        pending_lidar_buffer.push_back(ptr_con_i);  // 加入处理队列
        double time_con_i = time_con;
        pending_time_buffer.push_back(time_con_i);  // 使用起始时间作为时间戳
        ptr_con->clear();                           // 清空容器，准备下一轮
        frame_ct = 0;                               // 重置帧计数
      }
    } else {
      // ===== 单帧处理模式 =====
      // 直接处理单帧点云数据

      if (!ptr->points.empty()) {
        pending_lidar_buffer.emplace_back(ptr);  // 加入点云缓冲区
        pending_time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());  // 加入时间缓冲区
      }
    }
  }

  if (statistics_enabled) {
    const double preprocess_ms = (omp_get_wtime() - preprocess_start_time) * 1000.0;
    point_lio::RuntimeStatistics::instance().recordLidarCallback(
      last_timestamp_lidar,
      preprocess_ms,
      pending_lidar_buffer.size(),
      lidar_buffer.size());
  }

  sig_buffer.notify_all();
}

void standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  sensor_msgs::msg::PointCloud2::SharedPtr shared_msg(std::move(msg));
  standard_pcl_cbk(shared_msg);
}

/**
 * @brief Livox点云数据回调函数
 * @param msg Livox自定义消息格式（livox_ros_driver2::msg::CustomMsg）
 * @details 专门处理Livox LiDAR的自定义数据格式，Livox LiDAR特点：
 *          - 非重复扫描模式，每次扫描模式不同
 *          - 自定义的点云数据格式
 *          - 更高的点云密度和精度
 *          - 需要特殊的时间戳处理
 *
 *          处理流程与标准点云类似，但适配Livox特有的数据结构
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);

  const bool statistics_enabled = point_lio::RuntimeStatistics::instance().enabled();
  const double preprocess_start_time = statistics_enabled ? omp_get_wtime() : 0.0;

  // 扫描帧计数递增
  scan_count++;

  // Livox点云时间戳回环检测
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  // 更新Livox LiDAR最新时间戳
  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  if (cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    while (!ptr.empty() && !timestamp_lidar.empty()) {
      pending_lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      pending_time_buffer.push_back(timestamp_lidar.front() / double(1000));  // unit:s
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  // msg->header.stamp.toSec();
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        // std::cout << "ptr div num:" << ptr_div->size() << '\n';
        *ptr_con_i = *ptr_con;
        double time_con_i = time_con;
        pending_lidar_buffer.push_back(ptr_con_i);
        pending_time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        pending_lidar_buffer.emplace_back(ptr);
        pending_time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  if (statistics_enabled) {
    const double preprocess_ms = (omp_get_wtime() - preprocess_start_time) * 1000.0;
    point_lio::RuntimeStatistics::instance().recordLidarCallback(
      last_timestamp_lidar,
      preprocess_ms,
      pending_lidar_buffer.size(),
      lidar_buffer.size());
  }
  sig_buffer.notify_all();
}

void livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
  livox_ros_driver2::msg::CustomMsg::SharedPtr shared_msg(std::move(msg));
  livox_pcl_cbk(shared_msg);
}

/**
 * @brief IMU数据回调函数
 * @param msg_in ROS2 IMU消息常量共享指针
 * @details 处理IMU测量数据的核心函数，主要功能：
 *          1. IMU数据的时间戳校正和同步
 *          2. 硬件时间延迟补偿
 *          3. 数据质量检查和异常处理
 *          4. IMU数据缓冲区管理
 *
 *          时间校正策略：
 *          - 减去IMU相对LiDAR的硬件时间差
 *          - 减去系统配置的时间延迟参数
 *          - 确保时间戳的单调递增性
 */
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);

  // 创建IMU消息的可修改副本，用于时间戳校正
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

  // 发布计数器（已注释）
  // publish_count ++;

  // ===== IMU时间戳校正 =====
  // 应用硬件时间差和系统时间延迟补偿
  // 校正公式：corrected_time = original_time - hardware_delay - system_delay
  msg->header.stamp =
    get_ros_time(get_time_sec(msg_in->header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

  // 获取校正后的时间戳
  double timestamp = get_time_sec(msg->header.stamp);
  if (point_lio::RuntimeStatistics::instance().enabled()) {
    const double original_timestamp = get_time_sec(msg_in->header.stamp);
    const double sensor_dt = last_timestamp_imu >= 0.0 ? timestamp - last_timestamp_imu : 0.0;
    const auto arrival_time = std::chrono::steady_clock::now();
    const double arrival_dt =
      has_last_imu_arrival_time
        ? std::chrono::duration<double>(arrival_time - last_imu_arrival_time).count()
        : 0.0;

    int status = 0;
    int estimated_missing = 0;
    if (last_timestamp_imu >= 0.0) {
      if (sensor_dt < 0.0) {
        status = 3;
      } else if (sensor_dt == 0.0) {
        status = 2;
      } else if (imu_time_inte > 0.0 && sensor_dt > 1.5 * imu_time_inte) {
        status = 1;
        estimated_missing =
          std::max(0, static_cast<int>(std::llround(sensor_dt / imu_time_inte)) - 1);
      }
    }

    point_lio::ImuLogRecord imu_record;
    imu_record.sequence = imu_log_sequence++;
    imu_record.original_stamp = original_timestamp;
    imu_record.corrected_stamp = timestamp;
    imu_record.sensor_dt = sensor_dt;
    imu_record.arrival_dt = arrival_dt;
    imu_record.estimated_missing = estimated_missing;
    imu_record.status = status;
    imu_record.gyro = {
      msg_in->angular_velocity.x,
      msg_in->angular_velocity.y,
      msg_in->angular_velocity.z};
    imu_record.acc = {
      msg_in->linear_acceleration.x,
      msg_in->linear_acceleration.y,
      msg_in->linear_acceleration.z};
    imu_record.pending_queue_size =
      pending_imu_deque.size() + static_cast<std::size_t>(status != 3);
    point_lio::RuntimeStatistics::instance().recordImu(imu_record);
    last_imu_arrival_time = arrival_time;
    has_last_imu_arrival_time = true;
  }

  // 调试信息（已注释）
  // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

  // ===== IMU时间戳回环检测 =====
  // 检查时间戳是否出现倒退现象，这可能导致系统不稳定
  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu loop back, clear deque");

    // 可选的缓冲区清理操作（已注释）
    // imu_deque.shrink_to_fit();
    // std::cout << "check time:" << timestamp << ";" << last_timestamp_imu << '\n';
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    // 线程同步（已注释）
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
    return;  // 发现时间回环，丢弃当前数据
  }

  // ===== 数据入队和状态更新 =====
  pending_imu_deque.emplace_back(msg);  // 将校正后的IMU数据加入队列
  last_timestamp_imu = timestamp;       // 更新最新IMU时间戳

  sig_buffer.notify_all();
}

/**
 * @brief 多传感器数据包同步函数
 * @param meas 输出的同步测量数据组，包含LiDAR和IMU数据
 * @return true 同步成功，false 同步失败或数据不足
 * @details 核心的数据同步算法，实现LiDAR和IMU数据的精确时间对齐
 *
 *          同步策略：
 *          1. 以LiDAR扫描周期为时间基准（通常10-20Hz）
 *          2. 收集每个LiDAR周期内的所有IMU数据（通常200Hz+）
 *          3. 处理传感器时间偏差和数据丢失
 *          4. 支持纯LiDAR模式（IMU禁用）
 *
 *          时间对齐原理：
 *          - LiDAR提供低频高精度的空间约束
 *          - IMU提供高频的运动先验信息
 *          - 通过时间戳匹配实现多传感器融合
 */
bool sync_packages(MeasureGroup & meas)
{
  double available_last_timestamp_imu = -1.0;
  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    const std::size_t pending_lidar_frames = pending_lidar_buffer.size();
    const std::size_t pending_imu_samples = pending_imu_deque.size();
    flush_pending_sensor_data_locked();
    available_last_timestamp_imu = last_timestamp_imu;
    point_lio::RuntimeStatistics::instance().recordSensorQueues(
      pending_lidar_frames,
      lidar_buffer.size(),
      pending_imu_samples,
      imu_deque.size());
  }

  {
    // ===== 纯LiDAR模式（IMU禁用） =====
    if (!imu_en) {
      if (!lidar_buffer.empty()) {
        if (!lidar_pushed) {
          // 提取LiDAR数据和时间戳
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          lose_lid = false;

          // 检查LiDAR点云是否为空
          if (meas.lidar->points.empty()) {
            std::cout << "lose lidar" << '\n';
            lose_lid = true;  // 标记LiDAR数据丢失
          } else {
            // 计算LiDAR扫描结束时间
            // 注意：点云中的curvature字段存储了相对时间偏移
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points) {
              if (pt.curvature > end_time) {
                end_time = pt.curvature;  // 找到最大时间偏移
              }
            }
            // 计算绝对结束时间：开始时间 + 相对偏移
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
          }
          lidar_pushed = true;  // 标记LiDAR数据已推送
        }

        // 清理已处理的数据
        time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;  // 重置推送标志

        // 根据数据完整性返回结果
        if (!lose_lid) {
          return true;  // 数据完整，同步成功
        } else {
          return false;  // 数据丢失，同步失败
        }
      }
      return false;  // 缓冲区为空，无数据可同步
    }

    // ===== LiDAR-IMU融合模式 =====

    // 检查数据缓冲区状态
    if (lidar_buffer.empty() || imu_deque.empty()) {
      return false;  // 任一传感器数据不足，无法同步
    }

    // ===== 步骤1：推送LiDAR扫描数据 =====
    if (!lidar_pushed) {
      lose_lid = false;
      meas.lidar = lidar_buffer.front();          // 获取队列头部的LiDAR数据
      meas.lidar_beg_time = time_buffer.front();  // 获取对应的开始时间戳

      // 检查LiDAR点云完整性
      if (meas.lidar->points.size() < 1) {
        std::cout << "lose lidar" << '\n';
        lose_lid = true;  // 标记LiDAR数据丢失
        // 可选择直接丢弃该帧（已注释）
        // lidar_buffer.pop_front();
        // time_buffer.pop_front();
        // return false;
      } else {
        // 计算LiDAR扫描的结束时间
        // 遍历所有点，找到最大的时间偏移（存储在curvature字段）
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points) {
          if (pt.curvature > end_time) {
            end_time = pt.curvature;  // 更新最大时间偏移
          }
        }
        // 计算绝对结束时间：开始时间 + 最大偏移时间（毫秒转秒）
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        // 调试信息（已注释）
        // std::cout << "check time lidar:" << end_time << '\n';

        meas.lidar_last_time = lidar_end_time;  // 设置LiDAR结束时间
      }
      lidar_pushed = true;  // 标记LiDAR数据已推送
    }

    // ===== 步骤2：检查IMU数据充足性 =====
    // 确保有足够的IMU数据覆盖整个LiDAR扫描周期

    if (!lose_lid && (available_last_timestamp_imu < lidar_end_time)) {
      // LiDAR数据完整，但IMU数据还未覆盖到LiDAR结束时间
      return false;  // 等待更多IMU数据
    }

    if (lose_lid && available_last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
      // LiDAR数据丢失，但需要等待足够的IMU数据进行时间积分
      // lidar_time_inte: LiDAR时间积分窗口
      return false;  // 等待更多IMU数据
    }

    // ===== 步骤3：收集对应时间段的IMU数据 =====

    if (!lose_lid && !imu_pushed) {
      // ===== LiDAR数据完整的情况 =====
      // 收集从LiDAR开始到结束时间段内的所有IMU数据

      if (p_imu->imu_need_init_) {  // 检查IMU是否需要初始化数据
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        imu_next = *(imu_deque.front());  // 保存下一帧IMU数据
        meas.imu.shrink_to_fit();         // 优化IMU数据向量内存

        // 收集时间窗口内的所有IMU数据
        while (imu_time < lidar_end_time) {
          meas.imu.emplace_back(imu_deque.front());  // 加入IMU数据到测量组
          imu_last = imu_next;                       // 更新上一帧IMU数据
          imu_deque.pop_front();                     // 从队列中移除已使用的数据

          if (imu_deque.empty())
            break;  // 队列为空则退出

          // 获取下一帧时间戳
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());  // 保存下一帧数据
        }
      }
      imu_pushed = true;  // 标记IMU数据已推送
    }

    if (lose_lid && !imu_pushed) {
      // ===== LiDAR数据丢失的情况 =====
      // 收集LiDAR时间窗口 + 积分时间内的IMU数据，用于纯惯导传播

      if (p_imu->imu_need_init_) {  // 检查IMU是否需要初始化数据
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();  // 优化内存使用
        imu_next = *(imu_deque.front());

        // 收集扩展时间窗口内的IMU数据
        // 时间窗口：LiDAR开始时间 + 预设的积分时间间隔
        while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
          meas.imu.emplace_back(imu_deque.front());  // 加入IMU数据
          imu_last = imu_next;                       // 更新历史数据
          imu_deque.pop_front();                     // 移除已使用数据

          if (imu_deque.empty())
            break;  // 防止队列下溢

          // 获取下一帧时间戳和数据
          imu_time = get_time_sec(imu_deque.front()->header.stamp);
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;  // 标记IMU数据已推送
    }

    // ===== 步骤4：清理缓冲区，完成同步 =====
    lidar_buffer.pop_front();  // 移除已处理的LiDAR数据
    time_buffer.pop_front();   // 移除对应的时间戳
    lidar_pushed = false;      // 重置LiDAR推送标志
    imu_pushed = false;        // 重置IMU推送标志

    return true;  // 同步完成，返回成功
  }
}
