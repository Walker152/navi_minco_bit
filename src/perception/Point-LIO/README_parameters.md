# Point-LIO 参数系统详细说明

## 概述

`parameters.h` 和 `parameters.cpp` 是 Point-LIO 系统的参数管理核心，负责：
- 系统配置参数的声明和定义
- ROS2 参数的读取和解析
- 全局变量的管理
- 滤波器初始化参数设置

## 文件结构分析

### parameters.h - 参数声明头文件

#### 1. 核心功能
- **全局变量声明**: 声明所有系统级参数
- **外部接口定义**: 提供参数访问接口
- **类型定义**: 定义复杂数据类型
- **函数声明**: 声明参数处理相关函数

#### 2. 主要包含内容

##### 2.1 依赖库引入
```cpp
#include <Python.h>                    // Python接口支持
#include <ivox/ivox3d.h>               // iVox 3D映射库
#include <Eigen/Core>                  // Eigen线性代数库
#include <rclcpp/rclcpp.hpp>           // ROS2核心功能
#include <sensor_msgs/msg/imu.hpp>     // IMU消息类型
#include <livox_ros_driver2/msg/custom_msg.hpp> // Livox自定义消息
#include "IMU_Processing.h"            // IMU处理模块
#include "preprocess.h"                // 预处理模块
```

##### 2.2 关键类型定义
```cpp
// iVox地图类型定义 - 根据编译选项选择不同的节点类型
#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif
```

##### 2.3 全局变量分类

**时间管理变量**
```cpp
extern bool is_first_frame;                    // 是否为第一帧
extern double lidar_end_time;                  // 激光雷达结束时间
extern double first_lidar_time;                // 第一帧激光雷达时间
extern double last_timestamp_lidar;            // 上一帧激光雷达时间戳
extern double last_timestamp_imu;              // 上一帧IMU时间戳
extern double time_diff_lidar_to_imu;          // 激光雷达到IMU的时间差
```

**状态管理变量**
```cpp
extern state_input state_in;                   // 输入状态向量(24维)
extern state_output state_out;                 // 输出状态向量(30维)
extern int pcd_index;                          // 点云文件索引
```

**传感器配置变量**
```cpp
extern std::string lid_topic, imu_topic;       // 激光雷达和IMU话题名称
extern int lidar_type;                         // 激光雷达类型标识
extern bool imu_en;                            // IMU使能标志
extern double imu_time_inte;                   // IMU积分时间间隔
```

**算法控制变量**
```cpp
extern bool use_imu_as_input;                  // IMU作为输入还是观测
extern bool extrinsic_est_en;                  // 外参在线估计使能
extern bool prop_at_freq_of_imu;               // 是否以IMU频率传播
extern bool space_down_sample;                 // 空间降采样使能
```

**滤波器参数变量**
```cpp
extern double laser_point_cov;                 // 激光雷达点协方差
extern double acc_cov_input, gyr_cov_input;    // IMU输入模式协方差
extern double acc_cov_output, gyr_cov_output;  // IMU输出模式协方差
extern double b_gyr_cov, b_acc_cov;            // IMU偏置协方差
extern double imu_meas_acc_cov, imu_meas_omg_cov; // IMU测量协方差
```

**几何参数变量**
```cpp
extern std::vector<double> extrinT;            // 外参平移向量
extern std::vector<double> extrinR;            // 外参旋转矩阵
extern std::vector<double> gravity_init;       // 初始重力向量
extern std::vector<double> gravity;            // 当前重力向量
```

### parameters.cpp - 参数实现文件

#### 1. 核心功能
- **参数初始化**: 设置默认参数值
- **ROS2参数读取**: 从launch文件或命令行读取参数
- **协方差矩阵初始化**: 设置滤波器初始协方差
- **工具函数实现**: 提供参数处理辅助函数

#### 2. 主要函数详解

##### 2.1 readParameters() - 参数读取函数
```cpp
void readParameters(std::shared_ptr<rclcpp::Node> & nh)
{
    // 1. 初始化预处理和IMU处理对象
    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());
    
    // 2. 逐个读取ROS2参数
    try {
        // 算法控制参数
        nh->declare_parameter<bool>("use_imu_as_input", false);
        nh->get_parameter("use_imu_as_input", use_imu_as_input);
        
        // 传感器话题参数
        nh->declare_parameter<std::string>("common.lid_topic", ".livox.lidar");
        nh->get_parameter("common.lid_topic", lid_topic);
        
        // 滤波器噪声参数
        nh->declare_parameter<double>("mapping.satu_acc", 3.0);
        nh->get_parameter("mapping.satu_acc", satu_acc);
        
        // ... 更多参数读取
    } catch (const rclcpp::ParameterTypeException & e) {
        RCLCPP_ERROR(nh->get_logger(), "Parameter type exception: %s", e.what());
    }
    
    // 3. 设置iVox参数
    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    }
    // ... 其他iVox配置
    
    // 4. 设置IMU重力向量
    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}
```

**参数读取流程**:
1. `declare_parameter`: 声明参数及默认值
2. `get_parameter`: 从ROS2参数服务器获取实际值
3. 异常处理: 捕获参数类型错误

##### 2.2 reset_cov() - 输入滤波器协方差重置
```cpp
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    // 整体初始化为0.1 * I
    P_init = MD(24, 24)::Identity() * 0.1;
    
    // 重力估计不确定性较小
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    
    // 陀螺仪和加速度计偏置不确定性
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}
```

**协方差矩阵结构** (24×24):
- 位置 [0:2] - 较大不确定性 (0.1)
- 旋转 [3:5] - 较大不确定性 (0.1) 
- 外参旋转 [6:8] - 较大不确定性 (0.1)
- 外参平移 [9:11] - 较大不确定性 (0.1)
- 速度 [12:14] - 较大不确定性 (0.1)
- 陀螺偏置 [15:17] - 中等不确定性 (0.001)
- 加速度偏置 [18:20] - 中等不确定性 (0.001)
- 重力 [21:23] - 小不确定性 (0.0001)

##### 2.3 reset_cov_output() - 输出滤波器协方差重置
```cpp
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    // 整体初始化为0.01 * I (比输入模式更信任初值)
    P_init_output = MD(30, 30)::Identity() * 0.01;
    
    // 重力估计不确定性
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    
    // 外参估计不确定性  
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}
```

**协方差矩阵结构** (30×30):
- 位置 [0:2] - 小不确定性 (0.01)
- 旋转 [3:5] - 小不确定性 (0.01)
- 速度 [6:8] - 小不确定性 (0.01)
- 陀螺偏置 [9:11] - 小不确定性 (0.01)
- 加速度偏置 [12:14] - 小不确定性 (0.01)
- 重力 [15:17] - 极小不确定性 (0.0001)
- 外参旋转 [18:20] - 小不确定性 (0.01)
- 外参平移 [21:23] - 极小不确定性 (0.0001)
- 外参旋转(额外) [24:26] - 中等不确定性 (0.001)
- 外参平移(额外) [27:29] - 中等不确定性 (0.001)

##### 2.4 SO3ToEuler() - 旋转矩阵到欧拉角转换
```cpp
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 & rot)
{
    // 计算sin(yaw)用于判断奇异性
    double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;  // 检查万向锁
    
    double x, y, z;
    if (!singular) {
        // 正常情况：ZYX欧拉角提取
        x = atan2(rot(2, 1), rot(2, 2));  // Roll
        y = atan2(-rot(2, 0), sy);        // Pitch  
        z = atan2(rot(1, 0), rot(0, 0));  // Yaw
    } else {
        // 万向锁情况处理
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;  // 设置yaw为0
    }
    
    return Eigen::Matrix<double, 3, 1>(x, y, z);
}
```

##### 2.5 open_file() - 日志文件初始化
```cpp
void open_file()
{
    // 打开状态输出日志文件
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    
    // 打开IMU点对点处理日志文件
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    
    // 检查文件打开状态
    if (fout_out && fout_imu_pbp)
        std::cout << "~~~~" << ROOT_DIR << " file opened" << '\n';
    else
        std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << '\n';
}
```

## 在LIO系统中的作用

### 1. **配置管理中心**
- 统一管理所有系统参数
- 支持运行时参数修改
- 提供参数验证和错误处理

### 2. **模块间通信桥梁**
- 为各个模块提供共享参数访问
- 确保参数一致性
- 减少模块间耦合

### 3. **系统初始化支持**
- 提供滤波器初始协方差设置
- 支持传感器外参配置
- 管理算法模式切换

### 4. **调试和日志支持**
- 提供日志文件管理
- 支持运行时状态记录
- 便于系统调试和分析

## 参数分类详解

### 算法核心参数
```cpp
use_imu_as_input         // IMU使用模式 (true: 输入, false: 观测)
extrinsic_est_en         // 外参在线估计
prop_at_freq_of_imu      // IMU频率传播
space_down_sample        // 空间降采样
```

### 传感器参数  
```cpp
lid_topic, imu_topic     // 传感器话题
lidar_type              // 激光雷达类型 (1:Livox, 2:Velodyne, etc.)
imu_en                  // IMU使能
satu_acc, satu_gyro     // IMU饱和值
acc_norm                // 加速度单位
```

### 滤波器参数
```cpp
// 观测噪声
laser_point_cov         // 激光点协方差
imu_meas_acc_cov        // IMU加速度测量协方差
imu_meas_omg_cov        // IMU角速度测量协方差

// 过程噪声
acc_cov_input/output    // 加速度过程噪声
gyr_cov_input/output    // 角速度过程噪声
b_acc_cov, b_gyr_cov    // 偏置过程噪声
```

### 几何参数
```cpp
extrinT, extrinR        // 激光雷达到IMU外参
gravity, gravity_init   // 重力向量
filter_size_surf_min    // 表面特征滤波尺寸
filter_size_map_min     // 地图滤波尺寸
```

### 发布控制参数
```cpp
path_en                          // 路径发布使能
scan_pub_en                      // 点云发布使能
scan_body_pub_en                 // 机体坐标系点云发布
publish_odometry_without_downsample // 高频里程计发布
tf_send_en                       // TF变换发布
```

## 使用建议

### 1. **参数调优顺序**
1. 首先设置传感器基础参数 (话题、类型、外参)
2. 然后调整滤波器噪声参数
3. 最后优化算法控制参数

### 2. **常用参数组合**
- **高精度IMU**: `use_imu_as_input=true`, 降低IMU噪声参数
- **消费级IMU**: `use_imu_as_input=false`, 增大IMU噪声参数  
- **室内环境**: 减小`filter_size_surf_min`, 增大`plane_thr`
- **室外环境**: 增大`DET_RANGE`, 减小`plane_thr`

### 3. **调试技巧**
- 启用`runtime_pos_log`查看状态估计过程
- 使用`pcd_save_en`保存点云进行离线分析
- 通过`publish_odometry_without_downsample`获得高频输出

这套参数系统为Point-LIO提供了极大的灵活性，允许用户根据具体应用场景进行精细调优。