# 2025 Sentry Navi (ROS 2 Humble)

## 项目概述
本仓库为 RoboMaster 哨兵系统的 ROS 2 集成工程，包含感知、定位重定位、导航规划控制、决策与通信、工具链与接口定义。

## 目录结构

### 顶层目录
- decision: 决策相关包
- navigation: 导航与控制相关包
- perception: 感知与地图相关包
- ros_interfaces: 消息与接口定义
- utils: 工具类包

### 包结构清单

#### decision
- bt_manager

#### navigation
- back_up_twz_free
- communication
- minco_controller
- minco_planner
- navi2_bringup

#### perception
- icp_relocalization
- lidar_merger
- livox_ros_driver2
- msg_convert
- Point-LIO
- rog_map

#### interfaces
- ros_interfaces

#### utils
- pcd2ele
- pcd2esdf
- pcd2pgm
- pclfilter

## 功能说明

### 决策层
- bt_manager: 行为树决策与任务流程控制。

### 导航与控制层
- navi2_bringup: 导航系统启动与参数组织。
- minco_planner: 全局路径/轨迹规划。
- minco_controller: 轨迹跟踪与运动控制输出。
- back_up_twz_free: 备用导航相关能力。
- communication: 与下位机/外部系统通信。

### 感知与地图层
- livox_ros_driver2: Livox 雷达驱动与数据发布。
- Point-LIO: 激光惯导里程计。
- icp_relocalization: 全局重定位。
- lidar_merger: 多源点云融合。
- msg_convert: 感知链路消息格式转换。
- rog_map: 地图与距离场相关能力。
- pclfilter: ROS2 版本 Depth Cluster（地面分割与障碍物聚类）。

### 接口与工具层
- ros_interfaces: 自定义消息、服务、动作接口。
- pcd2ele: 点云转高程相关数据处理。
- pcd2esdf: 点云转 ESDF 相关数据处理。
- pcd2pgm: 点云转栅格图相关处理。
- pclfilter: 点云过滤与预处理。

## 使用说明

### 1) 环境准备
- 安装 Ubuntu 22.04
- 安装 ROS 2 Humble
- 安装 colcon 与常用 ROS 2 构建工具

### 2) 构建
- 全量构建: ./build.bash
- 清理构建: ./clean.bash

### 3) 运行
- 一键启动: ./start.bash
- 调试/演示启动: ./play.bash

### 4) 开发建议
- 修改参数后建议重新 source install/setup.bash
- 功能联调建议分模块逐步启动（驱动 -> 感知 -> 定位 -> 导航 -> 决策）

## 配置方法与需求

### 基础配置需求
- 操作系统: Ubuntu 22.04
- 中间件: ROS 2 Humble
- 构建工具: colcon
- 传感器: Livox 雷达（按实际硬件接入）

### 运行前配置
- 网络与设备权限配置完成
- 雷达与 IMU 话题可正常发布
- TF 树基础关系可用
- 地图与重定位所需数据文件路径已配置

