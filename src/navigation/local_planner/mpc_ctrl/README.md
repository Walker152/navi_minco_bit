# DreamChaser MPC控制器

## 项目概述

`dreamchaser_mpc_ctrl` 是一个基于模型预测控制（Model Predictive Control, MPC）的全向轮机器人路径跟踪控制器，作为ROS2 Nav2框架的控制器插件实现。该控制器专为RoboMaster哨兵机器人设计，具有预测能力强、约束处理好、控制精度高的特点。

## 系统架构

### 核心组件

- **MPController**: 主控制器类，继承自`nav2_core::Controller`
- **MPCSolver**: MPC优化问题求解器
- **MPCParameters**: MPC参数配置结构体

### 控制流程

```
当前状态 → 参考轨迹生成 → MPC优化求解 → 控制量输出 → 速度约束 → 全向轮控制
```

## 技术特性

### MPC模型设计

**状态向量**: `x = [px, py, θ, vx, vy, ω]ᵀ` (位置、姿态、速度)
**控制向量**: `u = [ax, ay, α]ᵀ` (线性加速度、角加速度)

**离散化系统模型**:
```
x(k+1) = A*x(k) + B*u(k)

A = [1, 0, 0, dt, 0,  0 ]    B = [0,  0,  0 ]
    [0, 1, 0, 0,  dt, 0 ]        [0,  0,  0 ]
    [0, 0, 1, 0,  0,  dt]        [0,  0,  0 ]
    [0, 0, 0, 1,  0,  0 ]        [dt, 0,  0 ]
    [0, 0, 0, 0,  1,  0 ]        [0,  dt, 0 ]
    [0, 0, 0, 0,  0,  1 ]        [0,  0,  dt]
```

### 优化问题

**目标函数**:
```
J = Σ(k=0→N-1) [||x(k)-x_ref(k)||²_Q + ||u(k)||²_R] + ||x(N)-x_ref(N)||²_Qf
```

**约束条件**:
- 系统动力学约束
- 速度限制：`|vx|, |vy| ≤ max_linear_vel`, `|ω| ≤ max_angular_vel`
- 加速度限制：`|ax|, |ay| ≤ max_linear_acc`, `|α| ≤ max_angular_acc`

## 项目结构

```
mpc_ctrl/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS2包配置
├── dreamchaser_mpc_ctrl.xml    # Nav2插件描述
├── README.md                   # 本文档
├── include/
│   ├── dreamchaser_mpc_ctrl.hpp # 主控制器头文件
│   └── mpc.hpp                 # MPC求解器头文件
└── src/
    ├── dreamchaser_mpc_ctrl.cpp # 主控制器实现
    └── mpc.cpp                 # MPC求解器实现
```

## 构建和安装

### 依赖项

- **系统依赖**: ROS2 Humble/Iron, Eigen3
- **ROS2包依赖**: nav2_core, nav2_util, nav2_costmap_2d, tf2, pluginlib

### 编译

```bash
cd ~/your_ws
colcon build --packages-select dreamchaser_mpc_ctrl
source install/setup.bash
```

### 插件注册

插件通过`dreamchaser_mpc_ctrl.xml`自动注册到Nav2框架：

```xml
<library path="dreamchaser_mpc_ctrl">
  <class type="dreamchaser_mpc_ctrl::MPController" 
         base_class_type="nav2_core::Controller">
    <description>MPC controller for omnidirectional robots</description>
  </class>
</library>
```

## 配置参数

### 关键参数说明

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "dreamchaser_mpc_ctrl::MPController"
      
      # MPC核心参数
      prediction_horizon: 10        # 预测范围N (建议10-20)
      control_frequency: 20.0       # 控制频率 [Hz]
      prediction_dt: 0.05          # 预测时间步长 [s]
      
      # 权重矩阵 (调整控制性能的关键参数)
      state_weights: [10.0, 10.0, 1.0, 1.0, 1.0, 0.1]  # [px,py,θ,vx,vy,ω]
      control_weights: [1.0, 1.0, 0.1]                  # [ax,ay,α]
      terminal_weights: [20.0, 20.0, 2.0, 0.0, 0.0, 0.0] # 终端权重
      
      # 物理约束
      max_linear_velocity: 2.0      # 最大线速度 [m/s]
      max_angular_velocity: 2.0     # 最大角速度 [rad/s]
      max_linear_acceleration: 3.0  # 最大线加速度 [m/s²]
      max_angular_acceleration: 3.0 # 最大角加速度 [rad/s²]
```

### 参数调优指南

- **提高跟踪精度**: 增加`state_weights`中位置权重(px, py)
- **增强控制平滑性**: 增加`control_weights`数值
- **改善转向性能**: 调整角度权重(θ)和角速度约束
- **实时性优化**: 减少`prediction_horizon`，提高`control_frequency`

## 使用方法

### 1. 启动导航系统

```bash
# 启动完整导航栈
ros2 launch navi2_bringup navigation2.launch.py

# 或单独启动控制器服务器
ros2 run nav2_controller controller_server --ros-args \
  --params-file your_nav2_params.yaml
```

### 2. 运行时切换控制器

```bash
# 动态切换到MPC控制器
ros2 param set /controller_server FollowPath.plugin "dreamchaser_mpc_ctrl::MPController"

# 重新配置控制器
ros2 service call /controller_server/reload_plugins std_srvs/srv/Empty
```

### 3. 调试和监控

```bash
# 查看控制器状态
ros2 topic echo /cmd_vel

# 可视化本地路径规划
ros2 topic echo /local_plan

# 监控MPC求解性能
ros2 topic hz /cmd_vel  # 检查控制频率
```

## 核心算法实现

### MPC求解器

采用**无约束二次规划解析解**方法，避免了复杂的数值优化：

```cpp
// 关键算法：解析解公式
u_opt = -(R_bar + Gamma^T * Q_bar * Gamma)^(-1) * Gamma^T * Q_bar * (Phi * x0 - ref)
```

### 参考轨迹生成

- 基于当前位置在全局路径上进行前瞻采样
- 自适应采样距离，考虑当前速度
- 生成包含位置和期望速度的完整参考轨迹

### 实时性优化

- 预计算系统矩阵A和B
- 使用Eigen库的高效矩阵运算
- LDLT分解求解线性方程组

## 性能特点

### 优势
- **预测控制**: 考虑未来轨迹，提前做出最优决策
- **约束处理**: 直接在优化中处理速度和加速度限制
- **全局最优**: 在预测范围内寻找全局最优解
- **参数可调**: 通过权重矩阵灵活调整控制特性

### 与PID控制器对比
| 特性 | MPC控制器 | PID控制器 |
|------|-----------|-----------|
| 预测能力 | ✅ 多步预测 | ❌ 仅考虑当前 |
| 约束处理 | ✅ 直接优化 | ❌ 后处理限幅 |
| 控制平滑性 | ✅ 全局优化 | ⚠️ 局部反应 |
| 计算复杂度 | ⚠️ 中等 | ✅ 极低 |
| 参数调优 | ⚠️ 需要经验 | ✅ 相对简单 |

## 故障排除

### 常见问题

1. **编译错误**: `_mm_prefetch` 冲突
   ```cpp
   // 解决方案：在mpc.hpp顶部添加
   #define EIGEN_DONT_VECTORIZE
   ```

2. **MPC求解失败**
   - 检查参考轨迹是否有效
   - 降低预测范围N
   - 增加数值稳定性（权重矩阵正定性）

3. **控制器振荡**
   - 增加控制权重R
   - 降低控制频率
   - 检查时间步长dt设置

4. **跟踪精度不足**
   - 增加状态权重Q中的位置权重
   - 提高预测范围N
   - 检查参考轨迹生成质量

### 调试工具

```bash
# 实时查看MPC参数
ros2 param list /controller_server | grep FollowPath

# 性能分析
ros2 run nav2_util perf_test

# 可视化调试
rviz2 -d /path/to/your/config.rviz
```

## 开发指南

### 扩展功能

1. **障碍物避让**: 在约束中添加障碍物距离约束
2. **自适应权重**: 根据路径曲率动态调整权重
3. **鲁棒MPC**: 考虑模型不确定性和外界扰动
4. **学习型MPC**: 集成强化学习优化参数

### 贡献代码

1. Fork本项目
2. 创建功能分支：`git checkout -b feature/your-feature`
3. 提交代码：`git commit -am 'Add some feature'`
4. 推送分支：`git push origin feature/your-feature`
5. 提交Pull Request

## 许可证

本项目遵循MIT许可证，详见LICENSE文件。

## 联系方式

- **开发者**: DreamChaser Team
- **邮箱**: your-email@example.com
- **项目主页**: https://github.com/your-org/dreamchaser_mpc_ctrl

---

**注意**: 本控制器专为RoboMaster哨兵机器人优化，在其他机器人平台上使用时可能需要调整参数和模型。