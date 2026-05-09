# 任务背景与目标
你是一个资深的机器人算法工程师与现代 C++17 专家。
我现在需要你辅助重构一个基于 ROS 2 (Nav2 插件体系) 的 RoboMaster 哨兵机器人 MPC 控制器。

当前的系统已经从运动学 MPC 彻底升级为**基于全局受力输入的 6D 解耦动力学 LTI-MPC (Linear Time-Invariant MPC)**。
我们的核心控制哲学是：**状态与控制量在全局坐标系下解耦以实现 LTI 常数矩阵加速；外部环境（重力/摩擦力）作为扰动仿射项内生于动力学梯度中；物理极限通过“16 维动态耦合多边形”榨干电机性能；速度安全通过“后置力矩调速器”进行物理反推保障。**

# 架构革新与核心数学机制 (极其重要，必须严格遵循)

### 1. 动态耦合多边形约束 (Dynamic Coupled Polytope)
**问题背景：** 传统的额度切分法（预留固定的偏航力矩额度）会在直线行驶时浪费平动推力。
**终极解法：** 我们在 QP 求解器的 `A_con` 中，将平动推力 $F_{xy}$ 与偏航力矩 $M_z$ 进行动态线性耦合。
底盘物理极限满足：$|F_{proj\_octagon}| + \frac{1}{2R} |M_z| \le 2 f_{max}$ （$R$ 为底盘半径）。
在预测视界 $N$ 内，每一步的 `A_con` 包含 **17 行**：
- 前 16 行：分别对应八边形的 8 个投影方向在 $+M_z$ 和 $-M_z$ 下的耦合（系数分别为 $\pm 1$ 和 $\pm \sqrt{2}/2$，以及 $M_z$ 的系数 $\pm 1/(2R)$）。其上界统一为 $2 f_{max}$。
- 第 17 行：时变功率约束 $v_{gx,ref} F_{gx} + v_{gy,ref} F_{gy} \le P_{limit}$。
- **变量基础边界 (`lb`, `ub`)**：$F_{gx}, F_{gy} \in [-2f_{max}, 2f_{max}]$， $M_{gz} \in [-M_{max}, M_{max}]$。

### 2. 后置力矩调速器 (Force Governor)
**问题背景：** 将速度硬约束写入 QP 易导致受迫扰动时求解器崩溃 (Infeasible)。
**终极解法：** 在求解器返回最优力 $F^*$ 后，下发前执行显式积分预测：$v_{cmd} = v_0 + (F^*/m)\Delta t$。
若 $|v_{cmd}| > V_{max}$，则将速度等比例缩放至 $V_{max}$，并**数学反推**所需的合法推力：$F_{safe} = (V_{safe} - v_0) \cdot m / \Delta t$。既保证安全，又避免无解。

### 3. 延迟补偿的提取策略
**摒弃状态前推：** 绝对不通过 $x = x_0 + v\Delta t$ 修改初始状态。
**最优提取：** 用当前真实状态 $x_0$ 求解出完整的未来最优控制序列 $U$。根据延迟时间算出索引 `delay_idx`，直接从全序列中提取 `U[delay_idx]` 下发，实现考虑地形与曲率的真实延迟预判。

# 模块拆分与职责定义

### 1. `MincoMpcController` (ROS 2 层 & 宏观调度层)
- **职责：** 负责 ROS 2 节点、参数热更新 (`on_set_parameters_callback`)、状态融合。
- **状态融合：** 从 Nav2 获取平动位置 `pose`，从雷达里程计获取姿态和速度，并进行**杆臂补偿 (Lever Arm)** 后旋转到 map 全局系。
- **参数派生：** `horizon` 必须由 `ceil(lookahead_time / dt)` 动态推导计算，不可作为独立静态参数。
- **调度流：** 构建参考轨迹 -> 调用 `ModelBuilder::buildQP` -> 调用 `MpcSolver::solve(..., delay_idx)` -> 触发力矩调速器 (Force Governor) -> 动态比例安全限幅 (`clampCoupledForce`) -> 发布 Wrench 力矩与备用 Twist 速度。

### 2. `ModelBuilder` (动力学建模与约束层 - 纯 C++ & Eigen)
- **无 ROS 依赖：** 包含 `ModelConfig` 结构体配置。
- **LTI 矩阵：** $A$ 包含积分 $\Delta t$；$B$ 包含 $\Delta t^2 / (2m)$ 和 $\Delta t / m$ 及转动惯量参数；离线构建 $A_{hat}, B_{hat}, H$ 矩阵。
- **干扰项计算：** $D_{grav}$ 融合 Pitch/Roll 投影；$D_{fric}$ 基于参考速度计算，使用 `tanh(v_ref/eps)` 进行平滑。
- **约束构建：** 实现 16 维动态耦合矩阵的逻辑填充。

### 3. `MpcSolver` (求解器层 - 纯 C++ & qpOASES)
- **极简包装：** 仅负责将 Eigen 转为 `qpOASES` 需要的 `real_t` 一维数组。
- **接口：** `solve(const QPProblem& qp, Eigen::Vector3d& out_u, int delay_idx)`，提取并返回延时帧的解。

# 你的具体任务
我们已经完成了 `.cpp` 源文件的重构，且逻辑完全正确。
为了让整个工程能够顺利编译闭环，请你基于上述所有的架构约束和设计哲学，为我生成以下三个极其严谨、结构清晰的 Modern C++17 头文件 (`.hpp`)：
1. `minco_controller/data_types.hpp` (包含 `State`, `Control`, `ReferencePoint`, `ModelConfig`, `QPProblem`, `Attitude` 结构体)
2. `minco_controller/model_builder.hpp`
3. `minco_controller/mpc_solver.hpp`
4. `minco_controller/minco_mpc_controller.hpp`

要求代码注释详尽，包含头文件保护宏，明确的命名空间 `minco_controller`，并且符合上述职责划分规范（比如 `ModelBuilder` 必须暴露 `clampCoupledForce` 静态函数等）。