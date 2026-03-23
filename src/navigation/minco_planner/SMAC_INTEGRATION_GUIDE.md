# SMAC 2D A* 规划器集成说明

## 概述
本次移植成功将 `nav2_smac_planner` 中的 SMAC 2D A* 搜索算法集成到 `minco_planner` 项目中。

## 文件结构

### 新增的文件
```
minco_planner/
├── include/smac_search/
│   ├── constants.hpp          # 常量定义（运动模型、代价值等）
│   ├── types.hpp              # 类型定义（SearchInfo, SmootherParams等）
│   ├── collision_checker.hpp # 碰撞检测器接口
│   ├── node_2d.hpp            # 2D节点定义
│   └── smac_planner_2d_simple.hpp # 简化的SMAC 2D规划器接口
└── src/smac_search/
    ├── collision_checker.cpp
    ├── node_2d.cpp
    └── smac_planner_2d_simple.cpp
```

### 修改的文件
- `CMakeLists.txt` - 添加SMAC源文件到编译列表
- `package.xml` - 已包含所需依赖（nav2_costmap_2d, nav2_util）
- `include/minco_core/minco_planner.hpp` - 添加SMAC规划器成员和头文件引用
- `src/minco_core/minco_planner.cpp` - 集成SMAC规划器到makePlan函数

## 使用方法

### 1. 参数配置
在您的导航参数文件（如 `nav2_params.yaml`）中添加以下参数：

```yaml
minco_planner:
  ros__parameters:
    # 原有参数...
    use_astar: true          # 是否使用原始A*（保留）
    use_smac: true           # 是否使用SMAC 2D（新增）
    allow_unknown: true      # 是否允许穿越未知区域
    tolerance: 0.5           # 目标容差（米）
```

### 2. 切换规划器
通过设置 `use_smac` 参数来选择使用的规划器：
- `use_smac: true` - 使用SMAC 2D A*规划器
- `use_smac: false` - 使用原始A*规划器

**注意：** SMAC规划器在配置时会自动初始化，只有当 `use_smac` 为 true 时才会使用。

### 3. 运行系统
```bash
cd /home/alioth/2025-sentry-navi
bash build.bash
bash start.bash
```

## 特性说明

### SMAC 2D简化版特点
1. **基于Costmap**: 直接使用 `nav2_costmap_2d::Costmap2D` 进行规划
2. **优先队列A***: 使用标准A*算法，支持8连通邻域
3. **碰撞检测**: 集成了GridCollisionChecker进行实时碰撞检测
4. **路径回溯**: 支持从目标到起点的路径回溯

### 与原始A*的区别
| 特性 | 原始A* | SMAC 2D |
|------|--------|---------|
| 算法类型 | 波前扩散 (wavefront) | 优先队列A* |
| 代价计算 | 基于势场 | 基于启发式+实际代价 |
| 碰撞检测 | 简单索引查询 | GridCollisionChecker |
| 路径质量 | 较平滑 | 可能有锯齿 |
| 计算速度 | 较慢（大地图） | 较快（定向搜索） |

## 调试和日志

### 日志输出
当使用SMAC规划器时，您会看到如下日志：
```
[MincoPlanner] SMAC 2D Planner initialized and will be used for path planning
[MincoPlanner] SMAC 2D planning time: 0.123 seconds, path length: 456
```

### 可能的错误信息
- `"Costmap not set!"` - Costmap未正确初始化
- `"SMAC 2D: Failed to find path"` - 无法找到有效路径（可能因为障碍物阻挡）

## 性能调优

### 参数调整
在 `SmacPlanner2DSimple` 中可以调整的参数：
```cpp
smac_planner_->setParameters(
  allow_unknown_,  // 是否允许穿越未知区域
  1000000,         // 最大迭代次数
  tolerance_       // 目标容差
);
```

### 建议设置
- **大型开放空间**: 增加 `max_iterations` 到 2000000
- **密集障碍物**: 降低 `tolerance` 到 0.125
- **动态环境**: 设置 `allow_unknown` 为 true

## 未来改进方向

1. **完整模板支持**: 当前版本只支持Node2D，可扩展支持NodeHybrid和NodeLattice
2. **解析膨胀**: 集成analytic_expansion模块以生成更优路径
3. **路径平滑**: 添加Smoother模块对路径进行后处理
4. **动态参数**: 支持动态参数调整（通过ROS 2参数服务）

## 技术细节

### 命名空间
所有SMAC相关代码位于 `minco_planner::smac` 命名空间下，以避免与原有代码冲突。

### 依赖关系
- `nav2_costmap_2d` - Costmap和碰撞检测
- `nav2_util` - ROS 2生命周期节点工具
- `rclcpp_lifecycle` - 生命周期管理

### 关键类
- `SmacPlanner2DSimple` - 主规划器类
- `GridCollisionChecker` - 碰撞检测器  
- `Node2D` - 2D图节点

## 故障排除

### 编译错误
如果遇到编译错误，请确保：
1. 所有依赖包已安装（`nav2_costmap_2d`, `nav2_util`）
2. ROS 2环境已正确配置
3. C++标准设置为C++17

### 运行时错误
如果规划失败，检查：
1. Costmap是否正确加载
2. 起点和终点是否在地图范围内
3. 路径是否被障碍物完全阻挡

## 联系和支持

如有问题，请查看：
- 原始nav2_smac_planner文档: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner
- ROS 2 Navigation文档: https://navigation.ros.org/

---

创建日期: 2026年2月8日
版本: 1.0
