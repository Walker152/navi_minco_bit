# Depth Cluster 测试指南

本文档介绍如何测试 `depth_cluster` 节点的地面提取和障碍物聚类功能。

## 功能说明

`depth_cluster_node` 实现了以下功能：
1. **地面提取**: 自动识别并提取地面点云（显示为绿色）
2. **障碍物聚类**: 对非地面点进行深度聚类，不同聚类显示为不同颜色
3. **点云分类**: 输出地面点云和聚类后的障碍物点云

## 测试脚本说明

### 1. `test_depth_cluster_demo.py` - 测试数据发布器
发布包含地面和多个障碍物的模拟点云数据。

**功能**:
- 生成平面地面点云
- 生成多个障碍物（长方体、圆柱体）
- 可选添加噪声
- 支持三种场景模式

**发布话题**: `/cloud_registered`

### 2. `monitor_depth_cluster.py` - 效果监控器
实时监控和统计depth_cluster的处理结果。

**订阅话题**:
- `/cloud_registered` - 输入点云
- `/cloud_baselink` - 地面点云（绿色）
- `/cloud_filter_baselink` - 障碍物聚类（彩色）

**输出信息**:
- 点云数量统计
- 地面/障碍物/其他点的比例
- 性能分析建议

### 3. `test_depth_cluster.launch.py` - 测试启动文件
一键启动完整测试系统。

## 快速开始

### 准备工作

1. 确保已编译项目：
```bash
cd /home/rm/pclfilter
colcon build
source install/setup.bash
```

2. 给脚本添加执行权限：
```bash
chmod +x scripts/test_depth_cluster_demo.py
chmod +x scripts/monitor_depth_cluster.py
```

### 方法一：使用Launch文件（推荐）

启动完整测试系统（包括RViz2可视化）：

```bash
# 基础测试（复杂场景）
ros2 launch pclfilter test_depth_cluster.launch.py

# 简单场景测试
ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=simple

# 动态场景测试（障碍物会移动）
ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=dynamic

# 不启动RViz
ros2 launch pclfilter test_depth_cluster.launch.py use_rviz:=false

# 调整发布频率
ros2 launch pclfilter test_depth_cluster.launch.py publish_rate:=5.0

# 无噪声模式
ros2 launch pclfilter test_depth_cluster.launch.py add_noise:=false
```

### 方法二：分别启动各节点

**终端1**: 启动depth_cluster节点
```bash
ros2 run pclfilter depth_cluster_node
```

**终端2**: 启动测试数据发布器
```bash
ros2 run pclfilter test_depth_cluster_demo.py
```

**终端3** (可选): 启动监控器
```bash
ros2 run pclfilter monitor_depth_cluster.py
```

**终端4** (可选): 启动RViz2可视化
```bash
ros2 run rviz2 rviz2 -d config/simple_rviz2.rviz
```

## RViz2 可视化配置

在RViz2中添加以下显示项：

### 1. 输入点云（原始数据）
- **Topic**: `/cloud_registered`
- **Type**: PointCloud2
- **Color**: 白色或灰色
- **Size**: 0.02

### 2. 地面点云（绿色）
- **Topic**: `/cloud_baselink`
- **Type**: PointCloud2
- **Color Transformer**: RGB8
- **Size**: 0.02

### 3. 障碍物聚类（彩色）
- **Topic**: `/cloud_filter_baselink`
- **Type**: PointCloud2
- **Color Transformer**: RGB8
- **Size**: 0.03

### 4. 参考坐标系
- **Fixed Frame**: `map` 或 `camera_init`
- **添加**: Axes 或 Grid

## 预期效果

### 视觉效果
- **绿色点云**: 地面点（应该是平整的地面）
- **不同颜色的点团**: 不同的障碍物聚类
- 每个障碍物应该有唯一的颜色
- 地面应该与障碍物明确分离

### 控制台输出

**测试发布器输出**:
```
============================================================
Publishing Scene #0 - Type: complex
Ground points: 1600
Total points: 2500
  - Ground: 1600 (应显示为绿色)
  - Obstacles: 900 (6 个聚类)
Expected: 6 colored clusters + green ground
Published successfully!
============================================================
```

**监控器输出**:
```
======================================================================
  Depth Cluster Statistics  
======================================================================
Current Frame:
  Input Points:       2500
  Ground Points:       1580 (绿色 - 地面)
  Cluster Points:       890 (彩色 - 障碍物)
  Other/Filtered:        30
----------------------------------------------------------------------
Average (last 10 frames):
  Input Points:       2500.0
  Ground Points:      1580.0  ( 63.2%)
  Cluster Points:      890.0  ( 35.6%)
  Other/Filtered:       30.0  (  1.2%)
======================================================================
Analysis:
  ✓ Ground detection: 63.2% (normal)
  ✓ Obstacle clustering: 35.6% (normal)
======================================================================
```

## 场景类型说明

### Simple（简单场景）
- 1个地面平面
- 3个障碍物（2个箱子 + 1个圆柱）
- 适合快速验证基本功能

### Complex（复杂场景）- 默认
- 1个地面平面
- 8个障碍物（各种形状和大小）
- 包含散点噪声
- 适合全面测试

### Dynamic（动态场景）
- 与Complex相同
- 部分障碍物会随时间移动
- 适合测试实时性能

## 参数调整

### 发布器参数
```bash
ros2 run pclfilter test_depth_cluster_demo.py \
  --ros-args \
  -p publish_rate:=5.0 \
  -p add_noise:=true \
  -p scene_type:=complex
```

### Depth Cluster 参数（在代码中）
在 `src/main.cpp` 中修改：
```cpp
DepthCluster depthCluster_(
    1,      // vertical_resolution (垂直分辨率)
    0.2,    // horizontal_resolution (水平分辨率)
    32,     // lidar_lines (激光线数)
    20      // cluster_size (最小聚类大小)
);
```

## 故障排查

### 问题1: 没有点云显示
**检查**:
- 话题是否发布: `ros2 topic list`
- 话题数据: `ros2 topic echo /cloud_registered --no-arr`
- TF树: `ros2 run tf2_tools view_frames`

### 问题2: 地面检测不准确
**可能原因**:
- 地面不够平整
- 传感器高度参数不对
- 需要调整地面检测阈值

### 问题3: 聚类效果不好
**可能原因**:
- `cluster_size` 太大或太小
- 分辨率参数不匹配实际数据
- 障碍物太小或太稀疏

### 问题4: 性能问题
**优化建议**:
- 降低发布频率
- 减少点云密度
- 调整聚类参数

## 进阶测试

### 1. 使用真实传感器数据
替换测试发布器为真实的LiDAR或深度相机：
```bash
# 修改launch文件中的remapping
remappings=[
    ('/cloud_registered', '/your_sensor_topic'),
]
```

### 2. 记录和回放数据
```bash
# 记录测试数据
ros2 bag record /cloud_registered /cloud_baselink /cloud_filter_baselink

# 回放数据
ros2 bag play <bag_file>
```

### 3. 性能分析
```bash
# 查看话题频率
ros2 topic hz /cloud_filter_baselink

# 查看节点性能
ros2 run rqt_top rqt_top
```

## 总结

通过这套测试脚本，你可以：
1. ✅ 验证地面提取功能
2. ✅ 验证障碍物聚类功能
3. ✅ 可视化查看效果
4. ✅ 实时监控统计数据
5. ✅ 调试和优化参数

祝测试顺利！🚀
