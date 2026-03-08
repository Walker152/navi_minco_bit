# RViz2 点云过滤可视化指南

## 当前运行状态

✅ **已启动的节点：**
1. `test_pointcloud_publisher` - 发布测试点云到 `/cloud_registered`
2. `clear_node` - 过滤点云并发布到 `/cloud_filter_baselink`
3. `rviz2` - 可视化工具

## RViz2配置步骤

### 方法1：在RViz2中手动添加显示

**第一步：检查Fixed Frame**
1. 在左侧面板找到 "Global Options"
2. 将 "Fixed Frame" 改为 `camera_init` （因为测试数据使用该坐标系）

**第二步：添加原始点云显示**
1. 点击左下方 "Add" 按钮
2. 选择 "By topic" 标签
3. 找到 `/cloud_registered` 话题
4. 点击下拉框选择 `sensor_msgs/msg/PointCloud2`
5. 点击 "Add" 

应该看到**白色点云**出现（机器人前后方的点）

**第三步：添加过滤后点云显示**
1. 再次点击 "Add" 按钮
2. 选择 "By topic" 标签
3. 找到 `/cloud_filter_baselink` 话题
4. 点击下拉框选择 `sensor_msgs/msg/PointCloud2`
5. 点击 "Add"

应该看到**绿色点云**出现（已移除机器人本体的点）

### 方法2：使用配置文件启动（需要修复）

```bash
cd /home/rm/sentinel-up-gimbal/src/pclfilter
source /opt/ros/humble/setup.bash
rviz2 -d config/simple_rviz2.rviz
```

## 调整显示效果

### 改变点云大小
1. 在左侧 "Displays" 面板找到对应的点云显示
2. 展开它找到 "Size (m)" 或 "Size (Pixels)"
3. 调整数值（推荐0.05-0.1米）

### 改变点云颜色
1. 找到对应的点云显示
2. 在 "Color Transformer" 选择：
   - "Intensity" - 按强度着色
   - "Flat Color" - 单色
3. 调整 "Color" 值

### 改变视角
- **鼠标中键拖动** - 旋转视角
- **鼠标滚轮** - 缩放
- **鼠标右键拖动** - 平移

## 预期显示效果

### 原始点云 (/cloud_registered)
```
总点数: 130个
分布: 
  - 50个点在后方 (y < 0)
  - 30个点在机器人本体 (机器人前方0.3-1.2m)
  - 50个点在前方 (y > 2)
颜色: 白色或彩色梯度
```

### 过滤后点云 (/cloud_filter_baselink)  
```
总点数: 100个 (过滤了30个)
分布:
  - 50个点在后方
  - 0个点在机器人本体 ✓ (已移除)
  - 50个点在前方
颜色: 绿色
```

## 故障排除

### 问题1：看不到点云
**解决方案：**
- 检查 "Fixed Frame" 是否为 `camera_init`
- 检查左下方 "Displays" 是否启用（勾选框应该打钩）
- 确认话题是否发布：`ros2 topic list | grep cloud`

### 问题2：坐标系错误
**解决方案：**
```bash
# 查看可用坐标系
ros2 run tf2_tools view_frames

# 查看坐标变换
ros2 run tf2_ros tf2_echo camera_init base_link
```

### 问题3：点云显示为空
**解决方案：**
- 确认clear_node正在运行：`ros2 node list`
- 确认点云发布者正在运行：`ros2 topic hz /cloud_registered`
- 检查话题质量设置 (QoS)

## 性能优化

如果点云显示卡顿，可以：
1. 减少 "Point Cloud2" 的 "Queue Size"（从10改为5或2）
2. 增加 "Size (Pixels)" 数值
3. 关闭 "Use rainbow" 选项

## 下一步

✅ 点云过滤功能验证完成
✅ RViz2可视化就绪

**可以尝试的功能：**
- [ ] 修改cube.yaml配置，测试不同的过滤区域
- [ ] 启用init_odom()模式，基于里程计清除区域
- [ ] 启动depth_cluster_node进行深度聚类
- [ ] 集成实际LiDAR驱动和定位系统

---

有问题请查看相关launch文件：
- `launch/clear.launch.py` - clear_node启动文件  
- `launch/depth_cluster.launch.py` - depth_cluster启动文件
