# RViz2 点云显示故障排除指南

## 🔍 问题诊断

### 问题：在RViz2中看不到点云

**首先验证数据是否在流动：**

```bash
# 1. 检查节点是否运行
ros2 node list

# 应该看到:
# /clear_node
# /rviz
# /test_pointcloud_publisher

# 2. 检查话题是否发布
ros2 topic list | grep cloud

# 应该看到:
# /cloud_filter_baselink
# /cloud_registered

# 3. 检查数据频率
timeout 5 ros2 topic hz /cloud_registered

# 应该显示 ~1.0 Hz (每秒1条消息)
```

---

## ✅ 解决方案 

### **步骤1：关闭并重新启动RViz2**

```bash
# 1. 杀死所有RViz2进程
pkill -f rviz2

# 2. 清空RViz2配置缓存
rm -rf ~/.config/rviz2/

# 3. 重新启动RViz2
source /opt/ros/humble/setup.bash
rviz2
```

### **步骤2：在RViz2中手动配置**

#### **① 修改Fixed Frame**

1. 打开RViz2窗口
2. 在**左侧面板**找到 "Global Options"（最上面）
3. 找到 "Fixed Frame" 下拉框（默认可能是"map"）
4. **改为 "camera_init"** ⚠️ 重要！
5. 观察画面中心是否有变化

#### **② 添加原始点云显示**

1. 点击左下方 **"Add"** 按钮（绿色+号）
2. 在弹出的对话框中，选择 **"By topic"** 标签（不是 "By display type"）
3. 在列表中找到：
   ```
   /cloud_registered [sensor_msgs/msg/PointCloud2]
   ```
4. 点击它
5. 点击下方 **"Add"** 按钮
6. **✓ 应该看到白色点云出现**

#### **③ 添加过滤后的点云**

1. 再次点击左下方 **"Add"** 按钮
2. 选择 **"By topic"** 标签
3. 找到：
   ```
   /cloud_filter_baselink [sensor_msgs/msg/PointCloud2]
   ```
4. 点击它
5. 点击下方 **"Add"** 按钮
6. **✓ 应该看到绿色点云出现**（比白色点云少）

---

## 🎯 如果还是看不到

### 检查1：点云话题是否有数据

```bash
# 确认发布器正在运行
ps aux | grep test_pointcloud_publisher

# 查看点云详细信息
ros2 topic info /cloud_registered

# 应该显示:
# Type: sensor_msgs/msg/PointCloud2
# Publisher count: 1
# Subscription count: 1 (RViz2)
```

### 检查2：RViz2是否能找到话题

如果 "By topic" 选项中找不到话题：
1. 重新启动RViz2
2. 确保RViz2进程已启动：`ps aux | grep rviz2`
3. 检查ROS2环境：
   ```bash
   echo $ROS_DOMAIN_ID
   # 应该是空的或0 (默认值)
   ```

### 检查3：坐标系问题

```bash
# 查看RViz2能访问的坐标系
ros2 run tf2_tools view_frames

# 应该至少有 "camera_init" 坐标系
# 查看可用的TF
ros2 run tf2_ros tf2_echo camera_init camera_init
```

---

## 💡 快速修复

**如果上述都检查过还是有问题，试试这个：**

```bash
# 完整重启流程
cd /home/rm/sentinel-up-gimbal/src/pclfilter

# 1. 杀死所有相关进程
pkill -f clear_node
pkill -f test_pointcloud_publisher  
pkill -f rviz2

# 等待2秒
sleep 2

# 2. 重新启动
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# 3. 在3个不同的终端运行：
# 终端1:
python3 scripts/test_pointcloud_publisher.py

# 终端2:  
./install/pclfilter/lib/pclfilter/clear_node

# 终端3:
rviz2
```

---

## 🔧 RViz2配置保存

配置成功后可以保存设置：

1. 在RViz2顶部菜单 → File → Save Config
2. 给配置起名，比如 "pointcloud_filter"
3. 下次启动时：
   ```bash
   rviz2 -d ~/.rviz2/pointcloud_filter.rviz
   ```

---

## 📊 预期效果

如果配置成功，应该看到：

| 指标 | 预期值 |
|------|--------|
| **原始点云** | 130个白色点 |
| **过滤点云** | 100个绿色点 |
| **点云更新频率** | ~1 Hz |
| **坐标系** | camera_init |
| **视角** | 能旋转、缩放、平移 |

---

## 🚨 最后的终极技巧

如果RViz2完全无法显示点云，可以尝试使用PCL的可视化工具：

```bash
# 保存点云为PCD文件并可视化
cd /home/rm/sentinel-up-gimbal/src/pclfilter

# 创建点云保存脚本（如需要）
# pcl_viewer output.pcd
```

---

## 需要帮助？

请收集以下信息并提供：

```bash
# 1. ROS节点状态
ros2 node list > /tmp/nodes.txt

# 2. 话题列表  
ros2 topic list > /tmp/topics.txt

# 3. 点云数据检查
timeout 2 ros2 topic echo /cloud_registered > /tmp/cloud_data.txt

# 4. RViz2日志
echo "检查终端输出中是否有错误信息"
```

提供这些信息会帮助快速定位问题！
