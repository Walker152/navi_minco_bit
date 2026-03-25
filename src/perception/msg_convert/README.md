# msg_convert

ROS2功能包，用于将Livox自定义点云消息转换为标准的sensor_msgs/PointCloud2格式。

## 功能说明

- **订阅话题**: `/livox/lidar` (livox_ros_driver2/msg/CustomMsg)
- **发布话题**: `/livox/stdpc` (sensor_msgs/msg/PointCloud2)
- **保留字段**: x, y, z, intensity (从reflectivity转换)

## 依赖

- ROS2 (Humble或更高版本)
- rclcpp
- sensor_msgs
- livox_ros_driver2

## 编译

```bash
cd ~/loc_ws
colcon build --packages-select msg_convert
source install/setup.bash
```

## 使用方法

### 方法1: 使用launch文件（推荐）

```bash
ros2 launch msg_convert livox_to_pointcloud2.launch.py
```

自定义话题名称：
```bash
ros2 launch msg_convert livox_to_pointcloud2.launch.py \
  input_topic:=/your_livox_topic \
  output_topic:=/your_output_topic
```

### 方法2: 直接运行节点

```bash
ros2 run msg_convert livox_to_pointcloud2
```

使用自定义参数：
```bash
ros2 run msg_convert livox_to_pointcloud2 \
  --ros-args \
  -p input_topic:=/your_livox_topic \
  -p output_topic:=/your_output_topic \
  -p queue_size:=20
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `input_topic` | string | `/livox/lidar` | 输入的Livox点云话题 |
| `output_topic` | string | `/livox/stdpc` | 输出的标准点云话题 |
| `queue_size` | int | 10 | 订阅和发布的队列大小 |

## 点云字段说明

转换后的PointCloud2包含以下字段：

| 字段名 | 类型 | 说明 |
|--------|------|------|
| x | float32 | X坐标 (米) |
| y | float32 | Y坐标 (米) |
| z | float32 | Z坐标 (米) |
| intensity | float32 | 强度值 (从reflectivity转换，范围0-255) |

**注意**: Livox CustomPoint中的其他字段（offset_time, tag, line）在转换过程中会丢失，但保留了最重要的位置和强度信息。

## 测试

### 查看话题

```bash
# 查看输入话题
ros2 topic echo /livox/lidar

# 查看输出话题
ros2 topic echo /livox/stdpc

# 查看话题信息
ros2 topic info /livox/stdpc
```

### 在RViz2中可视化

```bash
rviz2
```

在RViz2中：
1. 添加 PointCloud2 显示类型
2. 设置话题为 `/livox/stdpc`
3. 设置Fixed Frame为点云的frame_id（通常是雷达坐标系）

### 性能检查

```bash
# 查看发布频率
ros2 topic hz /livox/stdpc

# 查看带宽
ros2 topic bw /livox/stdpc
```

## 示例工作流

```bash
# 终端1: 启动Livox驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2: 启动转换器
ros2 launch msg_convert livox_to_pointcloud2.launch.py

# 终端3: 可视化
rviz2
```

## 故障排除

### 问题1: 找不到livox_ros_driver2消息

**解决方案**: 确保livox_ros_driver2已正确编译和source
```bash
cd ~/loc_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

### 问题2: 没有接收到点云数据

**检查步骤**:
1. 确认Livox驱动是否正在运行
   ```bash
   ros2 topic list | grep livox
   ```
2. 检查输入话题是否有数据
   ```bash
   ros2 topic hz /livox/lidar
   ```
3. 检查话题名称是否匹配

### 问题3: RViz2显示点云为空

**可能原因**:
- Fixed Frame设置不正确
- 点云的frame_id与TF树不匹配
- 尝试设置Fixed Frame为点云消息的header.frame_id

## 性能说明

- 转换过程高效，对CPU占用较低
- 内存开销：每个点约16字节 (4个float32)
- 支持实时转换，延迟在毫秒级别

## 许可证

Apache-2.0
