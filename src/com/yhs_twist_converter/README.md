# yhs_twist_converter

这个 ROS2 功能包提供了将标准的 `geometry_msgs/Twist` 消息转换为自定义的 `yhs_can_interfaces/CtrlCmd` 消息的功能。

## 功能描述

- **输入**: `geometry_msgs/Twist` 消息（通常来自遥控器、键盘控制或路径规划器）
- **输出**: `yhs_can_interfaces/CtrlCmd` 消息（用于 YHS 底盘控制）

## 消息映射关系

| geometry_msgs/Twist | yhs_can_interfaces/CtrlCmd | 描述 |
|-------------------|------------------------|------|
| `linear.x` | `ctrl_cmd_x_linear` | 前后方向线性速度 (m/s) |
| `linear.y` | `ctrl_cmd_y_linear` | 左右方向线性速度 (m/s) |
| `angular.z` | `ctrl_cmd_z_angular` | 绕Z轴角速度 (rad/s) |
| - | `ctrl_cmd_gear` | 档位（通过参数设置） |

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `default_gear` | int | 1 | 默认档位 (0=倒档, 1=空档, 2=前进档) |
| `max_linear_x` | double | 2.0 | X方向最大线性速度限制 (m/s) |
| `max_linear_y` | double | 2.0 | Y方向最大线性速度限制 (m/s) |
| `max_angular_z` | double | 2.0 | Z轴最大角速度限制 (rad/s) |

## 话题接口

### 订阅的话题
- `/cmd_vel` (geometry_msgs/Twist) - 输入的速度命令

### 发布的话题
- `/ctrl_cmd` (yhs_can_interfaces/CtrlCmd) - 转换后的控制命令

## 使用方法

### 1. 基本启动
```bash
# 编译功能包
cd /home/rm/FW-mid-pro
colcon build --packages-select yhs_twist_converter
source install/setup.bash

# 启动转换节点
ros2 launch yhs_twist_converter twist_converter.launch.py
```

### 2. 自定义参数启动
```bash
# 启动时修改参数
ros2 launch yhs_twist_converter twist_converter.launch.py \
    default_gear:=2 \
    max_linear_x:=1.5 \
    max_linear_y:=1.0 \
    max_angular_z:=1.0
```

### 3. 直接运行节点
```bash
ros2 run yhs_twist_converter twist_to_ctrl_cmd_node \
    --ros-args \
    -p default_gear:=2 \
    -p max_linear_x:=1.5
```

## 完整控制链路示例

```bash
# 终端 1: 启动转换器
ros2 launch yhs_twist_converter twist_converter.launch.py

# 终端 2: 启动底盘控制（如果有的话）
ros2 launch yhs_can_control yhs_can_control.launch.py

# 终端 3: 发送测试命令
# 前进 0.5 m/s
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转 0.3 rad/s
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"
```

## 调试与监控

```bash
# 查看转换结果
ros2 topic echo /ctrl_cmd

# 监控输入
ros2 topic echo /cmd_vel

# 查看话题连接
ros2 node info /twist_to_ctrl_cmd_node

# 检查参数
ros2 param list /twist_to_ctrl_cmd_node
ros2 param get /twist_to_ctrl_cmd_node default_gear
```

## 与键盘控制结合使用

```bash
# 安装键盘控制包（如果没有的话）
sudo apt install ros-humble-teleop-twist-keyboard

# 终端 1: 启动转换器
ros2 launch yhs_twist_converter twist_converter.launch.py

# 终端 2: 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 现在可以用键盘控制机器人了！
```

## 故障排查

1. **编译错误**: 确保 `yhs_can_interfaces` 包已经编译
2. **话题不见**: 检查节点是否正常启动 `ros2 node list`
3. **速度限制**: 检查参数设置是否合理
4. **消息格式**: 确认输入的 Twist 消息格式正确

## 集成建议

这个转换器可以很好地与以下组件集成：
- 键盘/游戏手柄控制 (`teleop_twist_keyboard`, `joy`)
- 导航系统 (`nav2`)
- 路径规划器
- 自动驾驶算法

只需要确保这些组件发布标准的 `geometry_msgs/Twist` 消息到 `/cmd_vel` 话题即可。