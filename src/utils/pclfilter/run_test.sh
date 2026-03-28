#!/bin/bash

# 设置工作目录
cd /home/rm/pclfilter

# source ROS2环境
source /opt/ros/humble/setup.bash

# 手动设置AMENT_PREFIX_PATH (解决colcon setup.bash的bug)
export AMENT_PREFIX_PATH=/home/rm/pclfilter/install/pclfilter:$AMENT_PREFIX_PATH

# source本地包
source install/local_setup.bash 2>/dev/null || true

# 检查包是否可用
if ros2 pkg list | grep -q pclfilter; then
    echo "✓ pclfilter包已找到"
else
    echo "✗ 错误: pclfilter包未找到"
    exit 1
fi

# 启动launch文件
echo "启动depth_cluster测试系统..."
ros2 launch launch/test_depth_cluster.launch.py scene_type:=${1:-slope}
