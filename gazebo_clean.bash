#!/bin/bash

# 更安全的 Gazebo 清理脚本
echo "开始清理 Gazebo 残留进程..."

# 1. 更精确地终止进程
pkill -9 gazebo
pkill -9 gzserver
pkill -9 gzclient

# 仅终止当前用户的 ROS 进程
if pgrep -u $(id -u) ros2; then
    pkill -9 -u $(id -u) ros2
fi

if pgrep -u $(id -u) rosmaster; then
    pkill -9 -u $(id -u) rosmaster
fi

# 仅终止 spawn_entity.py 进程
pkill -9 -f "python.*spawn_entity.py"

# 2. 更安全的端口释放
for port in 11345; do  # 只处理 Gazebo 默认端口
    lsof -i :$port | awk 'NR!=1 {print $2}' | xargs -r kill -9 2>/dev/null
done

# 3. 选择性清理临时文件
# 仅删除 Gazebo 相关的临时文件
rm -rf /tmp/gazebo-*
rm -rf /tmp/ros*

echo "Gazebo 和相关进程已安全清理"