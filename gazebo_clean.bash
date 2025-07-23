#!/bin/bash

# 终止所有相关进程
pkill -9 -f 'gazebo'
pkill -9 -f 'gzserver'
pkill -9 -f 'gzclient'
pkill -9 -f 'ros2 master'
pkill -9 -f 'rosmaster'
pkill -9 -f 'spawn_entity'

# 释放端口
for port in 11345 11311; do
  lsof -i :$port | awk 'NR!=1 {print $2}' | xargs kill -9 2>/dev/null
done

# 删除残留文件
rm -rf ~/.ros/log/* /tmp/gazebo* /tmp/ros*
# rm -rf ~/.gazebo/* ~/.ros/log/* /tmp/gazebo* /tmp/ros*

echo "Gazebo和相关进程已强制清除"
