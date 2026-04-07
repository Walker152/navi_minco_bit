#!/bin/bash
# 使用调试参数启动节点的脚本
# Script to launch node with debug parameters

set -e

cd /home/rm/pclfilter

# 使用源码环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 清理旧进程
echo "🧹 清理旧进程..."
killall -9 depth_cluster_node 2>/dev/null || true
sleep 1

# 启动 TF 发布器
echo "📍 启动 TF 发布器..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init > /dev/null 2>&1 &
TF_PID=$!
sleep 1

# 启动深度聚类节点
echo "🚀 启动深度聚类节点 (带参数)..."
ros2 run pclfilter depth_cluster_node \
  --ros-args \
  -p normal_estimation_radius:=0.5 \
  -p min_neighbors:=5 \
  -p max_slope_angle_degrees:=30.0 \
  -p verbose:=true \
  -p print_interval:=1 \
  > /tmp/depth_cluster_debug.log 2>&1 &
NODE_PID=$!
sleep 2

# 启动 RViz
echo "📊 启动 RViz2..."
ros2 run rviz2 rviz2 -d install/pclfilter/share/pclfilter/config/depth_cluster_test.rviz > /dev/null 2>&1 &
RVIZ_PID=$!

echo ""
echo "✅ 所有进程已启动！"
echo "📋 进程 PID:"
echo "   TF Publisher: $TF_PID"
echo "   Depth Cluster Node: $NODE_PID"
echo "   RViz2: $RVIZ_PID"
echo ""
echo "📝 日志文件: /tmp/depth_cluster_debug.log"
echo ""
echo "💡 快速查看日志:"
echo "   tail -f /tmp/depth_cluster_debug.log"
echo ""
echo "🛑 停止所有进程:"
echo "   killall -9 depth_cluster_node rviz2 static_transform_publisher 2>/dev/null"
echo ""

# 实时显示日志
sleep 2
echo "📌 实时日志输出 (按 Ctrl+C 停止):"
echo "---"
tail -f /tmp/depth_cluster_debug.log
