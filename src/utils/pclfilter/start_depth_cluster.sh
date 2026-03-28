#!/bin/bash

# Depth Cluster 节点启动脚本

cd /home/rm/pclfilter
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "========================================" 
echo "启动 Depth Cluster 节点"
echo "========================================"
echo ""

# 启动静态TF
echo "[1/2] 启动 TF 发布器: map -> camera_init"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init > /dev/null 2>&1 &
TF_PID=$!
sleep 1

# 启动 depth_cluster 节点
echo "[2/2] 启动 depth_cluster 节点"
echo "  订阅话题: /gicp_map"
echo "  输出话题: /cloud_baselink (地面点云), /cloud_filter_baselink (障碍物)"
ros2 run pclfilter depth_cluster_node \
  --ros-args \
  -p input_cloud_topic:=/gicp_map \
  -p output_ground_topic:=/cloud_baselink \
  -p output_obstacles_topic:=/cloud_filter_baselink \
  -p max_slope_angle:=30.0 \
  -p normal_estimation_radius:=0.5 > /tmp/depth_cluster.log 2>&1 &
NODE_PID=$!

echo ""
echo "========================================" 
echo "节点已启动"
echo "========================================" 
echo ""
echo "进程 PID:"
echo "  TF Publisher: $TF_PID"
echo "  Depth Cluster: $NODE_PID"
echo ""
echo "查看实时日志:"
echo "  tail -f /tmp/depth_cluster.log"
echo ""
echo "查看话题列表:"
echo "  ros2 topic list"
echo ""
echo "查看点云数据:"
echo "  ros2 topic echo /cloud_baselink"
echo ""
echo "停止所有节点:"
echo "  killall -9 depth_cluster_node static_transform_publisher"
echo ""
