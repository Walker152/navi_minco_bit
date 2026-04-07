#!/bin/bash
# Depth Cluster 测试快速启动脚本

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  Depth Cluster 测试系统${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# 检查是否已source
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo -e "${YELLOW}正在加载ROS2环境...${NC}"
    source /home/rm/pclfilter/install/setup.bash
fi

echo -e "${GREEN}可用的测试模式:${NC}"
echo "  1. 完整测试 (带RViz2) - 简单场景"
echo "  2. 完整测试 (带RViz2) - 复杂场景 [推荐]"
echo "  3. 完整测试 (带RViz2) - 动态场景"
echo "  4. 仅运行节点 (不启动RViz)"
echo "  5. 仅启动监控器"
echo ""
echo -e "${YELLOW}请选择测试模式 (1-5):${NC} "
read choice

case $choice in
    1)
        echo -e "${GREEN}启动简单场景测试...${NC}"
        ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=simple
        ;;
    2)
        echo -e "${GREEN}启动复杂场景测试 (推荐)...${NC}"
        ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=complex
        ;;
    3)
        echo -e "${GREEN}启动动态场景测试...${NC}"
        ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=dynamic
        ;;
    4)
        echo -e "${GREEN}启动节点 (无RViz)...${NC}"
        ros2 launch pclfilter test_depth_cluster.launch.py use_rviz:=false
        ;;
    5)
        echo -e "${GREEN}启动监控器...${NC}"
        echo -e "${YELLOW}请确保depth_cluster_node和测试发布器已在其他终端运行${NC}"
        ros2 run pclfilter monitor_depth_cluster.py
        ;;
    *)
        echo -e "${YELLOW}无效选择，默认启动复杂场景测试...${NC}"
        ros2 launch pclfilter test_depth_cluster.launch.py scene_type:=complex
        ;;
esac
