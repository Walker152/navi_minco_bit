# 在开头添加CAN初始化
echo "初始化CAN接口..."
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
sleep 1s
echo "等待CAN接口启动..."
# COM:CAN
source ~/2025-sentry-navi/install/setup.bash

# gnome-terminal -- bash -c "ros2 launch yhs_twist_converter com.launch.py; exec bash"

# # MID360
source ~/ws_livox/install/setup.bash
# sleep 1s
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 1s
# sleep 30s
# SLAM
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py ; exec bash"
sleep 1s
# Navigation2
source ~/2025-sentry-navi/install/setup.bash
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 2s
# BT-manager
# gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 1s
