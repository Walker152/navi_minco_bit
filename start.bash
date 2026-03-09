# !/bin/bash 
# MID360
source ~/ws_livox/install/setup.bash
# sleep 1s
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 1s

source ~/2025-sentry-navi/install/setup.bash

# SLAM
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py ; exec bash"
sleep 1s

# Navigation2
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 4s

# gnome-terminal -- bash -c "ros2 launch icp_relocalization gicp_relocalization.launch.py ; exec bash"

# BT-manager
# gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"

# COM
# gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"

# gnome-terminal -- bash -c "cd /home/rm/rosbag && ros2 bag record -a ;exec bash"
