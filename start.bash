#!/bin/bash
cd ~/2025-sentry-navi
source ./install/setup.bash

# PTP Sync
# gnome-terminal -- bash -c "sudo ./scripts/ptp_sync.bash; exec bash"
# sleep 2

# # MID360 
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"
<<<<<<< HEAD
# sleep 1
=======
sleep 1
>>>>>>> develop_reset

# # Lidar Merger
# gnome-terminal -- bash -c "ros2 launch lidar_merger dual_lidar_merger.launch.py; exec bash"
# sleep 1

# # # SLAM
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py; exec bash"
# # sleep 1

# # # Cloud Crop Filter
gnome-terminal -- bash -c "ros2 launch msg_convert cloud_registered_crop_filter.launch.py; exec bash"
# # sleep 1

# # # ICP
# gnome-terminal -- bash -c "ros2 launch icp_relocalization gicp_relocalization.launch.py; exec bash"
# # sleep 1

# Navigation2
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 5

# # Decision
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 1

# Communication
gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"

# Rosbag record
# gnome-terminal -- bash -c "mkdir -p ~/roslaunchbag && ros2 bag record -a -o ~/rosbag/$(date +%Y%m%d_%H%M%S); exec bash"

# Foxglove
# gnome-terminal -- bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; exec bash"