## !/bin/bash
cd ~/2025-sentry-navi
source ./install/setup.bash

# MID360 
# gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"
# sleep 3

# # SLAM
# gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py; exec bash"
# sleep 1

# Navigation2
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
# sleep 7

## ICP
gnome-terminal -- bash -c "ros2 launch icp_relocalization gicp_relocalization.launch.py; exec bash"
# sleep 1


## Cluster
# gnome-terminal -- bash -c "ros2 launch msg_convert cloud_registered_crop_filter.launch.py; exec bash"
# sleep 1

# ## Decision
# gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
# sleep 1

# # Communication
# gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"

# Rosbag record
# gnome-terminal -- bash -c "mkdir -p ~/rosbag && ros2 bag record -a -o ~/rosbag/$(date +%Y%m%d_%H%M%S); exec bash"

# Foxglove
# gnome-terminal -- bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; exec bash"