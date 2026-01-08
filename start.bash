# # MID360
source ~/ws_livox/install/setup.bash

gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 1s

# SLAM
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py ; exec bash"
sleep 1s

# Navigation2
source ~/2025-sentry-navi/install/setup.bash

gnome-terminal -- bash -c "ros2 launch icp_relocalization gicp_relocalization.launch.py ; exec bash"
sleep 5s

gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 2s

# Behavior tree manager
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 1s

# COM
gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"

# Rosbag record (optional)
# gnome-terminal -- bash -c "cd /home/rm/rosbag && ros2 bag record -a ;exec bash"