# # MID360
source ~/ws_livox/install/setup.bash
# sleep 1s
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 1s
# sleep 30s
# SLAM
source ~/slam_ws/install/setup.bash
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py ; exec bash"
# Navigation2
source ~/sentry/install/setup.bash
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"

sleep 1s
# BT-manager
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 1s
# COM
gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"
# sleep 3s

# gnome-terminal -- bash -c "cd /home/rm/rosbag && ros2 bag record --all; exec bash"