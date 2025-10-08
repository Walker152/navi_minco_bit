# MID360
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 0.5s
# SLAM
source ~/slam_ws/install/setup.bash
gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py ; exec bash"

# Navigation2
source ~/sentry/install/setup.bash
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 0.5s
# BT-manager
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 0.5s
# COM
gnome-terminal -- bash -c "ros2 launch com com.launch.py; exec bash"
sleep 0.5s
