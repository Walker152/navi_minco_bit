


# Navigation2
source ~/sentry/install/setup.bash
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 2s
# BT-manager
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 0.5s

# tODESK
