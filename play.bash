
#param_client_test
# source ~/ws01_plumbing_ljy/install/setup.bash
# gnome-terminal -- bash -c "ros2 run param study02_param_client ; exec bash"
# sleep 2s
# Navigation2
source ./install/setup.bash
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 0.5s
# BT-manager
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 0.5s
# commmunication
gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"
sleep 0.5s

