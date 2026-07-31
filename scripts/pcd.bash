#!/bin/bash
cd ~/2025-sentry-navi
source ./install/setup.bash

gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_mixed_MID360.launch.py; exec bash"
# gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360.launch.py; exec bash"
sleep 5s

gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py; exec bash"