#!/bin/bash
cd ~/2027-sentry-navi
source ./install/setup.bash

# PTP Sync
# gnome-terminal -- bash -c "sudo ./scripts/ptp_sync.bash; exec bash"
# sleep 8

# Driver + SLAM
# gnome-terminal -- bash -c "ros2 launch point_lio mixed_livox_pointlio_intra_process.launch.py; exec bash"
gnome-terminal -- bash -c "ros2 launch point_lio single_livox_pointlio_intra_process.launch.py; exec bash"
sleep 3

# ICP
# gnome-terminal -- bash c "ros2 launch icp_relocalization gicp_relocalization.launch.py; exec bash"
# sleep 1

# Cloud Crop Filter
gnome-terminal -- bash -c "ros2 launch msg_convert cloud_registered_crop_filter.launch.py; exec bash"
sleep 1

# Navigation2
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
# gnome-terminal -- bash -c "ros2 launch navi2 slam.launch.py; exec bash"
sleep 5

# Decision
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
# sleep 3

# Communication
gnome-terminal -- bash -c "ros2 launch communication com.launch.py; exec bash"

# Rosbag record
ROSBAG_TOPICS=(
  /tf
  /tf_static
  /aft_mapped_to_init
  /global_costmap/costmap_raw
  /opt_path
  /opt_path_vis
  /astar_path_vis
  /recover_goal
  /mpc_predict_path
  /cmd_vel_mpc
  /sentry/behaivor_send
  /sentry/area_markers
  /cloud_registered_full
  /rog_map/occupied
  /rog_map/layer_value
  /rog_map/field
  /minco_control_points_vis
  /rog_map/layer_height_delta
  # /livox/lidar
  # /livox/imu_192_168_1_135
)
# gnome-terminal -- bash -c "mkdir -p ~/rosbag && ros2 bag record -o ~/rosbag/$(date +%Y%m%d_%H%M%S) ${ROSBAG_TOPICS[*]}; exec bash"

# Foxglove
# gnome-terminal -- bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; exec bash"
