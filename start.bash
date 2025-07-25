source ./install/setup.bash 
# gnome-terminal -- bash -c "ros2 launch simulation once.launch.py; exec bash"
# sleep 25s # 等待mid360仿真启动

# 真实机器人启动
gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash"
sleep 0.5s
source ~/slam_ws/install/setup.bash
gnome-terminal -- bash -c "ros2 launch fast_lio mapping.launch.py ; exec bash"
sleep 0.5s
# 仿真机器人启动 = 真实机器人上电
# gnome-terminal -- bash -c "ros2 launch simulation gazebo_sim.launch.py; exec bash"
# sleep 2s # 等待gazebo仿真启动


# sleep 1s
# gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard ; exec bash"
sleep 1s
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"

gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.3 0 0 0 /map /camera_init; exec bash"
sleep 0.2s
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0.1 0 0.0 0 0 0 /body /base_link; exec bash"
