source ./install/setup.bash 
# gnome-terminal -- bash -c "ros2 launch simulation once.launch.py; exec bash"
# sleep 25s # 等待mid360仿真启动
# 仿真机器人启动 = 真实机器人上电
gnome-terminal -- bash -c "ros2 launch simulation car.launch.py; exec bash"
sleep 5s # 等待gazebo仿真启动
# gnome-terminal -- bash -c "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True; exec bash"
# sleep 1s
gnome-terminal -- bash -c "ros2 launch navigate nav2.launch.py ; exec bash"
# gnome-terminal -- bash -c "ros2 launch navigate reference.launch.py ; exec bash"
sleep 1s
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.0 0 0 0 /map /base_link; exec bash"
