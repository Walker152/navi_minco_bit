# 使用符号链接，实现launch文件无需编译，直接修改就能用
# 实际上只能解决一部分问题 bug实在找不出来就重启吧
colcon build --symlink-install

#    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
#   "{linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
# sudo apt install ros-humble-spatio-temporal-voxel-layer