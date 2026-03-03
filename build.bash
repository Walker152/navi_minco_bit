source /opt/ros/humble/setup.bash
source ../ws_livox/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --event-handlers console_direct+