cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# 自定义消息生成
find_package(rosidl_default_generators REQUIRED)
set(msg_files
    "msg/Test.msg"
)
set(srv_files
    "srv/Test.srv"
)

set(action_files
    "action/Test.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

ament_package()
