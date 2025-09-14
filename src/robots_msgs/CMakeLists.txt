cmake_minimum_required(VERSION 3.8)
project(robots_msgs)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare ROS2 message, service, and action files
set(msg_files
  "msg/Detect.msg"
  "msg/Nav.msg"
  "msg/NavStatus.msg"
  "msg/Radar.msg"
  "msg_send/Chassis.msg"
  "msg_send/MotorSpeed.msg"
)

set(srv_files
  "srv/Trans.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()