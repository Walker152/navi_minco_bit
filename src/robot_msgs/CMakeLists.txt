cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Find required packages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS2 message, service, and action files
set(msg_files
  "msg/Detector.msg"
  "msg/MultiDetect.msg"
  "msg/ScsSignal.msg"
  "msg/Nav.msg"
  "msg/NavStatus.msg"
  "msg/Referee.msg"
  "msg/Detect.msg"
  "msg/EventStatus.msg"
)

# set(srv_files

# )

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()