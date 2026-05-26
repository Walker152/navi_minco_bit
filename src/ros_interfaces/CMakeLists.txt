cmake_minimum_required(VERSION 3.8)
project(ros_interfaces)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_compile_options(-Werror)
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
  "msg/cluster/DynamicObstacle.msg"
  "msg/cluster/DynamicObstacleArray.msg"
  "msg/minco_traj/PositionCommand.msg"
  "msg/minco_traj/MpcPositionCommand.msg"
  "msg/Behavior.msg"
  "msg/EnemyRobotStatus.msg"
  "msg/RadarInfo.msg"
  "msg/AllyRobotStatus.msg"
  "msg/TeamInformation.msg"
  "msg/GameInfo.msg"
  "msg/SentryInfoOffline.msg"
  "msg/SentryInfoOnline.msg"
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