cmake_minimum_required(VERSION 3.8)
project(bt_manager)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_compile_options(-Werror)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
include_directories(
  include
)

file(GLOB_RECURSE BT_MANAGER_SOURCES CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/action/*.cpp"
  "src/condition/*.cpp"
  "src/utils/*.cpp"
)

add_executable(bt_manager_node 
  ${BT_MANAGER_SOURCES}
)

ament_target_dependencies(bt_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  ros_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  nav2_msgs
  ament_index_cpp
  nav2_behavior_tree
)

install(TARGETS
  bt_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch tree
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
