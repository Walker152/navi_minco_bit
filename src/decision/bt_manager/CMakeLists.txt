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

add_executable(bt_manager_node 
  src/bt_manager.cpp 
  src/ros_interface.cpp 
  src/action/auto_actions.cpp 
  src/condition/auto_conditions.cpp
  src/action/change_stance_action.cpp
  src/condition/change_stance_condition.cpp
  src/utils/nav_zone.cpp
  src/utils/tf_utils.cpp
  src/main.cpp
  )

add_executable(event_test
  src/event_status_pub_test.cpp
)

ament_target_dependencies(event_test
  rclcpp
  std_msgs
  ros_interfaces
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_behavior_tree
)

ament_target_dependencies(bt_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
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
  event_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch tree
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
