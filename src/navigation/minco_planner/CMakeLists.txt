cmake_minimum_required(VERSION 3.8)
project(minco_planner)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_compile_options(-Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_interfaces REQUIRED)

nav2_package()

set(library_name minco_planner)

add_library(${library_name} SHARED
  src/minco_core/minco_planner.cpp
  src/minco_core/astar.cpp
  src/traj_opt/minco_optimizer.cpp
  # MINCO / 轨迹优化核心实现（只保留规划实际用到的最小子集）
  src/utils/banded_system.cpp
  src/utils/lbfgs.cpp
  src/utils/minco.cpp
  src/utils/piece.cpp
  src/utils/trajectory.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${library_name}
  builtin_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  ros_interfaces
)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2_ros
  ros_interfaces
)
ament_export_targets(${library_name})
ament_package()