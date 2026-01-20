cmake_minimum_required(VERSION 3.8)
project(minco_planner)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_compile_options(-Werror -Wno-error=unused-variable -Wno-error=unused-parameter)
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
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(PCL REQUIRED)

nav2_package()

set(library_name minco_planner)

file(GLOB UTILS_SOURCES "src/utils/*.cpp")
file(GLOB SMALL_ROG_MAP_SOURCES "src/small_rog_map/*.cpp")

add_library(${library_name} SHARED
  src/minco_core/minco_planner.cpp
  src/minco_core/visualizer.cpp
  src/minco_core/astar.cpp
  src/minco_core/corridor_generator.cpp
  src/traj_opt/backup_traj_optimizer_s4.cpp
  src/traj_opt/minco_optimizer.cpp
  # MINCO / 轨迹优化核心实现（只保留规划实际用到的最小子集）
  ${UTILS_SOURCES}
  ${SMALL_ROG_MAP_SOURCES}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
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
  sensor_msgs
  tf2_ros
  ros_interfaces
)

target_link_libraries(${library_name}
  ${PCL_LIBRARIES}
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
  sensor_msgs
  tf2_ros
  ros_interfaces
)
ament_export_targets(${library_name})
ament_package()