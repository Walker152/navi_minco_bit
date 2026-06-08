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
find_package(rog_map REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

nav2_package()

set(library_name minco_planner)

file(GLOB UTILS_SOURCES "src/utils/*.cpp")
file(GLOB SMALL_ROG_MAP_SOURCES "src/small_rog_map/*.cpp")
file(GLOB SMAC_SEARCH_SOURCES "src/smac_search/*.cpp")
set(MINCO_CORE_SOURCES
  src/minco_core/astar.cpp
  src/minco_core/corridor_generator.cpp
  src/minco_core/components/global_path_searcher.cpp
  src/minco_core/components/local_path_processor.cpp
  src/minco_core/components/map_query_adapters.cpp
  src/minco_core/components/planner_mode_context.cpp
  src/minco_core/components/recovery_behaivor.cpp
  src/minco_core/components/trajectory_safety_checker.cpp
  src/minco_core/minco_fsm.cpp
  src/minco_core/minco_planner.cpp
  src/minco_core/minco_utils.cpp
  src/minco_core/visualizer.cpp
)
file(GLOB TRAJ_OPT_SOURCES "src/traj_opt/*.cpp")

add_library(${library_name} SHARED
  ${UTILS_SOURCES}
  ${SMALL_ROG_MAP_SOURCES}
  ${SMAC_SEARCH_SOURCES}
  ${MINCO_CORE_SOURCES}
  ${TRAJ_OPT_SOURCES}
)

target_compile_definitions(${library_name}
PUBLIC
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
  ORIGIN_AT_CORNER
  USE_ROS2
)

target_compile_options(${library_name}
PRIVATE
  -Wno-error=unused-parameter
  -Wno-error=unused-function
  -Wno-error=ignored-qualifiers
)

target_include_directories(${library_name} 
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
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
  rog_map
)

target_link_libraries(${library_name}
  OpenMP::OpenMP_CXX
  ${PCL_LIBRARIES}
  yaml-cpp
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
  rog_map
)
ament_export_targets(${library_name})
ament_package()
