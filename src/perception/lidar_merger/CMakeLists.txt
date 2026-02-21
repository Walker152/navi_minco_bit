cmake_minimum_required(VERSION 3.8)
project(lidar_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)

set(_perf_compile_opts -O3 -march=native)

add_library(lidar_merger_component SHARED
  src/lidar_merger_node.cpp
)

target_include_directories(lidar_merger_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(lidar_merger_component
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

target_compile_options(lidar_merger_component PRIVATE ${_perf_compile_opts})

ament_target_dependencies(
  lidar_merger_component
  rclcpp
  rclcpp_components
  message_filters
  livox_ros_driver2
)

rclcpp_components_register_nodes(lidar_merger_component
  "LidarMergerNode"
)
target_compile_features(lidar_merger_component PUBLIC cxx_std_17)

install(TARGETS
  lidar_merger_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
