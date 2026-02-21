cmake_minimum_required(VERSION 3.8)
project(lidar_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

option(USE_OPENMP "Enable OpenMP parallelization" ON)
if(USE_OPENMP)
  find_package(OpenMP)
endif()

add_executable(lidar_merger_node
  src/main.cpp
  src/lidar_merger_node.cpp
)

target_include_directories(lidar_merger_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_merger_node Eigen3::Eigen)

if(USE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(lidar_merger_node OpenMP::OpenMP_CXX)
  target_compile_definitions(lidar_merger_node PRIVATE LIDAR_MERGER_USE_OPENMP=1)
endif()

ament_target_dependencies(
  lidar_merger_node
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
  livox_ros_driver2
)

target_compile_features(lidar_merger_node PUBLIC cxx_std_17)

install(TARGETS lidar_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
