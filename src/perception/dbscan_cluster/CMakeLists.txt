cmake_minimum_required(VERSION 3.8)
project(dbscan_cluster)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(DEFINED PYTHON_EXECUTABLE)
  message(STATUS "PYTHON_EXECUTABLE provided by caller: ${PYTHON_EXECUTABLE}")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
  src/dbscan_cluster_node.cpp
  src/ros_interface.cpp
  src/dbscan_cluster.cpp
  src/kalman_tracker.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  ros_interfaces
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
