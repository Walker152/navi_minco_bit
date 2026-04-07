cmake_minimum_required(VERSION 3.8)
project(pclfilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS
  common
  segmentation
  features
  filters
  io
  visualization
)
find_package(Eigen3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  include
)

link_directories(${PCL_LIBRARY_DIRS})

# depth_cluster library
add_library(depth_cluster include/depth_cluster.cpp)
ament_target_dependencies(depth_cluster
  rclcpp
  sensor_msgs
  pcl_ros
)
target_link_libraries(depth_cluster ${PCL_LIBRARIES})

# clear library
add_library(clear_lib include/clear.cpp)
ament_target_dependencies(clear_lib
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(clear_lib 
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# depth_cluster_node executable
add_executable(depth_cluster_node src/main.cpp)
ament_target_dependencies(depth_cluster_node
  rclcpp
  sensor_msgs
  pcl_ros
)
target_link_libraries(depth_cluster_node
  depth_cluster
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)

# clear_node executable
add_executable(clear_node src/use_clear.cpp)
ament_target_dependencies(clear_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(clear_node
  clear_lib
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS depth_cluster_node clear_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(voxel_integrator src/voxel_integrator.cpp)
ament_target_dependencies(voxel_integrator
  rclcpp
  sensor_msgs
  pcl_conversions
  octomap
  octomap_msgs
)

install(TARGETS voxel_integrator
  DESTINATION lib/${PROJECT_NAME})