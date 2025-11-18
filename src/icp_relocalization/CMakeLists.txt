cmake_minimum_required(VERSION 3.5)
project(icp_relocalization)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenMP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

# GICP Filter Library
add_library(gicp_filter src/gicp_filter.cpp)
target_include_directories(gicp_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gicp_filter
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# GICP Node Executable (ROS Wrapper)
add_executable(gicp_node src/gicp_node.cpp src/gicp_ros_interface.cpp)
ament_target_dependencies(gicp_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(gicp_node gicp_filter)


# add_executable(transform_publisher src/transform_publisher.cpp)
# ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros)


# add_executable(sac_ia_gicp
#   src/sac_ia_gicp.cpp
# )
# ament_target_dependencies(sac_ia_gicp  rclcpp  std_msgs  PCL  pcl_ros  Eigen3)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS
    gicp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS gicp_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)


ament_package()
