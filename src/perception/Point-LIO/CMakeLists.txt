cmake_minimum_required(VERSION 3.8)
project(point_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   set(CMAKE_CXX_STANDARD 17)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   # set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
#   # add_compile_options(-Wall -Wextra -Wpedantic)
#   set(CMAKE_BUILD_TYPE "Release")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
# endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# add_definitions(-Wall -Werror)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Fix glog include issue for glog >= 0.7.0
add_definitions(-DGLOG_USE_GLOG_EXPORT)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")

  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenMP QUIET)
find_package(PythonLibs REQUIRED) # Consider using Pybind11 for ROS2
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

set(POINT_LIO_MAPPING_SOURCES
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
)

add_library(point_lio_component SHARED
  ${POINT_LIO_MAPPING_SOURCES}
)

ament_target_dependencies(point_lio_component
  rclcpp
  rclcpp_components
  rclpy
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(point_lio_component ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(point_lio_component PRIVATE ${PYTHON_INCLUDE_DIRS})
rclcpp_components_register_nodes(point_lio_component "point_lio::LaserMappingNode")

# Declare a ROS2 executable
add_executable(pointlio_mapping
  ${POINT_LIO_MAPPING_SOURCES}
)
target_compile_definitions(pointlio_mapping PRIVATE POINT_LIO_BUILD_MAIN)

ament_target_dependencies(pointlio_mapping
  rclcpp
  rclcpp_components
  rclpy
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

# Install the executable
install(TARGETS
  point_lio_component
  pointlio_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(time_batching_test test/time_batching_test.cpp)
  target_include_directories(time_batching_test PRIVATE include)

  ament_add_gtest(sensor_time_rate_limiter_test test/sensor_time_rate_limiter_test.cpp)
  target_include_directories(sensor_time_rate_limiter_test PRIVATE include)
endif()

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclpy
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()
