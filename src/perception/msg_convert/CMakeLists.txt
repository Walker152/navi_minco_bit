cmake_minimum_required(VERSION 3.8)
project(msg_convert)

# 设置C++17标准（支持 [[unlikely]] 等特性）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

  # Release模式的性能优化
  add_compile_options(-O2) # 最高优化级别
  add_compile_options(-march=native) # 针对本地CPU优化（启用SIMD）
  add_compile_options(-flto) # 链接时优化
  add_compile_options(-ffast-math) # 快速数学运算
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(cloud_registered_crop_filter src/cloud_registered_crop_filter.cpp)
ament_target_dependencies(cloud_registered_crop_filter
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
)
target_link_libraries(cloud_registered_crop_filter
  ${PCL_LIBRARIES}
)

set_target_properties(cloud_registered_crop_filter PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION TRUE
)

# Install executable
install(TARGETS cloud_registered_crop_filter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
