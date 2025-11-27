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
find_package(livox_ros_driver2 REQUIRED)

# Add executable
add_executable(livox_to_pointcloud2 src/livox_to_pointcloud2.cpp)
ament_target_dependencies(livox_to_pointcloud2
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

# 启用链接时优化
set_target_properties(livox_to_pointcloud2 PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION TRUE
)

# Install executable
install(TARGETS livox_to_pointcloud2
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
