cmake_minimum_required(VERSION 3.5)
project(communication)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXE_NAME sentry)

# 源文件列表
set(source
  src/behavier_tree_com.cpp
  src/main.cpp
  src/communication.cpp
)

# 查找 ROS2 和系统依赖包
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robots_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(Libevent REQUIRED)
find_package(fmt REQUIRED)

# include 目录
include_directories(
  include
  src/communication/RM_encoder
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  my-utils/MyUtils
  3rdparty/serial/include # 添加serial头文件路径
)

file(GLOB_RECURSE DIR_SRCS src/communication/RM_encoder/*.cpp)

add_executable(${EXE_NAME}
  ${source}
  ${DIR_SRCS}
)

if(${PROJECT_NAME}_EXPORTED_TARGETS)
  add_dependencies(${EXE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

# 添加子目录（3rdparty库）
add_subdirectory(3rdparty/plog-1.1.10)
add_subdirectory(3rdparty/sheriffos)
add_subdirectory(3rdparty/serial)

# 链接库
target_link_libraries(${EXE_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
  libevent::core
  libevent::pthreads
  fmt::fmt
  plog
  sheriff_os
  serial
)

ament_target_dependencies(${EXE_NAME}
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  robots_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS ${EXE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# 声明为ament包
ament_package()
