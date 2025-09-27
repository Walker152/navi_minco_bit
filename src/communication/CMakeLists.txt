cmake_minimum_required(VERSION 3.8)
project(com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 开了就全是报错，谁研究的communication呢
# add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(fmt REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVENT REQUIRED libevent)

###########
## Build ##
###########

include_directories(
  include
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/${PROJECT_NAME}_handle.cpp
  src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  robot_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_core
  fmt::fmt
  tf2::tf2
  tf2_ros::tf2_ros
  event
  pthread
  ${LIBEVENT_LIBRARIES}
  event_pthreads 
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
