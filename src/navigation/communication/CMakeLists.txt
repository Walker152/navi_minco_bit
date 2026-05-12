cmake_minimum_required(VERSION 3.8)
project(communication)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_compile_options(-Werror)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_interfaces REQUIRED)
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
  src/com.cpp
  src/com_interface_ros.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  ros_interfaces
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  # mars_quadrotor_msgs
)
target_link_libraries(${PROJECT_NAME}_core
  fmt::fmt
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

# If requested, add COM_DEBUG definition to targets so '#ifdef COM_DEBUG' blocks are enabled
if(COM_DEBUG)
  message(STATUS "COM debug enabled: adding COM_DEBUG define to targets")
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE COM_DEBUG)
  target_compile_definitions(${PROJECT_NAME} PRIVATE COM_DEBUG)
endif()

############
## Tests  ##
############

# if(BUILD_TESTING)
#   # Test executable (manual test harness)
#   add_executable(${PROJECT_NAME}_test
#     src/communication_test.cpp
#   )
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_core)
#   if(COM_DEBUG)
#     target_compile_definitions(${PROJECT_NAME}_test PRIVATE COM_DEBUG)
#   endif()

#   # Register ament/ctest to run the smoke test
#   ament_add_test(${PROJECT_NAME}_smoke
#     COMMAND $<TARGET_FILE:${PROJECT_NAME}_test>
#     TIMEOUT 30
#     GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
# endif()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  DESTINATION lib/${PROJECT_NAME}
)
if(TARGET ${PROJECT_NAME}_test)
  install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
