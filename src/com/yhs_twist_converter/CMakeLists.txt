cmake_minimum_required(VERSION 3.8)
project(yhs_twist_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yhs_can_interfaces REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(twist_to_ctrl_cmd_node
  src/twist_to_ctrl_cmd_node.cpp
)

# Specify dependencies
ament_target_dependencies(twist_to_ctrl_cmd_node
  rclcpp
  geometry_msgs
  yhs_can_interfaces
  mars_quadrotor_msgs
)

# Install targets
install(TARGETS
  twist_to_ctrl_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()