cmake_minimum_required(VERSION 3.8)
project(local_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/super_control.cpp
  src/pid.cpp
  src/mpc.cpp
)

pluginlib_export_plugin_description_file(nav2_core local_planner.xml)

ament_auto_package()
