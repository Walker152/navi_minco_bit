cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
