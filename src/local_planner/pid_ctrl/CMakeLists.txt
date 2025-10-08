# cmake_minimum_required(VERSION 3.10)
# project(dreamchaser_ctrl)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   set(CMAKE_CXX_STANDARD 17)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
#   add_compile_options(-Wall -Wextra -Wpedantic)
#   set(CMAKE_BUILD_TYPE "Release")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
# endif()

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # include
# include_directories(
#   include
# )
# # 似乎只需要这个就行
# find_package(nav2_core REQUIRED)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)

# # ros dependencies
# set(dependencies
#   rclcpp
#   nav_msgs
#   nav2_core
#   geometry_msgs
#   pluginlib
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   visualization_msgs
# )

# add_library(${PROJECT_NAME}_plugin SHARED 
#   src/dreamchaser_ctrl.cpp
#   src/pid.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_plugin ${dependencies})
# target_link_libraries(${PROJECT_NAME}_plugin 

# )

# install(TARGETS ${PROJECT_NAME}_plugin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# # 安装头文件到指定目录
# install(DIRECTORY include/
#  DESTINATION include
# )

# pluginlib_export_plugin_description_file(nav2_core dreamchaser_ctrl.xml)
# # 导出插件描述文件

# ament_package()
