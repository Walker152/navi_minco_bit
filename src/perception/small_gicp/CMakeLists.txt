# cmake_minimum_required(VERSION 3.11)
# project(small_gicp)

# ## Use C++14
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# ## enforcing cleaner code.
# # add_definitions(-Wall -Werror)

# ## Export compile commands for clangd
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# #######################
# ## Find dependencies ##
# #######################
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(OpenMP REQUIRED)
# find_package(small_gicp REQUIRED)
# find_package(PCL REQUIRED)

# include_directories(
#   include
#   ${PCL_INCLUDE_DIRS}
# )

# ###########
# ## Build ##
# ###########

# add_library(${PROJECT_NAME}_core SHARED
#   src/small_gicp.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_core
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   geometry_msgs
#   tf2
#   tf2_eigen
#   tf2_ros
#   pcl_conversions
# )

# target_link_libraries(${PROJECT_NAME}_core
#   OpenMP::OpenMP_CXX
#   small_gicp::small_gicp
#   ${PCL_LIBRARIES}
# )

# add_executable(${PROJECT_NAME}
#   src/main.cpp
# )

# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_NAME}_core
# )

# #############
# ## Install ##
# #############

# # 安装可执行文件
# install(
#   TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY launch config maps
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()