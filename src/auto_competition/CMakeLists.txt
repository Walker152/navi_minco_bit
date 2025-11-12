# cmake_minimum_required(VERSION 3.8)
# project(auto_rm)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   set(CMAKE_CXX_STANDARD 17)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
#   add_compile_options(-Wall -Wextra -Wpedantic)
#   set(CMAKE_BUILD_TYPE "Release")
#   add_definitions(-Wall -Werror)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
#   set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# endif()


# ## Find catkin macros and libraries
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)


# ###########
# ## Build ##
# ###########

# include_directories(
#   include
# )

# ## Declare a cpp library
# add_library(${PROJECT_NAME}_core
#   src/checker.cpp
#   src/main.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_core
#   rclcpp
#   std_msgs
  
# )
# # target_link_libraries(${PROJECT_NAME}_core
  
# # )

# ## Declare cpp executables
# add_executable(${PROJECT_NAME}
#   src/main.cpp
# )
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core )

# #############
# ## Install ##
# #############

# # Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
