# cmake_minimum_required(VERSION 3.0.2)
# project(communication)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(EXE_NAME sentry)

# # file(GLOB_RECURSE source CONFIGURE_DEPENDS
# #   src/behavier_tree_com.cpp
# #   src/main.cpp
# # )

# set(source
#     src/behavier_tree_com.cpp
#     src/main.cpp
#     /home/rm/sentry/src/communication/RM_encoder/communication.cpp
# )

# add_definitions(-DREFEREE_DEBUG=0)
# add_definitions(-DSEND_DEBUG=0)
# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   move_base_msgs
#   nav_msgs
#   robots_msgs
#   roscpp
#   rospy
#   serial
#   std_msgs
#   )

# find_package(Threads)
# set(Libevent_DIR "/usr/local/lib/cmake/libevent")
# find_package(Libevent REQUIRED)


# catkin_package(
#  INCLUDE_DIRS include
# #  LIBRARIES communication
#  CATKIN_DEPENDS geometry_msgs move_base_msgs nav_msgs robots_msgs roscpp rospy serial std_msgs
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${CERES_INCLUDE_DIRS}
#     my-utils/MyUtils
#     src/communication/RM_encoder
# )

# AUX_SOURCE_DIRECTORY(src/communication/RM_encoder DIR_SRCS)

# add_executable(${EXE_NAME} ${source} ${DIR_SRCS})

# add_dependencies(${EXE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${EXE_NAME} 
#   ${catkin_LIBRARIES}
#   ${CMAKE_THREAD_LIBS_INIT}
#   libevent::core
#   libevent::pthreads
# )

# # Add third-party libraries
# add_subdirectory(3rdparty/fmt)
# target_link_libraries(${EXE_NAME} fmt)

# add_subdirectory(3rdparty/plog-1.1.10)
# target_link_libraries(${EXE_NAME} plog)

# add_subdirectory(3rdparty/sheriffos)
# target_link_libraries(${EXE_NAME} sheriff_os)