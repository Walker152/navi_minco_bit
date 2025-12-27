cmake_minimum_required(VERSION 3.5)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

# reduce warning
add_compile_options(-Wno-dev)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)

string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

message("Using ROS2")
add_definitions(-DUSE_ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        yaml-cpp
)

set(ros_libs
        rclcpp
        std_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        pcl_conversions
        tf2_ros
)

include_directories(
        ${includes}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)

add_library(rog_map STATIC ${srcs})

target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        ${THIRD_PARTY})


ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_libs}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${third_party_libs}
)


install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}
        ${third_party_libs}
)
ament_export_include_directories(${includes})
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()
