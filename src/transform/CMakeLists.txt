cmake_minimum_required(VERSION 3.8)
project(tranform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(board_caster src/boardcaster.cpp)
ament_target_dependencies(board_caster
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
)

install(TARGETS
    board_caster
    listener
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
