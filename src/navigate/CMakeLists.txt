cmake_minimum_required(VERSION 3.8)
project(navigate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY launch map config rviz 
DESTINATION share/${PROJECT_NAME})

ament_package()
