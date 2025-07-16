cmake_minimum_required(VERSION 3.8)
project(base_demo)
# 指定编译版本
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)


add_executable(suber src/sub.cpp)
ament_target_dependencies(suber rclcpp std_msgs robot_msgs)

add_executable(puber src/pub.cpp)
ament_target_dependencies(puber rclcpp std_msgs robot_msgs)

install(TARGETS
  suber
  puber
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME})
ament_package()
