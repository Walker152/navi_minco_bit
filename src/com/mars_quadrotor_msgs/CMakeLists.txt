cmake_minimum_required(VERSION 3.5)

project(mars_quadrotor_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Collect message files
set(msg_files
        "ros2_msg/QuadrotorState.msg"
        "ros2_msg/TrakingPerformance.msg"
        "ros2_msg/PositionCommand.msg"
        "ros2_msg/PolynomialTrajectory.msg"
        "ros2_msg/MpcPositionCommand.msg"
)
# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs geometry_msgs
        ADD_LINTER_TESTS
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

# Finalize package
ament_package()