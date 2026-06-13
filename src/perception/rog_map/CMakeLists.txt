cmake_minimum_required(VERSION 3.8)
project(rog_map)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)
        add_compile_options(-Wall -Wextra -Wpedantic)
        add_compile_options(-Werror=unused-variable -Werror=return-type)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(
        QT_NO_VERSION_TAGGING
        ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
        ORIGIN_AT_CORNER
        USE_ROS2
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

set(library_name ${PROJECT_NAME})

file(GLOB_RECURSE ROG_MAP_SOURCES CONFIGURE_DEPENDS
        src/rog_map/*.cpp
        src/rog_map_ros/*.cpp
        include/rog_map/rog_map_core/raycaster.cpp
)

add_library(${library_name} STATIC
        ${ROG_MAP_SOURCES}
)

target_compile_definitions(${library_name}
        PUBLIC
                QT_NO_VERSION_TAGGING
                ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
                ORIGIN_AT_CORNER
                USE_ROS2
)

target_include_directories(${library_name}
        PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        PRIVATE
                ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${library_name}
        rclcpp
        rclcpp_lifecycle
        nav2_util
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_conversions
        tf2_ros
)

target_link_libraries(${library_name}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
        target_link_libraries(${library_name} OpenMP::OpenMP_CXX)
endif()

if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(projection_layer_height_ratio_test
                test/projection_layer_height_ratio_test.cpp
        )
        target_link_libraries(projection_layer_height_ratio_test
                ${library_name}
        )

        ament_add_gtest(query_adapter_status_test
                test/query_adapter_status_test.cpp
        )
        target_link_libraries(query_adapter_status_test
                ${library_name}
        )
endif()

install(TARGETS ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        nav2_util
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_conversions
        tf2_ros
)

ament_package()
