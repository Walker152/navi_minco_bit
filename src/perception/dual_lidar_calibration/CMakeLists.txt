cmake_minimum_required(VERSION 3.8)
project(dual_lidar_calibration)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_library(frame_synchronizer
  src/frame_synchronizer.cpp
)
target_compile_features(frame_synchronizer PUBLIC cxx_std_17)
target_include_directories(frame_synchronizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(calibration_core
  src/bag_reader.cpp
  src/calibration_config.cpp
  src/deskewer.cpp
  src/extrinsic_aggregator.cpp
  src/frame_calibrator.cpp
  src/imu_rotation_estimator.cpp
  src/offline_calibrator.cpp
  src/result_writer.cpp
)
target_compile_features(calibration_core PUBLIC cxx_std_17)
target_include_directories(calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(calibration_core PUBLIC
  livox_ros_driver2
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)
target_link_libraries(calibration_core PUBLIC
  frame_synchronizer
  Eigen3::Eigen
  yaml-cpp
  ${PCL_LIBRARIES}
)

add_executable(calibrate src/main.cpp)
target_compile_features(calibrate PUBLIC cxx_std_17)
ament_target_dependencies(calibrate rclcpp)
target_link_libraries(calibrate calibration_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_synchronizer
    test/test_frame_synchronizer.cpp
  )
  target_include_directories(test_frame_synchronizer PRIVATE include)
  target_link_libraries(test_frame_synchronizer frame_synchronizer)

  ament_add_gtest(test_imu_rotation_estimator
    test/test_imu_rotation_estimator.cpp
  )
  target_include_directories(test_imu_rotation_estimator PRIVATE include)
  target_link_libraries(test_imu_rotation_estimator calibration_core)

  ament_add_gtest(test_deskewer
    test/test_deskewer.cpp
  )
  target_include_directories(test_deskewer PRIVATE include)
  target_link_libraries(test_deskewer calibration_core)

  ament_add_gtest(test_extrinsic_aggregator
    test/test_extrinsic_aggregator.cpp
  )
  target_include_directories(test_extrinsic_aggregator PRIVATE include)
  target_link_libraries(test_extrinsic_aggregator calibration_core)

  ament_add_gtest(test_frame_calibrator
    test/test_frame_calibrator.cpp
  )
  target_include_directories(test_frame_calibrator PRIVATE include)
  target_link_libraries(test_frame_calibrator calibration_core)
endif()

install(TARGETS frame_synchronizer calibration_core calibrate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
