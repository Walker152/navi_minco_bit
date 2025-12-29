cmake_minimum_required(VERSION 3.10)
project(minco_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

# qpOASES: 尝试 find_package；如果系统未提供 CMake config，则回退到手动查找。
find_package(qpOASES QUIET)
if(NOT qpOASES_FOUND)
  find_path(QPOASES_INCLUDE_DIR qpOASES.hpp PATH_SUFFIXES qpOASES qpoases)
  find_library(QPOASES_LIBRARY qpOASES)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mpc_solver.cpp
  src/minco_mpc_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

set(_QPOASES_LINKED FALSE)

set(_QPOASES_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES)
if(EXISTS "${_QPOASES_VENDOR_DIR}/include/qpOASES.hpp" AND EXISTS "${_QPOASES_VENDOR_DIR}/src")
  file(GLOB _QPOASES_VENDOR_SRC "${_QPOASES_VENDOR_DIR}/src/*.cpp")
  add_library(qpOASES_vendor STATIC ${_QPOASES_VENDOR_SRC})
  set_target_properties(qpOASES_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(qpOASES_vendor PUBLIC "${_QPOASES_VENDOR_DIR}/include")
  target_compile_definitions(qpOASES_vendor PRIVATE __NO_COPYRIGHT__ LINUX)

  target_link_libraries(${PROJECT_NAME} qpOASES_vendor)
  set(_QPOASES_LINKED TRUE)
endif()

pluginlib_export_plugin_description_file(nav2_core minco_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
