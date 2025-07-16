cmake_minimum_required(VERSION 3.8)
project(simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

install(
  DIRECTORY launch urdf config meshes world xacro map 
  DESTINATION share/${PROJECT_NAME}  
)

ament_package()
