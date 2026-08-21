cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pcd2pgm.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
