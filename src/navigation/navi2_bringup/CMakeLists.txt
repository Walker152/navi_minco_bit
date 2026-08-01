cmake_minimum_required(VERSION 3.8)
project(navi2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_definitions(-Wall -Werror)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch maps config params
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
