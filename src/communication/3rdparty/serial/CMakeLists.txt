cmake_minimum_required(VERSION 3.5)
project(serial)

if(APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
endif()

set(serial_SRCS
    src/serial.cc
    src/impl/unix.cc               # 视平台调整，这里示例用 unix
    src/impl/list_ports/list_ports_linux.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

add_library(serial ${serial_SRCS})

target_include_directories(serial PUBLIC include)

if(APPLE)
    target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(serial rt pthread)
else()
    # windows or other systems
    target_link_libraries(serial setupapi)
endif()