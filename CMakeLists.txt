project(stm32)
cmake_minimum_required(VERSION 3.9)
set(CMAKE_C_FLAGS_DEBUG "-g")

include(config.cmake)
include_directories(${CMAKE_BINARY_DIR})
configure_file(config.h.in	${CMAKE_BINARY_DIR}/config.h)

add_subdirectory(nes)
add_subdirectory(apps)
add_subdirectory(logging)
add_subdirectory(stm32lib)
