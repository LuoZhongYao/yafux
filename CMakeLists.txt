project(stm32)
cmake_minimum_required(VERSION 3.9)
set(CMAKE_C_FLAGS_DEBUG "-g")

add_subdirectory(nes)
add_subdirectory(apps)
add_subdirectory(logging)
add_subdirectory(stm32lib)
