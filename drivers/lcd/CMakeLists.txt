add_library(lcd)
target_sources(lcd PRIVATE
	hx8357d.c)

target_include_directories(lcd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lcd PRIVATE stm32 mipi logging)
