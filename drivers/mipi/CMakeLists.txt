add_library(mipi)
target_sources(mipi PRIVATE
	mipi.c)

target_link_libraries(mipi PRIVATE stm32 logging)
target_include_directories(mipi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
