add_library(logging)
target_include_directories(logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(logging PRIVATE stm32)
target_sources(logging PRIVATE 
	led_log.c
	usart_log.c
	semihosting_log.c
	)
