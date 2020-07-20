add_executable(led.elf)
target_sources(led.elf PRIVATE
	main.c
	${CMAKE_SOURCE_DIR}/_syscalls.c)
target_link_libraries(led.elf PRIVATE stm32)

add_custom_command(TARGET led.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY}
	ARGS -Oihex led.elf ${CMAKE_BINARY_DIR}/led.hex)
