add_executable(log.elf)
target_sources(log.elf PRIVATE
	main.c)
# ${CMAKE_SOURCE_DIR}/_syscalls.c)
target_link_libraries(log.elf PRIVATE stm32 -Wl,--whole-archive logging -Wl,--no-whole-archive)

add_custom_command(TARGET log.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY}
	ARGS -Oihex log.elf log.hex)
