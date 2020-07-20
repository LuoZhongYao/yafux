add_executable(nes.elf)
target_sources(nes.elf PRIVATE
	main.c
	${CMAKE_SOURCE_DIR}/_syscalls.c)
target_link_libraries(nes.elf PRIVATE stm32 logging)

add_custom_command(TARGET nes.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY}
	ARGS -Oihex nes.elf nes.hex)
