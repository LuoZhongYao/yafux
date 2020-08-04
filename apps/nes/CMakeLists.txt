add_executable(nes.elf)
target_sources(nes.elf PRIVATE
	main.c)
# ${CMAKE_SOURCE_DIR}/_syscalls.c)
target_link_libraries(nes.elf PRIVATE nes stm32 -Wl,--whole-archive logging -Wl,--no-whole-archive)

add_custom_command(TARGET nes.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY}
	ARGS -Oihex nes.elf nes.hex)
