add_executable(nesdl)
target_sources(nesdl PRIVATE
	sdl.c
	)
target_link_libraries(nesdl PRIVATE SDL2 nes -Wl,--whole-archive logging -Wl,--no-whole-archive)
