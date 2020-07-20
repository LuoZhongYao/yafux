if(ENABLE_SDL2)

add_executable(nesdl)
target_sources(nesdl PRIVATE
	sdl.c
	)
target_link_libraries(nesdl PRIVATE SDL2 nes)

endif(ENABLE_SDL2)
