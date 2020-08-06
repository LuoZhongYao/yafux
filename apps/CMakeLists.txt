add_subdirectory(nes)
add_subdirectory(logging)

if(CONFIG_SDL2)
	add_subdirectory(sdl)
endif(CONFIG_SDL2)
