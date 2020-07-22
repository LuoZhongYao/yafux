add_subdirectory(led)
add_subdirectory(logging)

if(ENABLED_SDL2)
	add_subdirectory(sdl)
endif(ENABLED_SDL2)
