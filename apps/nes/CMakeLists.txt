add_executable(nes)
target_sources(nes PRIVATE
	nes.c
	cpu.c
	ppu.c
	psg.c
	rom.c
	sdl.c
	mapper.c
	)

target_link_libraries(nes PRIVATE SDL2)
target_compile_options(nes PRIVATE -pg)
target_link_options(nes PRIVATE -pg)
