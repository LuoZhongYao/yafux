add_library(nes)
target_sources(nes PRIVATE
	nes.c
	cpu.c
	ppu.c
	psg.c
	rom.c
	mapper.c
	)

target_include_directories(nes INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(nes PRIVATE -pg)
target_link_libraries(nes PRIVATE logging)
