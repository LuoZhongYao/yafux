add_library(gprof)
target_sources(gprof PRIVATE
	gmon.c
	profil.c
	profiler.S
	)

target_link_libraries(gprof PRIVATE logging)

set_source_files_properties(profiler.S PROPERTIES LANGUAGE C)
