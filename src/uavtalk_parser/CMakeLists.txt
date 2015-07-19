
add_library (uavtalk_parser uavtalk_parser)

SET_TARGET_PROPERTIES(uavtalk_parser PROPERTIES LINKER_LANGUAGE C)
target_link_libraries (uavtalk_parser ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (uavtalk_parser uquad_time)
