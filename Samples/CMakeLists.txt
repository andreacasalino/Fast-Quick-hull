option(BUILD_EXAMPLE_01 "Build Example_01" ON)
if(BUILD_EXAMPLE_01)
    add_executable(Sample01 Sample01.cpp)
    target_link_libraries(Sample01 PUBLIC Fast-Quick-Hull)	

    add_custom_command(TARGET Sample01 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endif()

