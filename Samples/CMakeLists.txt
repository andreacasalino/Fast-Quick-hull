option(BUILD_EXAMPLE_01 "Build Example_01" ON)
if(BUILD_EXAMPLE_01)
    add_executable(Sample01 Sample01.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.h ${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.cpp)
    target_link_libraries(Sample01 PUBLIC Fast-Quick-Hull)	

    add_custom_command(TARGET Sample01 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endif()

option(BUILD_EXAMPLE_02 "Build Example_02" ON)
if(BUILD_EXAMPLE_02)
    add_executable(Sample02 Sample02.cpp 
	${CMAKE_CURRENT_SOURCE_DIR}/src/ImporterSTL.h ${CMAKE_CURRENT_SOURCE_DIR}/src/ImporterSTL.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.h ${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.cpp)
    target_link_libraries(Sample02 PUBLIC Fast-Quick-Hull)	

    add_custom_command(TARGET Sample02 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)

   add_custom_command(TARGET Sample02 PRE_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Animals ${CMAKE_CURRENT_BINARY_DIR}/Animals
    )
endif()

