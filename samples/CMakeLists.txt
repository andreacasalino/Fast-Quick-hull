add_library(Utils
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/ImporterSTL.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/ImporterSTL.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/Logger.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/Logger.cpp
)

target_include_directories(Utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
)

target_link_libraries(Utils PUBLIC 
    Fast-Quick-Hull
)	

target_compile_definitions(Utils PUBLIC
    -D ANIMALS_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/Animals/"
)

# sample 01
add_executable(Sample01 Sample01.cpp)
target_link_libraries(Sample01 PUBLIC 
    Utils
)	
add_custom_command(TARGET Sample01 PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)

# sample 02
add_executable(Sample02 Sample02.cpp)
target_link_libraries(Sample02 PUBLIC 
    Utils
)		

add_custom_command(TARGET Sample02 PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)

add_custom_command(TARGET Sample02 PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Animals ${CMAKE_CURRENT_BINARY_DIR}/Animals
)

# for readme file
add_executable(ReadMe ReadMe.cpp)
target_link_libraries(ReadMe PUBLIC Fast-Quick-Hull)	
