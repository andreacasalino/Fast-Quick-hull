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

target_compile_definitions(Utils PRIVATE
    -D ANIMALS_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/Animals/"
)

macro(MAKE_SAMPLE NAME)
add_executable(${NAME} ${NAME}.cpp)
target_link_libraries(${NAME} PUBLIC 
    Utils
)	
add_custom_command(TARGET ${NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
endmacro()


# sample 01
MAKE_SAMPLE(Sample01)

# sample 02
MAKE_SAMPLE(Sample02)

# for readme file
add_executable(ReadMe ReadMe.cpp)
target_link_libraries(ReadMe PUBLIC Fast-Quick-Hull)	
