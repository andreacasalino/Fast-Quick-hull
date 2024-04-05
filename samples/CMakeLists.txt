macro(MAKE_SAMPLE NAME)
    add_executable(${NAME} ${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC 
    QuickHullUtils
    )	
    add_custom_command(TARGET ${NAME} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Plotter.py ${CMAKE_CURRENT_BINARY_DIR}/Plotter.py)
    target_compile_definitions(${NAME} PRIVATE
        -D PYTHON_CMD="${Python3_EXECUTABLE}"
    )
endmacro()


# sample 01
MAKE_SAMPLE(Sample01)

# sample 02
MAKE_SAMPLE(Sample02)

# for readme file
add_executable(ReadMe ReadMe.cpp)
target_link_libraries(ReadMe PUBLIC Fast-Quick-Hull)	
