file(GLOB_RECURSE SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(QuickHullUtils ${SRC})

target_link_libraries(QuickHullUtils PUBLIC
Fast-Quick-Hull
)

target_include_directories(QuickHullUtils PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(QuickHullUtils PRIVATE
    -D ANIMALS_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/Animals/"
)
