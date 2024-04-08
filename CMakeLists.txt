cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GroupSources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AutoCollect.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MakeLibrary.cmake)

project(Fast-Quick-Hull-Prj)

option(Fast-Quick-Hull-THREAD_POOL_OPT "thread pool enabled (ON) or disabled (OFF)" ON)
option(Fast-Quick-Hull-BUILD_SAMPLES "Build the samples showing how to use the package" ON)
option(Fast-Quick-Hull-BUILD_TESTS "" OFF)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

add_subdirectory(src)

if(Fast-Quick-Hull-BUILD_TESTS OR Fast-Quick-Hull-BUILD_SAMPLES)
    add_subdirectory(utils)
endif()

if(Fast-Quick-Hull-BUILD_SAMPLES)
   add_subdirectory(samples)
endif()

if(Fast-Quick-Hull-BUILD_TESTS)
    add_subdirectory(tests)
endif()
