cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)
option(BUILD_SAMPLES "Build the samples showing how to use the package" ON)

set(CMAKE_CXX_STANDARD 17)

# set macro-directory and find scripts
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# include cmake custom functions
set(WITH_SOURCE_TREE ON)
include(GroupSources)
include(AutoCollect)
include(MakeLibrary)

option(Fast-Quick-Hull-THREAD_POOL_OPT "thread pool enabled (ON) or disabled (OFF)" ON)
project(Fast-Quick-Hull-Prj)
add_subdirectory(src)

# if(BUILD_SAMPLES)
#    add_subdirectory(Samples)
# endif()
