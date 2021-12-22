message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(hull)

set(PROJECT_SHORTNAME "Fast-Quick-Hull")

MakeLibrary(${PROJECT_SHORTNAME} header)

target_link_libraries(${PROJECT_SHORTNAME} PUBLIC
Hull
)

if(Fast-Quick-Hull-THREAD_POOL_OPT)
  SET(BUILD_Thread_Pool_SAMPLES OFF CACHE BOOL "Override thread pool sample option" FORCE)
  message(STATUS "fetching Thread Pool")
  FetchContent_Declare(
    th_pool
    GIT_REPOSITORY https://github.com/andreacasalino/Cpp-Thread-Pool.git
    GIT_TAG        master
  )
  FetchContent_MakeAvailable(th_pool)


  target_compile_definitions(${PROJECT_SHORTNAME} PUBLIC THREAD_POOL_ENABLED)
  target_link_libraries(${PROJECT_SHORTNAME} PUBLIC
    Thread-Pool
  )
endif()

