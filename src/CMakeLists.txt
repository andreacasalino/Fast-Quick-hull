message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        bed032cbfb6c9f36ff581ca132803a250809d37e
)
FetchContent_MakeAvailable(hull)

set(PROJECT_SHORTNAME "Fast-Quick-Hull")

MakeLibrary(${PROJECT_SHORTNAME} header)

target_link_libraries(${PROJECT_SHORTNAME} PUBLIC
Hull
)

find_package(OpenMP)
target_link_libraries(${PROJECT_SHORTNAME}  PRIVATE
	OpenMP::OpenMP_CXX
)

