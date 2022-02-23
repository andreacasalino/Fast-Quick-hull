message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        c0517c4b9a568fd138d551e58ad09ed5d9ad5b9c
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

