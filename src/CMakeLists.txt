message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        506a77bd926c0f448f070f2cfa58a0812edefeb3
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

