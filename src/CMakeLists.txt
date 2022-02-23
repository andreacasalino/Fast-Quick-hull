message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        2d0732562c10294f9327b67e8c39d376c9d50f9a
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

