message(STATUS "fetching Hull")
include(FetchContent)
FetchContent_Declare(
  hull
  GIT_REPOSITORY https://github.com/andreacasalino/Hull.git
  GIT_TAG        90f09a1baefcfe02ef8d5a3cb44a42536efe5060
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

