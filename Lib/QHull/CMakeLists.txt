set(PROJECT_SHORTNAME "Fast-Quick-Hull")
project(${PROJECT_SHORTNAME} VERSION ${VERSION} LANGUAGES CXX)
string(REPLACE - _ COMPONENT_NAME ${PROJECT_NAME})

CollectSourceFiles(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)

GroupSources(${CMAKE_CURRENT_SOURCE_DIR})

if(LIB_OPT)
	add_library(${PROJECT_NAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_NAME} STATIC ${SOURCES})
endif()

target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header>
        $<INSTALL_INTERFACE:header>
)

if(THREAD_POOL_OPT)
target_compile_definitions(${PROJECT_NAME} PUBLIC THREAD_POOL_ENABLED)
target_link_libraries(
${PROJECT_NAME}
    PRIVATE:
	thpl::Thread-Pool
)
endif()
