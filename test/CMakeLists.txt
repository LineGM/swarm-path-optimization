# Parent project does not export its library target, so this CML implicitly
# depends on being added from it, i.e. the testing is done only from the build
# tree and is not feasible from an install location

project(swarm_optimizationTests LANGUAGES CXX)

project(
	swarm_optimizationTests
	VERSION 0.0.1
	DESCRIPTION "Tests for swarm optimization diploma project"
	HOMEPAGE_URL "LineGM@yandex.ru"
	LANGUAGES CXX
)

include(../cmake/project-is-top-level.cmake)
include(../cmake/folders.cmake)

if(PROJECT_IS_TOP_LEVEL)
	find_package(swarm_optimization_lib REQUIRED)
	enable_testing()
endif()

include(FetchContent)

set(BOOST.UT_VERSION_DOWNLOAD "2.0.1")
FetchContent_Declare(
	boost.ut
	GIT_REPOSITORY https://github.com/boost-ext/ut.git
	GIT_TAG v${BOOST.UT_VERSION_DOWNLOAD}
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/_deps_src/boost.ut-${BOOST.UT_VERSION_DOWNLOAD}
)
FetchContent_Populate(boost.ut)

# ---- Lib tests ----
add_executable(swarm_optimization_test source/swarm_optimization_test.cpp)

target_include_directories(
	swarm_optimization_test
	PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/source>"
	PUBLIC "$<BUILD_INTERFACE:${boost.ut_SOURCE_DIR}/include>"
)

target_compile_features(swarm_optimization_test PRIVATE cxx_std_20)

target_link_libraries(
    swarm_optimization_test 
    PRIVATE swarm_optimization_lib
)

add_test(NAME swarm_optimization_test COMMAND swarm_optimization_test)

# ---- End-of-file commands ----

add_folders(Test)
