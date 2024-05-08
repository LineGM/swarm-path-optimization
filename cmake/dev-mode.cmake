include(cmake/folders.cmake)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

if (WIN32 AND MINGW)
  add_custom_target(
    run-exe
    COMMAND ./swarm_optimization.exe
    VERBATIM
  )
  add_dependencies(run-exe swarm_optimization_exe)
else()
  add_custom_target(
    run-exe
    COMMAND ./swarm_optimization
    VERBATIM
  )
  add_dependencies(run-exe swarm_optimization_exe)
endif()

option(BUILD_DOCS "Build documentation using Doxygen" OFF)
if(BUILD_DOCS)
  include(cmake/docs.cmake)
endif()

option(ENABLE_COVERAGE "Enable coverage support separate from CTest's" OFF)
if(ENABLE_COVERAGE)
  include(cmake/coverage.cmake)
endif()

include(cmake/lint-targets.cmake)
include(cmake/spell-targets.cmake)

add_folders(Project)
